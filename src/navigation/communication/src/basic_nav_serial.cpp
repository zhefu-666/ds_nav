#include "utils/basic_nav_packet.hpp"
#include "utils/rmul_command_packet.hpp"
#include "utils/rmul_status_packet.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <ros_interfaces/msg/behavior.hpp>
#include <ros_interfaces/msg/rmul_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace ns_com::basic_nav
{
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
using RmulStatus = ros_interfaces::msg::RmulStatus;
using Behavior = ros_interfaces::msg::Behavior;

class BasicNavSerial final : public rclcpp::Node
{
public:
  BasicNavSerial()
  : Node("basic_nav_serial"), port_(declare_parameter("port", std::string("/dev/ttyACM0"))),
    rmul_v1_(declare_parameter("rmul_v1", false))
  {
    command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_mpc", rclcpp::QoS(1).best_effort().durability_volatile(),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        const auto x = msg->linear.x;
        const auto y = msg->linear.y;
        constexpr auto max = std::numeric_limits<float>::max();
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(msg->angular.z) ||
            std::abs(x) > max || std::abs(y) > max) {
          command_received_.reset();
          return;
        }
        linear_x_ = static_cast<float>(x);
        linear_y_ = static_cast<float>(y);
        command_received_ = Clock::now();
      });
    if (rmul_v1_) {
      status_pub_ = create_publisher<RmulStatus>("/sentry/rmul_status", rclcpp::QoS(1));
      behavior_sub_ = create_subscription<Behavior>(
        "/sentry/behaivor_send", rclcpp::QoS(1),
        [this](Behavior::ConstSharedPtr msg) {
          if (msg->desire_lifter_pos == Behavior::LIFTER_TOP ||
              msg->desire_lifter_pos == Behavior::LIFTER_BOTTOM) {
            lifter_target_down_ = msg->desire_lifter_pos;
            tunnel_prepare_active_ = msg->tunnel_prepare_active;
            tunnel_ready_ = msg->tunnel_ready;
            behavior_received_ = Clock::now();
          } else {
            behavior_received_.reset();
          }
        });
    }
    timer_ = create_wall_timer(20ms, [this]() { sendTick(); });
    RCLCPP_INFO(get_logger(), "Fixed zero-heading commissioning (%s) on %s",
      rmul_v1_ ? "RMUL V1" : "basic navigation", port_.c_str());
  }

  ~BasicNavSerial() override
  {
    closePort();
  }

private:
  void closePort()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
    pending_offset_ = 0;
    status_size_ = 0;
    status_received_.reset();
    command_received_.reset();
  }

  bool openPort()
  {
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ >= 0) {
      termios settings{};
      if (::ioctl(fd_, TIOCEXCL) == 0 && ::tcgetattr(fd_, &settings) == 0) {
        ::cfmakeraw(&settings);
        settings.c_iflag &= ~(IXOFF | IXANY);
        settings.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
        settings.c_cflag |= CS8 | CLOCAL | CREAD;
        if (::cfsetispeed(&settings, B115200) == 0 && ::cfsetospeed(&settings, B115200) == 0 &&
            ::tcsetattr(fd_, TCSANOW, &settings) == 0 && ::tcflush(fd_, TCIOFLUSH) == 0) {
          command_received_.reset();  // 首帧为零速，重连后等待新的 MPC 指令。
          RCLCPP_INFO(get_logger(), "Opened %s at 115200 8N1", port_.c_str());
          return true;
        }
      }
    }
    RCLCPP_WARN(get_logger(), "Cannot open/configure %s (errno %d)", port_.c_str(), errno);
    closePort();
    return false;
  }

  void acceptStatus()
  {
    constexpr auto checksum_offset = offsetof(rmul::StatusPacket, checksum);
    const auto read16 = [this](std::size_t offset) {
      return static_cast<std::uint16_t>(
        status_buffer_[offset] | (static_cast<std::uint16_t>(status_buffer_[offset + 1]) << 8));
    };
    if (dustSerialCrc(status_buffer_.data(), checksum_offset) != read16(checksum_offset)) {
      // After a damaged candidate, retain the next possible frame header and any bytes after it.
      const auto next = std::find(
        status_buffer_.begin() + 1, status_buffer_.end(), rmul::StatusPacket::kHeader);
      status_size_ = static_cast<std::size_t>(status_buffer_.end() - next);
      std::memmove(status_buffer_.data(), status_buffer_.data() + (status_buffer_.size() - status_size_),
        status_size_);
      return;
    }

    status_size_ = 0;
    RmulStatus msg;
    msg.sequence = status_buffer_[offsetof(rmul::StatusPacket, sequence)];
    msg.game_type = status_buffer_[offsetof(rmul::StatusPacket, game_type)];
    msg.stage_remain_s = read16(offsetof(rmul::StatusPacket, stage_remain_s));
    msg.game_progress = status_buffer_[offsetof(rmul::StatusPacket, game_progress)];
    msg.current_hp = read16(offsetof(rmul::StatusPacket, current_hp));
    msg.maximum_hp = read16(offsetof(rmul::StatusPacket, maximum_hp));
    msg.robot_id = status_buffer_[offsetof(rmul::StatusPacket, robot_id)];
    const auto power = status_buffer_[offsetof(rmul::StatusPacket, chassis_power_enabled)];
    msg.chassis_power_enabled = power == 1;
    msg.zone_state_and_rfid = read16(offsetof(rmul::StatusPacket, zone_state_and_rfid));
    msg.ally_control_degree = read16(offsetof(rmul::StatusPacket, ally_control_degree));
    msg.enemy_control_degree = read16(offsetof(rmul::StatusPacket, enemy_control_degree));
    msg.lifter_position = status_buffer_[offsetof(rmul::StatusPacket, lifter_position)];
    if (power > 1 || msg.lifter_position > 2) {
      status_received_.reset();
      return;
    }
    status_ = msg;
    status_received_ = Clock::now();
    status_pub_->publish(msg);
  }

  bool readStatus()
  {
    std::array<std::uint8_t, 512> input{};
    const auto count = ::read(fd_, input.data(), input.size());
    if (count < 0) {
      return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR;
    }
    for (ssize_t i = 0; i < count; ++i) {
      const auto byte = input[static_cast<std::size_t>(i)];
      if (status_size_ == 0 && byte != rmul::StatusPacket::kHeader) {
        continue;
      }
      status_buffer_[status_size_++] = byte;
      if (status_size_ == status_buffer_.size()) {
        acceptStatus();
      }
    }
    return true;
  }

  bool rmulMotionAllowed(Clock::time_point now) const
  {
    if (!status_received_ || now - *status_received_ >= 250ms ||
        status_.game_type != 4 || status_.game_progress != 4 ||
        status_.maximum_hp == 0 || status_.current_hp == 0 || !status_.chassis_power_enabled) {
      return false;
    }
    if (lifter_target_down_ == Behavior::LIFTER_BOTTOM || tunnel_prepare_active_) {
      return behavior_received_ && now - *behavior_received_ < 250ms &&
             lifter_target_down_ == Behavior::LIFTER_BOTTOM &&
             status_.lifter_position == Behavior::LIFTER_BOTTOM &&
             (!tunnel_prepare_active_ || tunnel_ready_);
    }
    return status_.lifter_position == Behavior::LIFTER_TOP;
  }

  void prepareFrame(float x, float y)
  {
    if (!rmul_v1_) {
      const auto frame = encodeFixedHeadingCommand(x, y);
      std::copy(frame.begin(), frame.end(), pending_.begin());
      pending_size_ = frame.size();
      return;
    }
    pending_.fill(0);
    pending_[0] = rmul::CommandPacket::kHeader;
    encodeFloat(x, pending_.data() + offsetof(rmul::CommandPacket, linear_x));
    encodeFloat(y, pending_.data() + offsetof(rmul::CommandPacket, linear_y));
    pending_[offsetof(rmul::CommandPacket, lifter_target_down)] = lifter_target_down_;
    constexpr auto checksum_offset = offsetof(rmul::CommandPacket, checksum);
    const auto crc = dustSerialCrc(pending_.data(), checksum_offset);
    pending_[checksum_offset] = static_cast<std::uint8_t>(crc);
    pending_[checksum_offset + 1] = static_cast<std::uint8_t>(crc >> 8);
    pending_size_ = pending_.size();
  }

  void sendTick()
  {
    const auto attempt_time = Clock::now();
    if (fd_ < 0) {
      if (attempt_time < next_open_attempt_) {
        return;
      }
      next_open_attempt_ = attempt_time + 1s;
      if (!openPort()) {
        return;
      }
    }

    if (rmul_v1_ && !readStatus()) {
      RCLCPP_WARN(get_logger(), "Serial read failed on %s (errno %d)", port_.c_str(), errno);
      closePort();
      next_open_attempt_ = Clock::now() + 1s;
      return;
    }
    const auto now = Clock::now();
    if (rmul_v1_ && !rmulMotionAllowed(now)) {
      command_received_.reset();
    }
    // 完成一次短写时使用同一帧。若此时指令已失效，直接断开以免补完过期帧。
    const bool command_fresh = command_received_ && now - *command_received_ < 100ms;
    if (pending_offset_ != 0 &&
        (now - pending_since_ >= 100ms ||
        (rmul_v1_ && pending_lifter_target_ != lifter_target_down_) ||
        (pending_command_received_ &&
        (!command_fresh || now - *pending_command_received_ >= 100ms)))) {
      closePort();
      next_open_attempt_ = now + 1s;
      return;
    }
    if (pending_offset_ == 0) {
      prepareFrame(
        command_fresh ? linear_x_ : 0.0F, command_fresh ? linear_y_ : 0.0F);
      pending_since_ = now;
      pending_lifter_target_ = lifter_target_down_;
      pending_command_received_ = command_fresh ? command_received_ : std::nullopt;
    }

    const auto written = ::write(fd_, pending_.data() + pending_offset_, pending_size_ - pending_offset_);
    if (written < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
        return;
      }
      RCLCPP_WARN(get_logger(), "Serial write failed on %s (errno %d)", port_.c_str(), errno);
      closePort();
      next_open_attempt_ = now + 1s;
      return;
    }
    pending_offset_ += static_cast<std::size_t>(written);
    if (pending_offset_ == pending_size_) {
      pending_offset_ = 0;
    }
  }

  const std::string port_;
  const bool rmul_v1_;
  std::array<std::uint8_t, rmul::StatusPacket::kSize> status_buffer_{};
  std::size_t status_size_ = 0;
  RmulStatus status_;
  std::optional<Clock::time_point> status_received_;
  std::optional<Clock::time_point> behavior_received_;
  std::uint8_t lifter_target_down_ = Behavior::LIFTER_TOP;
  bool tunnel_prepare_active_ = false;
  bool tunnel_ready_ = false;
  rclcpp::Publisher<RmulStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<Behavior>::SharedPtr behavior_sub_;

  int fd_ = -1;
  Clock::time_point next_open_attempt_{};
  std::optional<Clock::time_point> command_received_;
  float linear_x_ = 0.0F;
  float linear_y_ = 0.0F;
  static_assert(rmul::CommandPacket::kSize >= CmdVelPacket::kSize);
  std::array<std::uint8_t, rmul::CommandPacket::kSize> pending_{};
  std::size_t pending_size_ = 0;
  std::uint8_t pending_lifter_target_ = Behavior::LIFTER_TOP;
  std::size_t pending_offset_ = 0;
  Clock::time_point pending_since_{};
  std::optional<Clock::time_point> pending_command_received_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace ns_com::basic_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ns_com::basic_nav::BasicNavSerial>());
  rclcpp::shutdown();
  return 0;
}
