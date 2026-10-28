#include <chrono>
#include <algorithm>
#include <cmath>
#include <memory>
#include <limits>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{

geometry_msgs::msg::Twist makeStopTwist()
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.0;
  return twist;
}

double normalizeYaw(double yaw)
{
  constexpr double kPi = 3.14159265358979323846;
  while (yaw > kPi) {
    yaw -= 2.0 * kPi;
  }
  while (yaw < -kPi) {
    yaw += 2.0 * kPi;
  }
  return yaw;
}

double distance2d(double x0, double y0, double x1, double y1)
{
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  return std::hypot(dx, dy);
}

double quaternionToYaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

class PathFollowerNode : public rclcpp::Node
{
public:
  PathFollowerNode()
  : Node("path_follower_node"),
    tf_buffer_(this->get_clock())
  {
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    path_topic_ = this->declare_parameter<std::string>("path_topic", "/planned_path");
    cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

    control_frequency_ = this->declare_parameter<double>("control_frequency", 20.0);
    if (control_frequency_ <= 0.0) {
      control_frequency_ = 20.0;
    }

    lookahead_distance_ = this->declare_parameter<double>("lookahead_distance", 0.6);
    goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.20);
    linear_speed_ = this->declare_parameter<double>("linear_speed", 0.45);
    max_linear_speed_ = this->declare_parameter<double>("max_linear_speed", 0.60);
    max_angular_speed_ = this->declare_parameter<double>("max_angular_speed", 1.5);
    heading_stop_threshold_ = this->declare_parameter<double>("heading_stop_threshold", 1.05);

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      path_topic_,
      10,
      std::bind(&PathFollowerNode::pathCallback, this, std::placeholders::_1));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_, this);

    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / control_frequency_));
    timer_ = this->create_wall_timer(
      period,
      std::bind(&PathFollowerNode::controlLoop, this));

    RCLCPP_INFO(
      this->get_logger(),
      "path_follower_node started. path=%s cmd_vel=%s map=%s base=%s",
      path_topic_.c_str(),
      cmd_vel_topic_.c_str(),
      map_frame_.c_str(),
      base_frame_.c_str());
  }

private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    const bool goal_changed =
      !path_ || path_->poses.empty() || msg->poses.empty() ||
      distance2d(
        path_->poses.back().pose.position.x,
        path_->poses.back().pose.position.y,
        msg->poses.back().pose.position.x,
        msg->poses.back().pose.position.y) > 1e-3;

    path_ = msg;
    if (goal_changed) {
      goal_reached_ = false;
    }
  }

  void controlLoop()
  {
    if (!path_ || path_->poses.empty()) {
      publishStop();
      return;
    }

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      if (last_tf_error_ != ex.what()) {
        RCLCPP_WARN(this->get_logger(), "Waiting for TF %s -> %s: %s",
          map_frame_.c_str(), base_frame_.c_str(), ex.what());
        last_tf_error_ = ex.what();
      }
      publishStop();
      return;
    }

    last_tf_error_.clear();

    const double current_x = tf.transform.translation.x;
    const double current_y = tf.transform.translation.y;
    const double current_yaw = quaternionToYaw(tf.transform.rotation);
    const double goal_x = path_->poses.back().pose.position.x;
    const double goal_y = path_->poses.back().pose.position.y;

    if (distance2d(current_x, current_y, goal_x, goal_y) <= goal_tolerance_) {
      if (!goal_reached_) {
        RCLCPP_INFO(this->get_logger(), "Goal reached.");
        goal_reached_ = true;
      }
      publishStop();
      return;
    }

    const std::size_t target_index = selectTargetIndex(current_x, current_y);
    const auto & target_pose = path_->poses[target_index].pose.position;

    const double dx = target_pose.x - current_x;
    const double dy = target_pose.y - current_y;
    const double target_yaw = std::atan2(dy, dx);
    const double heading_error = normalizeYaw(target_yaw - current_yaw);

    double linear_speed = linear_speed_;
    const double target_distance = distance2d(current_x, current_y, target_pose.x, target_pose.y);
    if (target_distance < lookahead_distance_) {
      linear_speed *= std::max(0.2, target_distance / std::max(lookahead_distance_, 1e-3));
    }

    if (std::abs(heading_error) > heading_stop_threshold_) {
      linear_speed = 0.0;
    }

    const double x_robot = std::cos(-current_yaw) * dx - std::sin(-current_yaw) * dy;
    const double y_robot = std::sin(-current_yaw) * dx + std::cos(-current_yaw) * dy;
    const double curvature = (lookahead_distance_ > 1e-3) ?
      (2.0 * y_robot / (lookahead_distance_ * lookahead_distance_)) : 0.0;

    geometry_msgs::msg::Twist cmd = makeStopTwist();
    cmd.linear.x = std::clamp(linear_speed, 0.0, max_linear_speed_);
    cmd.angular.z = std::clamp(cmd.linear.x * curvature, -max_angular_speed_, max_angular_speed_);

    if (x_robot < 0.0 && std::abs(heading_error) > 0.35) {
      cmd.linear.x = 0.0;
      cmd.angular.z = std::clamp(heading_error * 1.2, -max_angular_speed_, max_angular_speed_);
    }

    cmd_vel_pub_->publish(cmd);
  }

  std::size_t selectTargetIndex(double current_x, double current_y) const
  {
    if (!path_ || path_->poses.empty()) {
      return 0;
    }

    std::size_t nearest_index = 0;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < path_->poses.size(); ++i) {
      const auto & pose = path_->poses[i].pose.position;
      const double candidate_distance = distance2d(current_x, current_y, pose.x, pose.y);
      if (candidate_distance < nearest_distance) {
        nearest_distance = candidate_distance;
        nearest_index = i;
      }
    }

    std::size_t target_index = nearest_index;
    double accumulated_distance = 0.0;
    for (std::size_t i = nearest_index + 1; i < path_->poses.size(); ++i) {
      const auto & previous = path_->poses[i - 1].pose.position;
      const auto & current = path_->poses[i].pose.position;
      accumulated_distance += distance2d(previous.x, previous.y, current.x, current.y);
      target_index = i;
      if (accumulated_distance >= lookahead_distance_) {
        break;
      }
    }

    return target_index;
  }

  void publishStop()
  {
    cmd_vel_pub_->publish(makeStopTwist());
  }

  std::string map_frame_;
  std::string base_frame_;
  std::string path_topic_;
  std::string cmd_vel_topic_;

  double control_frequency_ = 20.0;
  double lookahead_distance_ = 0.6;
  double goal_tolerance_ = 0.20;
  double linear_speed_ = 0.45;
  double max_linear_speed_ = 0.60;
  double max_angular_speed_ = 1.5;
  double heading_stop_threshold_ = 1.05;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  nav_msgs::msg::Path::SharedPtr path_;
  bool goal_reached_ = false;
  std::string last_tf_error_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
