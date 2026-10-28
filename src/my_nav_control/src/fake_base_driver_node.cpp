#include <chrono>
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/utils.h"
#include "tf2_ros/transform_broadcaster.h"

namespace
{

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  const double half_yaw = yaw * 0.5;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(half_yaw);
  q.w = std::cos(half_yaw);
  return q;
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

}  // namespace

class FakeBaseDriverNode : public rclcpp::Node
{
public:
  FakeBaseDriverNode()
  : Node("fake_base_driver_node")
  {
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/odom");

    publish_rate_ = this->declare_parameter<double>("publish_rate", 50.0);
    if (publish_rate_ <= 0.0) {
      publish_rate_ = 50.0;
    }

    cmd_timeout_ = this->declare_parameter<double>("cmd_timeout", 0.25);
    max_linear_speed_ = this->declare_parameter<double>("max_linear_speed", 0.8);
    max_angular_speed_ = this->declare_parameter<double>("max_angular_speed", 2.0);
    initial_x_ = this->declare_parameter<double>("initial_x", 0.0);
    initial_y_ = this->declare_parameter<double>("initial_y", 0.0);
    initial_yaw_ = this->declare_parameter<double>("initial_yaw", 0.0);

    x_ = initial_x_;
    y_ = initial_y_;
    yaw_ = initial_yaw_;

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_,
      10,
      std::bind(&FakeBaseDriverNode::cmdVelCallback, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_));
    timer_ = this->create_wall_timer(
      period,
      std::bind(&FakeBaseDriverNode::update, this));

    last_update_time_ = this->now();
    last_cmd_time_ = this->now();

    RCLCPP_INFO(
      this->get_logger(),
      "fake_base_driver_node started. odom=%s base=%s cmd_vel=%s",
      odom_topic_.c_str(),
      base_frame_.c_str(),
      cmd_vel_topic_.c_str());
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    current_cmd_.linear.x = std::clamp(msg->linear.x, -max_linear_speed_, max_linear_speed_);
    current_cmd_.angular.z = std::clamp(msg->angular.z, -max_angular_speed_, max_angular_speed_);
    last_cmd_time_ = this->now();
  }

  void update()
  {
    const rclcpp::Time current_time = this->now();
    const double dt = (current_time - last_update_time_).seconds();
    if (dt <= 0.0) {
      return;
    }
    last_update_time_ = current_time;

    geometry_msgs::msg::Twist active_cmd = current_cmd_;
    if ((current_time - last_cmd_time_).seconds() > cmd_timeout_) {
      active_cmd = geometry_msgs::msg::Twist();
    }

    x_ += active_cmd.linear.x * std::cos(yaw_) * dt;
    y_ += active_cmd.linear.x * std::sin(yaw_) * dt;
    yaw_ = normalizeYaw(yaw_ + active_cmd.angular.z * dt);

    publishOdometry(current_time, active_cmd);
  }

  void publishOdometry(const rclcpp::Time & stamp, const geometry_msgs::msg::Twist & twist)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = yawToQuaternion(yaw_);
    odom.twist.twist = twist;
    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = yawToQuaternion(yaw_);
    tf_broadcaster_->sendTransform(transform);
  }

  std::string odom_frame_;
  std::string base_frame_;
  std::string cmd_vel_topic_;
  std::string odom_topic_;

  double publish_rate_ = 50.0;
  double cmd_timeout_ = 0.25;
  double max_linear_speed_ = 0.8;
  double max_angular_speed_ = 2.0;
  double initial_x_ = 0.0;
  double initial_y_ = 0.0;
  double initial_yaw_ = 0.0;

  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;

  geometry_msgs::msg::Twist current_cmd_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Time last_update_time_;
  rclcpp::Time last_cmd_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeBaseDriverNode>());
  rclcpp::shutdown();
  return 0;
}
