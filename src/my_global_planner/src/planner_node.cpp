#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#include "my_global_planner/astar.hpp"

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

}  // namespace

class MyGlobalPlanner : public rclcpp::Node
{
public:
  MyGlobalPlanner()
  : Node("my_global_planner"),
    planner_(planner_options_)
  {
    map_topic_ = this->declare_parameter<std::string>("map_topic", "/map");
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/odom");
    goal_topic_ = this->declare_parameter<std::string>("goal_topic", "/goal_pose");
    path_topic_ = this->declare_parameter<std::string>("path_topic", "/planned_path");
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");

    planning_frequency_ = this->declare_parameter<double>("planning_frequency", 2.0);
    if (planning_frequency_ <= 0.0) {
      planning_frequency_ = 2.0;
    }

    planner_options_.obstacle_threshold = this->declare_parameter<int>("obstacle_threshold", 50);
    planner_options_.allow_unknown = this->declare_parameter<bool>("allow_unknown", false);
    planner_options_.use_diagonal = this->declare_parameter<bool>("use_diagonal", true);
    planner_ = my_global_planner::AStarPlanner(planner_options_);

    goal_pose_.header.frame_id = map_frame_;
    goal_pose_.header.stamp = this->now();
    goal_pose_.pose.position.x = this->declare_parameter<double>("goal_x", 2.8);
    goal_pose_.pose.position.y = this->declare_parameter<double>("goal_y", 2.8);
    goal_pose_.pose.position.z = 0.0;
    goal_pose_.pose.orientation = yawToQuaternion(
      this->declare_parameter<double>("goal_yaw", 0.0));

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_,
      rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&MyGlobalPlanner::mapCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      10,
      std::bind(&MyGlobalPlanner::odomCallback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_,
      10,
      std::bind(&MyGlobalPlanner::goalCallback, this, std::placeholders::_1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      path_topic_,
      rclcpp::QoS(1).reliable());

    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / planning_frequency_));
    plan_timer_ = this->create_wall_timer(
      period,
      std::bind(&MyGlobalPlanner::planAndPublish, this));

    RCLCPP_INFO(
      this->get_logger(),
      "my_global_planner started. map=%s odom=%s goal=%s path=%s",
      map_topic_.c_str(),
      odom_topic_.c_str(),
      goal_topic_.c_str(),
      path_topic_.c_str());
  }

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_ = msg;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_ = msg;
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    goal_pose_ = *msg;
    goal_pose_.header.frame_id = map_frame_;
    goal_pose_.header.stamp = this->now();
  }

  void planAndPublish()
  {
    if (!map_ || !odom_) {
      return;
    }

    geometry_msgs::msg::PoseStamped start;
    start.header.frame_id = map_frame_;
    start.header.stamp = this->now();
    start.pose = odom_->pose.pose;

    geometry_msgs::msg::PoseStamped goal = goal_pose_;
    goal.header.frame_id = map_frame_;
    goal.header.stamp = this->now();

    nav_msgs::msg::Path path;
    if (planner_.makePlan(*map_, start, goal, path)) {
      path.header.frame_id = map_frame_;
      path.header.stamp = this->now();
      path_pub_->publish(path);
      last_plan_error_.clear();
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000,
        "Published path with %zu poses.",
        path.poses.size());
      return;
    }

    const std::string error = planner_.lastError();
    if (error != last_plan_error_) {
      RCLCPP_WARN(this->get_logger(), "Planning failed: %s", error.c_str());
      last_plan_error_ = error;
    }
  }

  my_global_planner::AStarPlanner::Options planner_options_;
  my_global_planner::AStarPlanner planner_;

  std::string map_topic_;
  std::string odom_topic_;
  std::string goal_topic_;
  std::string path_topic_;
  std::string map_frame_;
  double planning_frequency_ = 2.0;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr plan_timer_;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  nav_msgs::msg::Odometry::SharedPtr odom_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  std::string last_plan_error_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyGlobalPlanner>());
  rclcpp::shutdown();
  return 0;
}
