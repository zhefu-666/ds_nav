#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

class MyGlobalPlanner : public rclcpp::Node
{
public:
  MyGlobalPlanner()
  : Node("my_global_planner")
  {
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map",
      10,
      std::bind(&MyGlobalPlanner::mapCallback, this, std::placeholders::_1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/planned_path",
      10);

    RCLCPP_INFO(this->get_logger(), "my_global_planner node started.");
  }

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_ = msg;

    RCLCPP_INFO(
      this->get_logger(),
      "Received map: width=%u, height=%u, resolution=%.3f",
      msg->info.width,
      msg->info.height,
      msg->info.resolution);
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyGlobalPlanner>());
  rclcpp::shutdown();
  return 0;
}
