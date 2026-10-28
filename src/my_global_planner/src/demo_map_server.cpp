#include <algorithm>
#include <cstdint>
#include <cmath>
#include <memory>
#include <string>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

class DemoMapServer : public rclcpp::Node
{
public:
  DemoMapServer()
  : Node("demo_map_server")
  {
    map_topic_ = this->declare_parameter<std::string>("map_topic", "/map");
    frame_id_ = this->declare_parameter<std::string>("frame_id", "map");
    resolution_ = this->declare_parameter<double>("resolution", 0.05);
    width_ = this->declare_parameter<int>("width", 160);
    height_ = this->declare_parameter<int>("height", 160);
    origin_x_ = this->declare_parameter<double>("origin_x", -10.0);
    origin_y_ = this->declare_parameter<double>("origin_y", -10.0);
    obstacle_value_ = this->declare_parameter<int>("obstacle_value", 100);
    free_value_ = this->declare_parameter<int>("free_value", 0);
    wall_thickness_cells_ = this->declare_parameter<int>("wall_thickness_cells", 4);

    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      map_topic_,
      rclcpp::QoS(1).reliable().transient_local());

    publishMap();
    RCLCPP_INFO(
      this->get_logger(),
      "Published demo map on %s: %dx%d @ %.3fm, origin=(%.2f, %.2f)",
      map_topic_.c_str(),
      width_,
      height_,
      resolution_,
      origin_x_,
      origin_y_);
  }

private:
  void publishMap()
  {
    nav_msgs::msg::OccupancyGrid map;
    map.header.stamp = this->now();
    map.header.frame_id = frame_id_;
    map.info.resolution = resolution_;
    map.info.width = static_cast<uint32_t>(width_);
    map.info.height = static_cast<uint32_t>(height_);
    map.info.origin.position.x = origin_x_;
    map.info.origin.position.y = origin_y_;
    map.info.origin.position.z = 0.0;
    map.info.origin.orientation.w = 1.0;
    map.data.assign(
      static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_),
      static_cast<int8_t>(free_value_));

    auto fillRectCells = [&](int min_x, int min_y, int max_x, int max_y) {
        min_x = std::clamp(min_x, 0, width_ - 1);
        min_y = std::clamp(min_y, 0, height_ - 1);
        max_x = std::clamp(max_x, 0, width_ - 1);
        max_y = std::clamp(max_y, 0, height_ - 1);

        for (int y = min_y; y <= max_y; ++y) {
          for (int x = min_x; x <= max_x; ++x) {
            const std::size_t index =
              static_cast<std::size_t>(y) * static_cast<std::size_t>(width_) +
              static_cast<std::size_t>(x);
            map.data[index] = static_cast<int8_t>(obstacle_value_);
          }
        }
      };

    auto fillRectWorld = [&](double min_x, double min_y, double max_x, double max_y) {
        const int min_cell_x = worldToCellX(min_x);
        const int min_cell_y = worldToCellY(min_y);
        const int max_cell_x = worldToCellX(max_x);
        const int max_cell_y = worldToCellY(max_y);
        fillRectCells(min_cell_x, min_cell_y, max_cell_x, max_cell_y);
      };

    fillRectCells(0, 0, width_ - 1, wall_thickness_cells_ - 1);
    fillRectCells(0, height_ - wall_thickness_cells_, width_ - 1, height_ - 1);
    fillRectCells(0, 0, wall_thickness_cells_ - 1, height_ - 1);
    fillRectCells(width_ - wall_thickness_cells_, 0, width_ - 1, height_ - 1);

    fillRectWorld(-1.325, 0.775, -0.475, 1.825);
    fillRectWorld(0.775, -1.825, 1.825, -0.775);

    map_pub_->publish(map);
  }

  int worldToCellX(double world_x) const
  {
    return static_cast<int>(std::floor((world_x - origin_x_) / resolution_));
  }

  int worldToCellY(double world_y) const
  {
    return static_cast<int>(std::floor((world_y - origin_y_) / resolution_));
  }

  std::string frame_id_;
  std::string map_topic_;
  double resolution_ = 0.05;
  int width_ = 160;
  int height_ = 160;
  double origin_x_ = -4.0;
  double origin_y_ = -4.0;
  int obstacle_value_ = 100;
  int free_value_ = 0;
  int wall_thickness_cells_ = 4;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DemoMapServer>());
  rclcpp::shutdown();
  return 0;
}
