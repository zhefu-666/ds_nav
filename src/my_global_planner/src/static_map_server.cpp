#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

namespace
{

struct MapYaml
{
  std::string image;
  double resolution = 0.05;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double origin_yaw = 0.0;
  int negate = 0;
  double occupied_thresh = 0.65;
  double free_thresh = 0.196;
  std::string mode = "trinary";
};

MapYaml loadYaml(const std::string & path)
{
  YAML::Node root = YAML::LoadFile(path);
  MapYaml y;
  y.image = root["image"].as<std::string>();
  y.resolution = root["resolution"].as<double>();
  const auto origin = root["origin"];
  y.origin_x = origin[0].as<double>();
  y.origin_y = origin[1].as<double>();
  if (origin.size() >= 3) {
    y.origin_yaw = origin[2].as<double>();
  }
  y.negate = root["negate"] ? root["negate"].as<int>() : 0;
  y.occupied_thresh =
    root["occupied_thresh"] ? root["occupied_thresh"].as<double>() : 0.65;
  y.free_thresh =
    root["free_thresh"] ? root["free_thresh"].as<double>() : 0.196;
  if (root["mode"]) {
    y.mode = root["mode"].as<std::string>();
  }
  return y;
}

// Very small PGM (P5) loader; enough for what map_server writes and what our
// build_static_map.py writes.
bool loadPgmP5(
  const std::string & path,
  std::vector<uint8_t> & pixels,
  int & width,
  int & height)
{
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    return false;
  }

  auto skip_ws_and_comments = [&](std::istream & s) {
      while (true) {
        const int c = s.peek();
        if (c == EOF) {
          return;
        }
        if (std::isspace(static_cast<unsigned char>(c))) {
          s.get();
          continue;
        }
        if (c == '#') {
          std::string ignored;
          std::getline(s, ignored);
          continue;
        }
        return;
      }
    };

  std::string magic;
  in >> magic;
  if (magic != "P5") {
    return false;
  }

  int maxval = 0;
  skip_ws_and_comments(in);
  in >> width;
  skip_ws_and_comments(in);
  in >> height;
  skip_ws_and_comments(in);
  in >> maxval;
  if (!in || width <= 0 || height <= 0 || maxval <= 0 || maxval > 255) {
    return false;
  }

  // consume the single whitespace byte after maxval before binary data starts
  in.get();

  pixels.resize(static_cast<std::size_t>(width) * static_cast<std::size_t>(height));
  in.read(reinterpret_cast<char *>(pixels.data()),
    static_cast<std::streamsize>(pixels.size()));
  return static_cast<std::size_t>(in.gcount()) == pixels.size();
}

}  // namespace

class StaticMapServer : public rclcpp::Node
{
public:
  StaticMapServer()
  : Node("static_map_server")
  {
    yaml_path_ = this->declare_parameter<std::string>("yaml_path", "");
    map_topic_ = this->declare_parameter<std::string>("map_topic", "/map");
    frame_id_ = this->declare_parameter<std::string>("frame_id", "map");
    republish_period_ = this->declare_parameter<double>("republish_period", 0.0);

    if (yaml_path_.empty()) {
      RCLCPP_FATAL(this->get_logger(), "yaml_path parameter is empty.");
      throw std::runtime_error("yaml_path parameter is empty");
    }

    if (!loadMap()) {
      RCLCPP_FATAL(this->get_logger(),
        "Failed to load map from %s", yaml_path_.c_str());
      throw std::runtime_error("Failed to load map");
    }

    pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(1).reliable().transient_local());
    map_.header.frame_id = frame_id_;
    map_.header.stamp = this->now();
    pub_->publish(map_);

    if (republish_period_ > 0.0) {
      const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(republish_period_));
      timer_ = this->create_wall_timer(period, [this]() {
          map_.header.stamp = this->now();
          pub_->publish(map_);
        });
    }

    RCLCPP_INFO(this->get_logger(),
      "Published %dx%d map @ %.3f m on %s (origin=%.3f, %.3f)",
      map_.info.width, map_.info.height, map_.info.resolution,
      map_topic_.c_str(),
      map_.info.origin.position.x, map_.info.origin.position.y);
  }

private:
  bool loadMap()
  {
    MapYaml y;
    try {
      y = loadYaml(yaml_path_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "YAML parse error: %s", e.what());
      return false;
    }

    std::filesystem::path image_path(y.image);
    if (image_path.is_relative()) {
      image_path = std::filesystem::path(yaml_path_).parent_path() / image_path;
    }

    std::vector<uint8_t> pixels;
    int width = 0;
    int height = 0;
    if (!loadPgmP5(image_path.string(), pixels, width, height)) {
      RCLCPP_ERROR(this->get_logger(),
        "Failed to load PGM image %s", image_path.string().c_str());
      return false;
    }

    map_.info.resolution = y.resolution;
    map_.info.width = static_cast<uint32_t>(width);
    map_.info.height = static_cast<uint32_t>(height);
    map_.info.origin.position.x = y.origin_x;
    map_.info.origin.position.y = y.origin_y;
    map_.info.origin.position.z = 0.0;
    const double half_yaw = y.origin_yaw * 0.5;
    map_.info.origin.orientation.z = std::sin(half_yaw);
    map_.info.origin.orientation.w = std::cos(half_yaw);
    map_.data.assign(static_cast<std::size_t>(width) * static_cast<std::size_t>(height), -1);

    // Convert pixel -> occupancy per REP-117:
    //   probability = (255 - pixel) / 255 if negate == 0
    //   probability = pixel / 255 otherwise
    //   occupied when probability > occupied_thresh
    //   free when probability < free_thresh
    //   unknown otherwise (trinary mode)
    // PGM row 0 is the top row; OccupancyGrid.data row 0 is the bottom row.
    for (int r = 0; r < height; ++r) {
      const int flipped_r = height - 1 - r;
      for (int c = 0; c < width; ++c) {
        const uint8_t px = pixels[static_cast<std::size_t>(r) * width + c];
        double prob = (y.negate == 0) ? (255.0 - px) / 255.0 : px / 255.0;
        int8_t cell;
        if (prob > y.occupied_thresh) {
          cell = 100;
        } else if (prob < y.free_thresh) {
          cell = 0;
        } else {
          cell = (y.mode == "scale") ? static_cast<int8_t>(prob * 100.0) : -1;
        }
        map_.data[static_cast<std::size_t>(flipped_r) * width + c] = cell;
      }
    }
    return true;
  }

  std::string yaml_path_;
  std::string map_topic_;
  std::string frame_id_;
  double republish_period_ = 0.0;

  nav_msgs::msg::OccupancyGrid map_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<StaticMapServer>());
  } catch (const std::exception & e) {
    std::fprintf(stderr, "static_map_server fatal: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
