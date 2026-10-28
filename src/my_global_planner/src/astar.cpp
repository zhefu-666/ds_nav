#include "my_global_planner/astar.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

namespace my_global_planner
{
namespace
{

constexpr double kStraightCost = 1.0;
constexpr double kDiagonalCost = 1.4142135623730951;
constexpr double kPi = 3.14159265358979323846;

struct QueueNode
{
  QueueNode() = default;
  QueueNode(int node_index, double node_f_cost)
  : index(node_index), f_cost(node_f_cost)
  {
  }

  int index = 0;
  double f_cost = 0.0;

  bool operator<(const QueueNode & other) const
  {
    return f_cost > other.f_cost;
  }
};

double normalizeYaw(double yaw)
{
  while (yaw > kPi) {
    yaw -= 2.0 * kPi;
  }
  while (yaw < -kPi) {
    yaw += 2.0 * kPi;
  }
  return yaw;
}

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  const double half_yaw = normalizeYaw(yaw) * 0.5;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(half_yaw);
  q.w = std::cos(half_yaw);
  return q;
}

}  // namespace

AStarPlanner::AStarPlanner()
: options_()
{
}

AStarPlanner::AStarPlanner(const Options & options)
: options_(options)
{
}

AStarPlanner::GridIndex::GridIndex(int x_in, int y_in)
: x(x_in), y(y_in)
{
}

bool AStarPlanner::makePlan(
  const nav_msgs::msg::OccupancyGrid & map,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  nav_msgs::msg::Path & path) const
{
  path.poses.clear();
  path.header = map.header;
  last_error_.clear();

  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  if (width <= 0 || height <= 0 || map.info.resolution <= 0.0) {
    last_error_ = "Invalid map dimensions or resolution.";
    return false;
  }

  const std::size_t expected_size =
    static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  if (map.data.size() < expected_size) {
    last_error_ = "OccupancyGrid data is smaller than width * height.";
    return false;
  }

  GridIndex start_grid;
  GridIndex goal_grid;
  if (!worldToGrid(map, start.pose.position.x, start.pose.position.y, start_grid)) {
    last_error_ = "Start pose is outside the map.";
    return false;
  }
  if (!worldToGrid(map, goal.pose.position.x, goal.pose.position.y, goal_grid)) {
    last_error_ = "Goal pose is outside the map.";
    return false;
  }
  if (!isFree(map, start_grid.x, start_grid.y)) {
    last_error_ = "Start pose is in an occupied or unknown cell.";
    return false;
  }
  if (!isFree(map, goal_grid.x, goal_grid.y)) {
    last_error_ = "Goal pose is in an occupied or unknown cell.";
    return false;
  }

  const int start_index = toIndex(map, start_grid.x, start_grid.y);
  const int goal_index = toIndex(map, goal_grid.x, goal_grid.y);

  if (start_index == goal_index) {
    path.poses.push_back(start);
    path.poses.back().header = path.header;
    path.poses.push_back(goal);
    path.poses.back().header = path.header;
    return true;
  }

  const double infinity = std::numeric_limits<double>::infinity();
  std::vector<double> g_cost(expected_size, infinity);
  std::vector<int> parent(expected_size, -1);
  std::vector<bool> closed(expected_size, false);
  std::priority_queue<QueueNode> open;

  g_cost[static_cast<std::size_t>(start_index)] = 0.0;
  open.push(QueueNode(start_index, heuristic(start_grid, goal_grid)));

  const std::vector<GridIndex> four_connected = {
    GridIndex{1, 0}, GridIndex{-1, 0}, GridIndex{0, 1}, GridIndex{0, -1}
  };
  const std::vector<GridIndex> eight_connected = {
    GridIndex{1, 0}, GridIndex{-1, 0}, GridIndex{0, 1}, GridIndex{0, -1},
    GridIndex{1, 1}, GridIndex{1, -1}, GridIndex{-1, 1}, GridIndex{-1, -1}
  };
  const std::vector<GridIndex> & motions =
    options_.use_diagonal ? eight_connected : four_connected;

  bool found = false;
  while (!open.empty()) {
    const QueueNode current = open.top();
    open.pop();

    if (closed[static_cast<std::size_t>(current.index)]) {
      continue;
    }
    closed[static_cast<std::size_t>(current.index)] = true;

    if (current.index == goal_index) {
      found = true;
      break;
    }

    const GridIndex current_grid{
      current.index % width,
      current.index / width
    };

    for (const auto & motion : motions) {
      const GridIndex next_grid{
        current_grid.x + motion.x,
        current_grid.y + motion.y
      };

      if (!isFree(map, next_grid.x, next_grid.y)) {
        continue;
      }

      if (motion.x != 0 && motion.y != 0) {
        const bool corner_blocked =
          !isFree(map, current_grid.x + motion.x, current_grid.y) ||
          !isFree(map, current_grid.x, current_grid.y + motion.y);
        if (corner_blocked) {
          continue;
        }
      }

      const int next_index = toIndex(map, next_grid.x, next_grid.y);
      if (closed[static_cast<std::size_t>(next_index)]) {
        continue;
      }

      const double step_cost =
        (motion.x != 0 && motion.y != 0) ? kDiagonalCost : kStraightCost;
      const double tentative_g =
        g_cost[static_cast<std::size_t>(current.index)] + step_cost;

      if (tentative_g < g_cost[static_cast<std::size_t>(next_index)]) {
        parent[static_cast<std::size_t>(next_index)] = current.index;
        g_cost[static_cast<std::size_t>(next_index)] = tentative_g;
        const double f_cost = tentative_g + heuristic(next_grid, goal_grid);
        open.push(QueueNode(next_index, f_cost));
      }
    }
  }

  if (!found) {
    last_error_ = "No path found.";
    return false;
  }

  std::vector<int> indices;
  for (int current = goal_index; current != -1; current = parent[static_cast<std::size_t>(current)]) {
    indices.push_back(current);
    if (current == start_index) {
      break;
    }
  }
  if (indices.empty() || indices.back() != start_index) {
    last_error_ = "Failed to reconstruct path.";
    return false;
  }
  std::reverse(indices.begin(), indices.end());

  path.poses.reserve(indices.size());
  for (std::size_t i = 0; i < indices.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;

    if (i == 0) {
      pose.pose = start.pose;
    } else if (i + 1 == indices.size()) {
      pose.pose = goal.pose;
    } else {
      const GridIndex grid{indices[i] % width, indices[i] / width};
      gridToWorld(map, grid, pose.pose.position.x, pose.pose.position.y);
      pose.pose.position.z = 0.0;
    }

    if (i + 1 < indices.size()) {
      double next_x = 0.0;
      double next_y = 0.0;
      if (i + 1 == indices.size() - 1) {
        next_x = goal.pose.position.x;
        next_y = goal.pose.position.y;
      } else {
        const GridIndex next_grid{indices[i + 1] % width, indices[i + 1] / width};
        gridToWorld(map, next_grid, next_x, next_y);
      }
      const double yaw = std::atan2(next_y - pose.pose.position.y, next_x - pose.pose.position.x);
      pose.pose.orientation = yawToQuaternion(yaw);
    } else {
      pose.pose.orientation = goal.pose.orientation;
    }

    path.poses.push_back(pose);
  }

  return true;
}

const std::string & AStarPlanner::lastError() const
{
  return last_error_;
}

bool AStarPlanner::worldToGrid(
  const nav_msgs::msg::OccupancyGrid & map,
  double world_x,
  double world_y,
  GridIndex & grid) const
{
  const double resolution = map.info.resolution;
  const double origin_x = map.info.origin.position.x;
  const double origin_y = map.info.origin.position.y;
  const double yaw = yawFromQuaternion(map.info.origin.orientation);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const double dx = world_x - origin_x;
  const double dy = world_y - origin_y;

  const double map_x = cos_yaw * dx + sin_yaw * dy;
  const double map_y = -sin_yaw * dx + cos_yaw * dy;

  grid.x = static_cast<int>(std::floor(map_x / resolution));
  grid.y = static_cast<int>(std::floor(map_y / resolution));

  return inBounds(map, grid.x, grid.y);
}

void AStarPlanner::gridToWorld(
  const nav_msgs::msg::OccupancyGrid & map,
  const GridIndex & grid,
  double & world_x,
  double & world_y) const
{
  const double resolution = map.info.resolution;
  const double origin_x = map.info.origin.position.x;
  const double origin_y = map.info.origin.position.y;
  const double yaw = yawFromQuaternion(map.info.origin.orientation);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  const double map_x = (static_cast<double>(grid.x) + 0.5) * resolution;
  const double map_y = (static_cast<double>(grid.y) + 0.5) * resolution;

  world_x = origin_x + cos_yaw * map_x - sin_yaw * map_y;
  world_y = origin_y + sin_yaw * map_x + cos_yaw * map_y;
}

bool AStarPlanner::isFree(
  const nav_msgs::msg::OccupancyGrid & map,
  int x,
  int y) const
{
  if (!inBounds(map, x, y)) {
    return false;
  }

  const int value = map.data[static_cast<std::size_t>(toIndex(map, x, y))];
  if (value < 0) {
    return options_.allow_unknown;
  }
  return value < options_.obstacle_threshold;
}

bool AStarPlanner::inBounds(
  const nav_msgs::msg::OccupancyGrid & map,
  int x,
  int y) const
{
  return x >= 0 && y >= 0 &&
    x < static_cast<int>(map.info.width) &&
    y < static_cast<int>(map.info.height);
}

int AStarPlanner::toIndex(
  const nav_msgs::msg::OccupancyGrid & map,
  int x,
  int y) const
{
  return y * static_cast<int>(map.info.width) + x;
}

double AStarPlanner::heuristic(
  const GridIndex & from,
  const GridIndex & to) const
{
  const int dx = std::abs(from.x - to.x);
  const int dy = std::abs(from.y - to.y);

  if (!options_.use_diagonal) {
    return static_cast<double>(dx + dy);
  }

  const int diagonal = std::min(dx, dy);
  const int straight = std::max(dx, dy) - diagonal;
  return kDiagonalCost * static_cast<double>(diagonal) +
    kStraightCost * static_cast<double>(straight);
}

double AStarPlanner::yawFromQuaternion(
  const geometry_msgs::msg::Quaternion & q) const
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace my_global_planner
