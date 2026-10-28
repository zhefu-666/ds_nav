#ifndef MY_GLOBAL_PLANNER__ASTAR_HPP_
#define MY_GLOBAL_PLANNER__ASTAR_HPP_

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

namespace my_global_planner
{

class AStarPlanner
{
public:
  struct Options
  {
    Options() = default;

    int obstacle_threshold = 50;
    bool allow_unknown = false;
    bool use_diagonal = true;
  };

  AStarPlanner();
  explicit AStarPlanner(const Options & options);

  bool makePlan(
    const nav_msgs::msg::OccupancyGrid & map,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    nav_msgs::msg::Path & path) const;

  const std::string & lastError() const;

private:
  struct GridIndex
  {
    GridIndex() = default;
    GridIndex(int x_in, int y_in);

    int x = 0;
    int y = 0;
  };

  bool worldToGrid(
    const nav_msgs::msg::OccupancyGrid & map,
    double world_x,
    double world_y,
    GridIndex & grid) const;

  void gridToWorld(
    const nav_msgs::msg::OccupancyGrid & map,
    const GridIndex & grid,
    double & world_x,
    double & world_y) const;

  bool isFree(
    const nav_msgs::msg::OccupancyGrid & map,
    int x,
    int y) const;

  bool inBounds(
    const nav_msgs::msg::OccupancyGrid & map,
    int x,
    int y) const;

  int toIndex(
    const nav_msgs::msg::OccupancyGrid & map,
    int x,
    int y) const;

  double heuristic(
    const GridIndex & from,
    const GridIndex & to) const;

  double yawFromQuaternion(
    const geometry_msgs::msg::Quaternion & q) const;

  Options options_;
  mutable std::string last_error_;
};

}  // namespace my_global_planner

#endif  // MY_GLOBAL_PLANNER__ASTAR_HPP_
