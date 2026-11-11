// Simulation entry for the official Nav2 PlannerServer. Keep its executor out
// of rmw_wait before Context shutdown invokes lifecycle/plugin cleanup.
#include <chrono>
#include <csignal>
#include <exception>
#include <memory>

#include "nav2_planner/planner_server.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {
volatile std::sig_atomic_t stop_requested = 0;
void requestStop(int) { stop_requested = 1; }
}  // namespace

int main(int argc, char ** argv)
{
  // A signal handler must not call ROS, log, lock, or release plugin resources.
  std::signal(SIGINT, requestStop);
  std::signal(SIGTERM, requestStop);
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  try {
    auto node = std::make_shared<nav2_planner::PlannerServer>(rclcpp::NodeOptions());
    {
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(node->get_node_base_interface());
      while (rclcpp::ok() && !stop_requested) {
        // Only bounds the idle wait; ready callbacks retain their own timers.
        executor.spin_once(std::chrono::milliseconds(50));
      }
    }  // Destroy the wait set before preshutdown frees callback groups.
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("simulation_planner_server"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
}
