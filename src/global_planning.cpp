#include "ros2/ros2.hpp"
#include "ros2/ros2_2d.hpp"

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  auto nh = std::make_shared<rclcpp::Node>("global_planning_node");

  //planner::GlobalPlanner global_planner(nh);
  planner::GlobalPlanner2d global_planner_2d(nh);
  rclcpp::WallRate rate(1000);
  while (rclcpp::ok()) {
    rclcpp::spin_some(nh);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}