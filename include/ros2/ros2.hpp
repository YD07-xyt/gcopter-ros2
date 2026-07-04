// #pragma once
// #include "../config.hpp"
// #include "controller/differential_mpc.h"
// #include "gcopter/firi.hpp"
// #include "gcopter/flatness.hpp"
// #include "gcopter/gcopter.hpp"

// #include "gcopter/sfc_gen.hpp"
// #include "gcopter/trajectory.hpp"
// #include "gcopter/voxel_map.hpp"
// #include "map/grid_map.h"
// #include "misc/visualizer.hpp"
// #include <Eigen/Core>
// #include <Eigen/src/Core/Matrix.h>
// #include <cstddef>
// #include <geometry_msgs/msg/detail/twist__struct.hpp>
// #include <geometry_msgs/msg/point.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include <nav_msgs/msg/detail/path__struct.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <nav_msgs/msg/path.hpp>
// #include <optional>
// #include <rclcpp/publisher.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp/subscription.hpp>
// #include <rclcpp/time.hpp>
// #include <rclcpp/timer.hpp>
// #include <sensor_msgs/msg/point_cloud2.hpp>

// #include <chrono>
// #include <cmath>
// #include <iostream>
// #include <memory>
// #include <random>
// #include <spdlog/spdlog.h>
// #include <string>
// #include <vector>
// namespace planner {

// class GlobalPlanner {
// private:
//   Config config;
//   rclcpp::Node::SharedPtr nh;
//   // planner::KinodynamicAstar astar_planner_;
//   rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mapSub;
//   rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr targetSub;
//   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr OdomSub;
//   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
//   rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
//   rclcpp::TimerBase::SharedPtr planner_timer_;
//   std::optional<Eigen::Vector3d> current_pose = std::nullopt;
//   std::optional<Eigen::Vector3d> goal_pose = std::nullopt;
//   bool mapInitialized;
//   voxel_map::VoxelMap voxelMap;
//   Visualizer visualizer;
//   std::shared_ptr<Mpc> mpc_;
//   Trajectory<5> traj;
//   double trajStamp;


// public:
//   GlobalPlanner(rclcpp::Node::SharedPtr nh_);
//   void mapCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);
//   void odomCallBack(const nav_msgs::msg::Odometry::SharedPtr &msg);
//   void targetCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr &msg);
//   void planner_callback();
//   void plan_omni();
//   Eigen::Vector2d getLocalGoal(double x, double y,
//     const std::vector<Eigen::Vector2d>& path,double lookahead); 
//   void process();
//   void PubPath(std::vector<Eigen::Vector3d> &path) {
//     nav_msgs::msg::Path nav_path;
//     nav_path.header.frame_id = "world";
//     nav_path.header.stamp =
//         rclcpp::Clock().now(); // 或使用 node->get_clock()->now()

//     for (const auto &pt : path) {
//       geometry_msgs::msg::PoseStamped pose_stamped;
//       pose_stamped.header = nav_path.header; // 使用相同的帧ID和时间戳
//       pose_stamped.pose.position.x = pt.x();
//       pose_stamped.pose.position.y = pt.y();
//       pose_stamped.pose.position.z = pt.z(); // 如果只有2D，设为0
//       pose_stamped.pose.orientation.x = 0.0;
//       pose_stamped.pose.orientation.y = 0.0;
//       pose_stamped.pose.orientation.z = 0.0;
//       pose_stamped.pose.orientation.w = 1.0; // 单位四元数（无旋转）

//       nav_path.poses.push_back(pose_stamped);
//     }

//     global_path_pub_->publish(nav_path);
//   };
//   std::vector<Eigen::Vector3d> replan(std::vector<Eigen::Vector3d> route) {
//     // 检查初始路径是否有效
//     while (!(route.back().x() == goal_pose->x() &&
//              route.back().y() == goal_pose->y())) {
//       route.clear();
//       sfc_gen::planPath<voxel_map::VoxelMap>(
//           current_pose.value(), goal_pose.value(), voxelMap.getOrigin(),
//           voxelMap.getCorner(), &voxelMap, 0.01, route);
//       // // 可添加最大重规划次数防止无限循环
                            
//     }
//     spdlog::info("replan success");
//     return route;
//   }

// };
// } // namespace planner