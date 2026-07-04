#include "ros2/ros2_2d.hpp"
#include "config.hpp"
#include "controller/dwa_planner.h"
#include "utils/plotter.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <cstddef>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <memory>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <optional>
#include <spdlog/spdlog.h>
#include <string>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

namespace planner {
GlobalPlanner2d::GlobalPlanner2d(rclcpp::Node::SharedPtr nh_)
    : config(nh_), nh(nh_), mapInitialized(false), visualizer(nh_),
      traj_tracker_(0.1, 10),
      lmpc_tracker_(std::make_unique<controller::LmpcTracker>(0.1, 10)),
      plotter_() {
  grid_map_ = std::make_shared<grid_map::GridMap>();
  // lmpc_tracker_ = std::make_unique<controller::LmpcTracker>(0.1, 30);
  //  TODO:滑动更新，全局，局部
  const double map_size = 43.0;
  //============
  const double resolution = 0.1;
  grid_map_->init(map_size, map_size, resolution);

  mapSub = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
      config.mapTopic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        GlobalPlanner2d::mapCallBack(msg);
      });
  OdomSub = nh->create_subscription<nav_msgs::msg::Odometry>(
      config.odomTopic, rclcpp::QoS(10),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        GlobalPlanner2d::odomCallBack(msg);
      });
  targetSub = nh->create_subscription<geometry_msgs::msg::PoseStamped>(
      config.targetTopic, rclcpp::QoS(10),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        targetCallBack(msg);
      });
  cmd_vel_pub_ =
      nh->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_chassis", 10);
  global_path_pub_ =
      nh->create_publisher<nav_msgs::msg::Path>("global_path", 10);
  // 每 0.1 秒执行一次
  planner_timer_ =
      nh->create_wall_timer(std::chrono::milliseconds(100), // 时间间隔参数
                            [&]() { planner_callback(); }   // 回调函数
      );
  controller_timer_ =
      nh->create_wall_timer(std::chrono::milliseconds(100), // 时间间隔参数
                            [&]() { controller_callback(); } // 回调函数
      );
}
void GlobalPlanner2d::controller_callback() {
  if (current_pose.has_value() && goal_pose.has_value()) {
    if ((current_pose->x() == goal_pose->x()) &&
        (current_pose->y() == goal_pose->y())) {
      spdlog::info("success to goal");
    }
  }
  if (!current_XYTheta.has_value()) {
    spdlog::warn("current_XYTheta no value");
  }
  std::string tracker_model = "de";
  if (current_XYTheta.has_value()) {
    if (tracker_model == "de") {
      if (traj_tracker_.is_full_trajectory()) {
        auto v_w = traj_tracker_.update(current_XYTheta.value());
        geometry_msgs::msg::Twist cmd_pub_data;
        cmd_pub_data.angular.z = v_w.z();
        cmd_pub_data.linear.x = v_w.x();
        cmd_pub_data.linear.y = v_w.y();

        double total_linear_vel =
            std::sqrt(v_w.x() * v_w.x() + v_w.y() * v_w.y());
        // spdlog::info("v_x:{:3},v_y:{:3},w:{:3}", v_w.x(), v_w.y(),v_w.z());
        // spdlog::info("total_linear_vel:{}", total_linear_vel);

        this->cmd_vel_pub_->publish(cmd_pub_data);
      }
    } else if(tracker_model=="omni"){
      // 到达判断
      if (goal_pose.has_value() &&
          (current_XYTheta->head<2>() - goal_pose->head<2>()).norm() < 0.1) {
        // 停止
        geometry_msgs::msg::Twist stop;
        cmd_vel_pub_->publish(stop);
        spdlog::info("mpc 停止");
        return;
      }

      if (lmpc_tracker_->isReady()) {
        Eigen::Vector2d cmd = lmpc_tracker_->update(*current_XYTheta);
        geometry_msgs::msg::Twist twist;
        twist.linear.x = cmd.x();
        twist.linear.y = cmd.y();
        spdlog::info("MPC cmd: vx={:.3f}, vy={:.3f}", cmd.x(), cmd.y());
        twist.angular.z =0.0;
        // 如果有需要也可以计算 angular.z
        //spdlog::info("lmpc_tracker_ update pub success");
        cmd_vel_pub_->publish(twist);
      } else {
        //spdlog::info("mpc_tracker_->isReady() not");
      }
    }
  }
}
void GlobalPlanner2d::planner_callback() { GlobalPlanner2d::plan_omni(); }
void GlobalPlanner2d::plan_omni() {
  if (this->goal_pose == std::nullopt) {
    // spdlog::info("no goal");
  }
  if (this->current_pose == std::nullopt) {
    // spdlog::info("no current_pose");
  }
  if (this->goal_pose != std::nullopt && this->current_pose != std::nullopt) {
    auto result = fsm_.plan(goal_pose.value(), current_pose.value(), grid_map_);
    if (result) { // 或者 result.has_value()
      // 解包出 pair，然后使用结构化绑定分解 pair
      auto [astar_path, opt] = result.value(); // result.value() 返回 pair
      visualizer.PubGlobalPath(astar_path);
      std::vector<Eigen::Vector2d> opt_path = opt.sampleTrajectory(0.1);
      visualizer.PubOptPath(opt_path);
      // spdlog::info("path points:{}", opt_path.size());

      //==================================================//
      //======================MPC========================//
      //==================================================//
      traj_tracker_.setReferenceTrajectory(opt_path);
      lmpc_tracker_->setReferenceTrajectory(opt);
    } else {
      // 处理错误：result.error() 返回 path_error
      auto err = result.error();
      // 根据 err 做错误处理
    }
  }
}

void GlobalPlanner2d::odomCallBack(
    const nav_msgs::msg::Odometry::SharedPtr &msg) {
  if (!current_pose.has_value()) {
    current_pose = Eigen::Vector3d::Zero();
  }
  if (!current_XYTheta.has_value()) {
    current_XYTheta = Eigen::Vector3d::Zero();
  }
  const auto &quat = msg->pose.pose.orientation;
  tf2::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

  current_pose->x() = msg->pose.pose.position.x;
  current_pose->y() = msg->pose.pose.position.y;
  current_pose->z() = msg->pose.pose.position.z;
  current_XYTheta->x() = msg->pose.pose.position.x;
  current_XYTheta->y() = msg->pose.pose.position.y;
  current_XYTheta->z() = yaw;
  // spdlog::info("callback current_XYTheta: x:{},y:{},yaw:{}",
  //     current_XYTheta->x(),current_XYTheta->y(),current_XYTheta->z());
  // spdlog::info("callback
  // current_plose_:x:{},y:{}",current_pose->x(),current_pose->y());
  //  spdlog::info("Received current pose with position ({},{},{})",
  //               msg->pose.pose.position.x, msg->pose.pose.position.y,
  //               msg->pose.pose.position.z);
};

void GlobalPlanner2d::mapCallBack(
    const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
  // RCLCPP_INFO(nh->get_logger(), "Received map point cloud with %zu points",
  //             msg->data.size() / msg->point_step);
  auto voxel_num = grid_map_->getVoxelNum();
  grid_map::RowMatrixXi occupancy =
      grid_map::RowMatrixXi::Zero(voxel_num.x(), voxel_num.y());

  std::vector<Eigen::Vector3d> pc;
  size_t cur = 0;
  const size_t total = msg->data.size() / msg->point_step;
  float *fdata = (float *)(&msg->data[0]);
  for (size_t i = 0; i < total; i++) {
    cur = msg->point_step / sizeof(float) * i;
    if (std::isnan(fdata[cur + 0]) || std::isinf(fdata[cur + 0]) ||
        std::isnan(fdata[cur + 1]) || std::isinf(fdata[cur + 1]) ||
        std::isnan(fdata[cur + 2]) || std::isinf(fdata[cur + 2])) {
      spdlog::warn("map continue");
      continue;
    }
    //====================================//
    Eigen::Vector2d obstacle_world =
        Eigen::Vector2d(fdata[cur + 0], fdata[cur + 1]);
    Eigen::Vector2i grid_index;
    grid_map_->posToIndex(obstacle_world, grid_index);

    if (grid_index.x() >= 0 && grid_index.x() < voxel_num.x() &&
        grid_index.y() >= 0 && grid_index.y() < voxel_num.y()) {
      occupancy(grid_index.x(), grid_index.y()) = 1; // 标记为障碍物
      pc.emplace_back(Eigen::Vector3d(fdata[cur + 0], fdata[cur + 1], 0));
    }
    //===============================================//
  }
  grid_map_->setMap(occupancy);
  mapInitialized = true;

  visualizer.visualizeMap(pc);
}
void GlobalPlanner2d::targetCallBack(
    const geometry_msgs::msg::PoseStamped::SharedPtr &msg) {
  spdlog::info("Received target pose with position ({:2f},{:2f},{:2f})",
               msg->pose.position.x, msg->pose.position.y,
               msg->pose.position.z);
  if (mapInitialized) {

    const double zGoal =
        config.mapBound[4] + config.dilateRadius +
        fabs(msg->pose.orientation.z) *
            (config.mapBound[5] - config.mapBound[4] - 2 * config.dilateRadius);
    const Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, 0.0);

    goal_pose = goal;

    spdlog::info("get goal");

  } else {
    spdlog::warn("map no init");
  }
  return;
}

} // namespace planner