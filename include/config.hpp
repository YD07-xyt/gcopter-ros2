#pragma once

#include <cmath>
#include <string>
#include <vector>
#include<rclcpp/rclcpp.hpp>
namespace planner {

struct DwaConfig{
  double max_speed;
  double min_speed;
  double max_yawrate; 
  double max_accel;
  double max_dyawrate;
  double v_resolution;
  double yawrate_resolution;
  int velocity_samples;
  int yawrate_samples;
  double dt;
  double predict_time;
  double weights_to_goal;
  double weights_obstacle; 
  double weights_speed;
  double robot_radius;
  double range;
};


struct Config {
  std::string mapTopic  = "/terrain_map_ext";
  std::string targetTopic = "/goal_pose";
  std::string odomTopic = "/fake_odom";
  double dilateRadius = 0.2;
  double voxelWidth = 0.1;
  //
  double traj_duration=20;
  std::vector<double> mpc_weights={10,10,1,1,1};
  //
  std::vector<double> mapBound = {-20.0, 20.0, -20.0, 20.0, -15.0, 15.0};
  double timeoutRRT = 0.5;
  double maxVelMag = 2.0;
  double maxBdrMag = 1.0;
  double maxTiltAngle = M_PI / 4;
  double minThrust = 0.0;
  double maxThrust = 20.0;
  double vehicleMass = 1.0;
  double gravAcc = 9.81;
  double horizDrag = 0.1;
  double vertDrag = 0.1;
  double parasDrag = 0.1;
  double speedEps = 0.01;
  double weightT = 1.0;
  std::vector<double> chiVec = {1.0, 1.0, 1.0};
  double smoothingEps = 0.01;
  int integralIntervs = 10;
  double relCostTol = 0.01;

  Config(const rclcpp::Node::SharedPtr &node) {
    // 声明参数（带默认值）
    node->declare_parameter<std::string>("map_topic", mapTopic);
    node->declare_parameter<std::string>("target_topic", targetTopic);
    node->declare_parameter<std::string>("odom_topic", odomTopic);
    node->declare_parameter<double>("dilate_radius", dilateRadius);
    node->declare_parameter<double>("voxel_width", voxelWidth);
    node->declare_parameter<std::vector<double>>("map_bound", mapBound);
    node->declare_parameter<double>("timeout_rrt", timeoutRRT);
    node->declare_parameter<double>("max_vel_mag", maxVelMag);
    node->declare_parameter<double>("max_bdr_mag", maxBdrMag);
    node->declare_parameter<double>("max_tilt_angle", maxTiltAngle);
    node->declare_parameter<double>("min_thrust", minThrust);
    node->declare_parameter<double>("max_thrust", maxThrust);
    node->declare_parameter<double>("vehicle_mass", vehicleMass);
    node->declare_parameter<double>("grav_acc", gravAcc);
    node->declare_parameter<double>("horiz_drag", horizDrag);
    node->declare_parameter<double>("vert_drag", vertDrag);
    node->declare_parameter<double>("paras_drag", parasDrag);
    node->declare_parameter<double>("speed_eps", speedEps);
    node->declare_parameter<double>("weight_t", weightT);
    node->declare_parameter<std::vector<double>>("chi_vec", chiVec);
    node->declare_parameter<double>("smoothing_eps", smoothingEps);
    node->declare_parameter<int>("integral_intervs", integralIntervs);
    node->declare_parameter<double>("rel_cost_tol", relCostTol);

    // 读取参数（将实际值赋给成员变量）
    node->get_parameter("map_topic", mapTopic);
    node->get_parameter("target_topic", targetTopic);
    node->get_parameter("odom_topic", odomTopic);
    node->get_parameter("dilate_radius", dilateRadius);
    node->get_parameter("voxel_width", voxelWidth);
    node->get_parameter("map_bound", mapBound);
    node->get_parameter("timeout_rrt", timeoutRRT);
    node->get_parameter("max_vel_mag", maxVelMag);
    node->get_parameter("max_bdr_mag", maxBdrMag);
    node->get_parameter("max_tilt_angle", maxTiltAngle);
    node->get_parameter("min_thrust", minThrust);
    node->get_parameter("max_thrust", maxThrust);
    node->get_parameter("vehicle_mass", vehicleMass);
    node->get_parameter("grav_acc", gravAcc);
    node->get_parameter("horiz_drag", horizDrag);
    node->get_parameter("vert_drag", vertDrag);
    node->get_parameter("paras_drag", parasDrag);
    node->get_parameter("speed_eps", speedEps);
    node->get_parameter("weight_t", weightT);
    node->get_parameter("chi_vec", chiVec);
    node->get_parameter("smoothing_eps", smoothingEps);
    node->get_parameter("integral_intervs", integralIntervs);
    node->get_parameter("rel_cost_tol", relCostTol);
  }
};
} // namespace planner