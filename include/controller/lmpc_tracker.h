#pragma once

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include "controller/lmpc.hpp"
#include "st_opt/backend_tool/traj_opt.hpp"
#include "st_opt/SplineTrajectory.hpp"

namespace controller {

class LmpcTracker {
public:
  /**
   * @param dt        MPC 预测步长 (默认 0.1 s)
   * @param window    预测窗口长度 (默认 30)
   */
  LmpcTracker(double dt = 0.1, int window = 30);

  /**
   * @brief 设置参考轨迹 (从 TrajectoryOptimizer 获取)
   * @param traj_opt  已完成优化的轨迹优化器对象
   */
  void setReferenceTrajectory(const TrajOpt::TrajectoryOptimizer& traj_opt);

  /**
   * @brief 直接设置参考轨迹 (PPoly2D)
   */
  void setReferenceTrajectory(const SplineTrajectory::PPoly2D& traj);

  /**
   * @brief 主更新函数：根据当前状态计算控制指令
   * @param current_xytheta  当前位姿 (x, y, yaw) – yaw 暂未使用
   * @return                 最优速度指令 (vx, vy)
   */
  Eigen::Vector2d update(const Eigen::Vector3d& current_xytheta);

  /// 是否已加载有效轨迹
  bool isReady() const { return traj_initialized_; }

private:
  Mpc mpc_;                                   // MPC 求解器
  SplineTrajectory::PPoly2D optimized_traj_;  // 当前要跟踪的轨迹
  bool traj_initialized_ = false;
  rclcpp::Time start_time_;                   // 轨迹起始时间 (ROS 时间)
  double dt_;
  int window_;
};

} // namespace controller