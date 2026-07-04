#include "../include/controller/lmpc_tracker.h"
#include "controller/lmpc.hpp"

namespace controller {

LmpcTracker::LmpcTracker(double dt, int window)
    : mpc_(dt,window), dt_(dt),window_(window) {
  // Mpc 构造函数内部已经完成了所有矩阵的预计算
}

void LmpcTracker::setReferenceTrajectory(const TrajOpt::TrajectoryOptimizer& traj_opt) {
  optimized_traj_ = traj_opt.getOptimizedTrajectory();
  start_time_ = rclcpp::Clock().now();   // 记录当前时间为轨迹起始时刻
  traj_initialized_ = optimized_traj_.isInitialized();
}

void LmpcTracker::setReferenceTrajectory(const SplineTrajectory::PPoly2D& traj) {
  optimized_traj_ = traj;
  start_time_ = rclcpp::Clock().now();
  traj_initialized_ = traj.isInitialized();
}

Eigen::Vector2d LmpcTracker::update(const Eigen::Vector3d& current_xytheta) {
  if (!traj_initialized_) {
    return Eigen::Vector2d::Zero();
  }

  const double traj_duration = optimized_traj_.getDuration();
  rclcpp::Time now = rclcpp::Clock().now();
  double t_cur = (now - start_time_).seconds();

  // 构造参考位置矩阵 (2 x (window_+1))
  Eigen::Matrix2Xd x_ref(2, window_ + 1);
  for (int i = 0; i <= window_; ++i) {
    double t = t_cur + i * dt_;
    Eigen::Vector2d pos;
    if (t <= traj_duration) {
      pos = optimized_traj_.evaluate(t, 0);          // 获取位置
    } else {
      pos = optimized_traj_.evaluate(traj_duration, 0); // 超出时间停在终点
    }
    x_ref.col(i) = pos;
  }

  // 当前状态 (仅使用位置)
  Eigen::Vector2d x0(current_xytheta.x(), current_xytheta.y());
  mpc_.solveMpc(x0, x_ref);

  return mpc_.getControlCmd();   // 返回 (vx, vy)
}

} // namespace controller