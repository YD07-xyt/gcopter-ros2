#pragma once

#include "st_opt/backend_tool/traj_opt.hpp"
#include "st_opt/frontend_tool/astar.hpp"
#include "st_opt/perception_tool/grid_map.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <optional>
#include <spdlog/spdlog.h>
#include <utils/expected.hpp>
namespace planner {
class FSM {
public:
  enum path_error {
    none,
    success,
    astar_path_empty,
    optimizer_failed,
    astar_path_collision,
  };
  using path = std::vector<Eigen::Vector2d>;
  using astar_opt_path = tl::expected<std::pair<path, TrajOpt::TrajectoryOptimizer>, path_error>;

  auto plan(const Eigen::Vector3d &goal_pose,
            const Eigen::Vector3d &current_pose,
            std::shared_ptr<grid_map::GridMap> grid_map) -> astar_opt_path {
    // 1. 目标比较（已修正）
    const Eigen::Vector3d deviation(0.5, 0.5, 0.0);
    if (checkPointEqual(old_goal_pose_, goal_pose, deviation)) {
      // 目标没变，但需要检查当前已规划路径是否仍然安全
      if (path_state_ == PathState::running ||
          path_state_ == PathState::successed) {
        // 重新检查上次的 astar_traj_ 是否发生碰撞
        if (!astar_traj_.optimized_path.empty() &&
            !checkCollision(astar_traj_)) {
          // 路径仍然安全，且目标未变，可以直接返回成功（避免重复规划）
          return tl::make_unexpected(path_error::success);
        } else {
          // 路径不再安全，重置状态，准备重规划
          spdlog::warn("Path became unsafe, triggering replanning...");
          path_state_ = PathState::idle;
          // 继续执行后续的重规划逻辑（不返回）
        }
      } else {
        // 其他状态（如 idle, failed）也应该继续尝试规划
      }
    }
    if (!checkPointEqual(old_goal_pose_, goal_pose, Eigen::Vector3d::Zero())) {
      spdlog::info("plan goal is change");
      old_goal_pose_ = goal_pose;
      path_state_ = PathState::idle;
    }

    // 2. 准备数据
    grid_map_ = grid_map;
    safe_threshold_ = 0.45;
    const Eigen::Vector2d start(current_pose.x(), current_pose.y());
    const Eigen::Vector2d goal(goal_pose.x(), goal_pose.y());

    // 3. 带重试的规划循环
    const int max_retries = 10;
    int retry_count = 0;
    path_planning::AStar astar(*grid_map, safe_threshold_);
    while (path_state_ != PathState::running && retry_count < max_retries) {

      auto astar_traj = astar.planWithPostProcessing(start, goal, 5000);
      astar_traj_ = astar_traj;
      old_goal_pose_ = goal_pose;

      spdlog::info("old_goal_pose_: ({:.2f}, {:.2f})", old_goal_pose_.x(),
                   old_goal_pose_.y());

      if (astar_traj.optimized_path.empty()) {
        spdlog::error("A* planning failed!");
        path_state_ = PathState::failed;
        return tl::make_unexpected(path_error::astar_path_empty);
      }

      // 碰撞检测
      if (checkCollision(astar_traj_)) {
        spdlog::warn("Collision detected, retrying... (attempt {}/{})",
                     retry_count + 1, max_retries);
        path_state_ = PathState::idle; // 重置为 idle 以继续循环
        retry_count++;
        continue; // 重新执行 A*
      } else {
        spdlog::info("collision is not failed");
      }

      // 无碰撞，进入优化阶段
      path_state_ = PathState::running;

      auto start_time = std::chrono::high_resolution_clock::now();
      TrajOpt::TrajectoryParams params;
      params.piece_len = astar_traj.total_length / astar_traj.total_time;
      params.total_time = astar_traj.total_time;
      params.total_len = astar_traj.total_length;

      TrajOpt::TrajectoryOptimizer optimizer(grid_map,
                                             astar_traj.optimized_path, params);
      if (!optimizer.plan()) {
        spdlog::error("Trajectory optimization failed!");
        path_state_ = PathState::failed;
        return tl::make_unexpected(path_error::optimizer_failed);
      }

      auto end_time = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
          end_time - start_time);
      auto metrics = optimizer.evaluateTrajectory();
      std::vector<Eigen::Vector2d> opt_path = optimizer.sampleTrajectory(0.1);
      return std::make_pair(astar_traj.optimized_path, optimizer);
    }

    // 循环结束仍未成功（状态不是 running 或重试耗尽）
    if (retry_count >= max_retries) {
      spdlog::error("Max retries reached, planning failed due to collisions");
      path_state_ = PathState::failed;
      return tl::make_unexpected(path_error::astar_path_empty);
    }

    return tl::make_unexpected(path_error::none);
  }
public:


private:
  auto checkCollision(path_planning::AStar::Trajectory astar_traj) -> bool {
    auto path = astar_traj.optimized_path;
    // Points outside map are considered collision-free
    for (auto pos : path) {
      if (!grid_map_->isInsideMap(pos)) {
        continue;
      }

      // Points inside map use safety distance check
      if (grid_map_->getDistance(pos) < safe_threshold_) {
        return true;
      };
    }
    return false;
  };
  auto checkPointEqual(const Eigen::Vector3d &pos1, const Eigen::Vector3d &pos2,
                       const Eigen::Vector3d &deviation) -> bool {
    if (deviation == Eigen::Vector3d::Zero()) {
      if (std::abs(pos1.x() - pos2.x()) <
              std::numeric_limits<double>::epsilon() &&
          std::abs(pos1.y() - pos2.y()) <
              std::numeric_limits<double>::epsilon()) {
        return true;
      }
      return false;
    }
    if (std::abs(pos1.x() - pos2.x()) < deviation.x() &&
        std::abs(pos1.y() - pos2.y()) < deviation.y()) {
      return true;
    }
    return false;
  }

  enum PathState {
    running,
    successed,
    failed,
    idle,
  } path_state_ = PathState::idle;
  // PathState opt_state_=PathState::idle;
  path_planning::AStar::Trajectory astar_traj_;
  std::shared_ptr<grid_map::GridMap> grid_map_;
  float safe_threshold_;
  Eigen::Vector3d old_goal_pose_;

};
} // namespace planner