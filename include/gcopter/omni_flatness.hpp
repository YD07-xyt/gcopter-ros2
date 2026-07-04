#pragma once
#include <Eigen/Core>
namespace planner {
//无用
class OmnidirectionalFlatnessMap {
public:
  inline void reset(const double &vehicle_mass, const double &drag_coeff_x,
                    const double &drag_coeff_y, const double &drag_coeff_z,
                    const double &speed_smooth_factor) {
    mass = vehicle_mass;
    dx = drag_coeff_x;
    dy = drag_coeff_y;
    dz = drag_coeff_z;
    veps = speed_smooth_factor;
  }

  inline void forward(const Eigen::Vector3d &vel, const Eigen::Vector3d &acc,
                      const Eigen::Vector3d &jer,
                      const double & /*psi*/, // 全向机器人偏航可忽略或独立控制
                      const double & /*dpsi*/,
                      Eigen::Vector3d &force, // 改为力矢量（3自由度）
                      Eigen::Vector3d &omg) { // 角速度（如果需要）

    // 计算速度范数（用于阻力）
    double v_norm =
        sqrt(vel(0) * vel(0) + vel(1) * vel(1) + vel(2) * vel(2) + veps);

    // 各轴独立阻力
    Eigen::Vector3d drag;
    drag(0) = dx * vel(0);
    drag(1) = dy * vel(1);
    drag(2) = dz * vel(2);

    // 牛顿第二定律：F = m*a + 阻力 + 重力（z方向）
    force(0) = mass * acc(0) + drag(0);
    force(1) = mass * acc(1) + drag(1);
    force(2) = mass * (acc(2) + 9.81) + drag(2);

    // 全向机器人通常不需要从力计算姿态，可直接输出力
    // 角速度可以根据需要从jer计算（简单微分）
    omg.setZero(); // 或根据偏航角速率设置

    // 缓存中间变量供backward使用
    v0 = vel(0);
    v1 = vel(1);
    v2 = vel(2);
    a0 = acc(0);
    a1 = acc(1);
    a2 = acc(2);
    f0 = force(0);
    f1 = force(1);
    f2 = force(2);
    drag0 = drag(0);
    drag1 = drag(1);
    drag2 = drag(2);
  }

  inline void
  backward(const Eigen::Vector3d &pos_grad, const Eigen::Vector3d &vel_grad,
           const Eigen::Vector3d &force_grad, // 改为力梯度
           const Eigen::Vector3d &omg_grad, Eigen::Vector3d &pos_total_grad,
           Eigen::Vector3d &vel_total_grad, Eigen::Vector3d &acc_total_grad,
           Eigen::Vector3d &jer_total_grad) const {

    // 力对加速度的梯度
    acc_total_grad(0) = mass * force_grad(0);
    acc_total_grad(1) = mass * force_grad(1);
    acc_total_grad(2) = mass * force_grad(2);

    // 力对速度的梯度（阻力部分）
    vel_total_grad(0) = vel_grad(0) + dx * force_grad(0);
    vel_total_grad(1) = vel_grad(1) + dy * force_grad(1);
    vel_total_grad(2) = vel_grad(2) + dz * force_grad(2);

    // 位置梯度直接传递
    pos_total_grad = pos_grad;

    // jer的梯度（如果jer影响力和力矩，可添加）
    jer_total_grad.setZero();
  }

private:
  double mass, dx, dy, dz, veps;
  double v0, v1, v2, a0, a1, a2;
  double f0, f1, f2;
  double drag0, drag1, drag2;
};
} // namespace planner