#pragma once
#include <vector>
#include <Eigen/Dense>
#include<spdlog/spdlog.h>
#include <casadi/casadi.hpp>
#include <chrono>
namespace planner {


class OmniMpc
{
private:
       // 优化问题对象（复用）
    casadi::Opti opti_;

    // 参数符号（每次 solve 时填入实际值）
    casadi::MX X_ref;   // 参考轨迹，3 × (N+1)
    casadi::MX X_cur;   // 当前状态，3 × 1

    // 决策变量符号（复用）
    casadi::MX X;       // 状态变量 3 × (N+1)
    casadi::MX U;       // 控制变量 3 × N

    // 求解结果
    std::unique_ptr<casadi::OptiSol> solution_;

    // 运动学方程（不变）
    casadi::Function kinematic_equation_;

    // 初始化标志，防止重复构造
    bool initialized_ = false;

    void buildOptimizationProblem();  // 在构造函数中调用




    //mpc params
    int N_;  //horizon
    double dt_;  //step
    //constrains
    double v_x_max_, v_x_min_;
    double v_y_max_, v_y_min_;
    double w_max_, w_min_;
    
    //weights
    casadi::DM Q_, R_;
    
public:
    OmniMpc();
    ~OmniMpc();

    casadi::Function setKinematicEquation();
    void setWeights(std::vector<double> weights);
    bool solve(Eigen::Vector3d current_states, Eigen::MatrixXd desired_states);
    std::vector<double> getFirstU();
    std::vector<double> getPredictX();
};
}
