#include "../include/controller/omni_mpc.hpp"
#include <casadi/core/mx.hpp>
#include <spdlog/spdlog.h>
namespace planner {


OmniMpc::OmniMpc() {
    N_   = 10;
    dt_  = 0.1;
    v_x_max_ = 4.0;  v_x_min_ = -4.0;
    v_y_max_ = 4.0;  v_y_min_ = -4.0;
    w_max_   = 2.0;  w_min_   = -2.0;

    // 权重矩阵 (对角)
    Q_ = casadi::DM::zeros(3,3);
    R_ = casadi::DM::zeros(3,3);
    Q_(0,0) = 10; Q_(1,1) = 10; Q_(2,2) = 1;
    R_(0,0) = 1;  R_(1,1) = 1;  R_(2,2) = 1;

    kinematic_equation_ = setKinematicEquation();   // 与原来相同

    buildOptimizationProblem();
}
OmniMpc::~OmniMpc() {}

// setWeights 改为接受6个权重（Q 3x3, R 3x3）
void OmniMpc::setWeights(std::vector<double> weights) {
  // weights: [qx, qy, qtheta, rvx, rvy, rw]
  Q_(0, 0) = weights[0];
  Q_(1, 1) = weights[1];
  Q_(2, 2) = weights[2];
  R_(0, 0) = weights[3];
  R_(1, 1) = weights[4];
  R_(2, 2) = weights[5];
}

casadi::Function OmniMpc::setKinematicEquation() {

  casadi::MX x = casadi::MX::sym("x");
  casadi::MX y = casadi::MX::sym("y");
  casadi::MX theta = casadi::MX::sym("theta");
  casadi::MX state_vars = casadi::MX::vertcat({x, y, theta});

  casadi::MX v_x = casadi::MX::sym("v_x");
  casadi::MX v_y = casadi::MX::sym("v_y");
  casadi::MX w = casadi::MX::sym("w");
  casadi::MX control_vars = casadi::MX::vertcat({v_x, v_y, w});

  // rhs means right hand side
  casadi::MX rhs = casadi::MX::vertcat({
      v_x * casadi::MX::cos(theta) - v_y * casadi::MX::sin(theta),
      v_x * casadi::MX::sin(theta) + v_y * casadi::MX::cos(theta),
      w // 如果无旋转，角速度为0
  });
  return casadi::Function("kinematic_equation", {state_vars, control_vars},
                          {rhs});
}
void OmniMpc::buildOptimizationProblem() {
    using namespace casadi;

    // 创建 Opti 栈
    opti_ = Opti();

    // ---- 定义决策变量 ----
    X = opti_.variable(3, N_ + 1);   // 状态 [x; y; theta] 共 N+1 个点
    U = opti_.variable(3, N_);       // 控制 [vx; vy; w] 共 N 个区间

    Slice all;

    // ---- 定义参数 ----
    X_ref = opti_.parameter(3, N_ + 1);  // 参考轨迹
    X_cur = opti_.parameter(3);          // 当前状态

    // ---- 构建代价函数 ----
    MX cost = 0;
    for (int i = 0; i < N_; ++i) {
        MX X_err = X(all, i) - X_ref(all, i);
        MX U_i   = U(all, i);
        cost += MX::mtimes({X_err.T(), Q_, X_err});
        cost += MX::mtimes({U_i.T(), R_, U_i});
    }
    // 终端代价
    cost += MX::mtimes({(X(all, N_) - X_ref(all, N_)).T(),
                        Q_,
                        X(all, N_) - X_ref(all, N_)});
    opti_.minimize(cost);

    // ---- 动力学约束 ----
    for (int i = 0; i < N_; ++i) {
        std::vector<MX> input = {X(all, i), U(all, i)};
        MX X_next = kinematic_equation_(input)[0] * dt_ + X(all, i);
        opti_.subject_to(X_next == X(all, i + 1));
    }

    // 初始状态约束
    opti_.subject_to(X(all, 0) == X_cur);

    // ---- 控制量限幅 ----
    //opti_.subject_to(v_x_min_ <= X(0, all) <= v_x_max_);   // 注意这里要对 U 切片，不是 X！
    // 修正：应该是对 U 的行做限制
    opti_.subject_to(v_x_min_ <= U(0, all) <= v_x_max_);
    opti_.subject_to(v_y_min_ <= U(1, all) <= v_y_max_);
    opti_.subject_to(w_min_   <= U(2, all) <= w_max_);

    // ---- 配置求解器 ----
    Dict solver_opts;
    solver_opts["expand"] = true;
    solver_opts["ipopt.max_iter"] = 100;
    solver_opts["ipopt.print_level"] = 0;
    solver_opts["print_time"] = 0;
    solver_opts["ipopt.tol"] = 1e-4;              // 放宽精度可提速
    solver_opts["ipopt.acceptable_tol"] = 1e-4;
    solver_opts["ipopt.acceptable_obj_change_tol"] = 1e-4;

    opti_.solver("ipopt", solver_opts);

    initialized_ = true;

}

bool OmniMpc::solve(Eigen::Vector3d current_states,
                    Eigen::MatrixXd desired_states) {
    if (!initialized_) return false;

    // 1. 设置当前状态参数
    casadi::DM cur = casadi::DM::vertcat({
        current_states(0), current_states(1), current_states(2)
    });
    opti_.set_value(X_cur, cur);

    // 2. 设置参考轨迹参数 (desired_states 是 3 x (N+1) 列优先)
    //    先将 Eigen 数据拷贝到 std::vector 再转为 DM
    std::vector<double> ref_data(desired_states.data(),
                                 desired_states.data() + desired_states.size());
    casadi::DM ref_dm = casadi::DM::reshape(casadi::DM(ref_data), 3, N_ + 1);
    opti_.set_value(X_ref, ref_dm);

    // 3. Warm start (热启动)
    if (solution_) {
        opti_.set_initial(X, solution_->value(X));
        opti_.set_initial(U, solution_->value(U));
        // 可选：设置拉格朗日乘子初值
        // opti_.set_initial(opti_.lam_g(), solution_->value(opti_.lam_g()));
    } else {
        // 第一次求解给一个简单初值（例如全部零）
        opti_.set_initial(X, 0);
        opti_.set_initial(U, 0);
    }

    // 4. 求解
    try {
        auto sol = opti_.solve();
        solution_ = std::make_unique<casadi::OptiSol>(sol);
        return true;
    } catch (const std::exception& e) {
        spdlog::error("MPC solve failed: {}", e.what());
        return false;
    }
}

std::vector<double> OmniMpc::getFirstU() {
    std::vector<double> res;
    if (!solution_) return {0.0, 0.0, 0.0};
    auto val = solution_->value(U);
    res.push_back(static_cast<double>(val(0,0)));
    res.push_back(static_cast<double>(val(1,0)));
    res.push_back(static_cast<double>(val(2,0)));
    return res;
}

std::vector<double> OmniMpc::getPredictX() {
  std::vector<double> res;
  auto predict_x = solution_->value(X);

  spdlog::info("nomal");
  // cout << "predict_x size :" << predict_x.size() << endl;
  for (int i = 0; i <= N_; ++i) {
    res.push_back(static_cast<double>(predict_x(0, i)));
    res.push_back(static_cast<double>(predict_x(1, i)));
  }
  return res;
}
}