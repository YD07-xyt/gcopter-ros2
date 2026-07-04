// #include "ros2/ros2.hpp"
// #include <Eigen/src/Core/Matrix.h>
// #include <cstddef>
// #include <nav_msgs/msg/detail/path__struct.hpp>
// #include <spdlog/spdlog.h>

// namespace planner {
// GlobalPlanner::GlobalPlanner(rclcpp::Node::SharedPtr nh_)
//     : config(nh_), nh(nh_), mapInitialized(false), visualizer(nh_){
//   const Eigen::Vector3i xyz(
//       (config.mapBound[1] - config.mapBound[0]) / config.voxelWidth,
//       (config.mapBound[3] - config.mapBound[2]) / config.voxelWidth,
//       (config.mapBound[5] - config.mapBound[4]) / config.voxelWidth);

//   Eigen::Vector3d offset(config.mapBound[0], config.mapBound[2],
//                          config.mapBound[4]);
//   mpc_ = std::make_shared<Mpc>();
//   mpc_->setWeights(config.mpc_weights);
// //   astar_planner_.setParam();
// //   astar_planner_.init();

//   mapSub = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
//       config.mapTopic, rclcpp::SensorDataQoS(),
//       [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
//         GlobalPlanner::mapCallBack(msg);
//       });
//   OdomSub = nh->create_subscription<nav_msgs::msg::Odometry>(
//       config.odomTopic, rclcpp::QoS(10),
//       [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
//         GlobalPlanner::odomCallBack(msg);
//       });
//   targetSub = nh->create_subscription<geometry_msgs::msg::PoseStamped>(
//       config.targetTopic, rclcpp::QoS(10),
//       [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
//         targetCallBack(msg);
//       });
//       global_path_pub_=nh->create_publisher<nav_msgs::msg::Path>("global_path",10);
//   if (this->current_pose.has_value()) {
//     offset.x() = current_pose->x() - xyz.x() * 0.5;
//     offset.y() = current_pose->y() - xyz.y() * 0.5;
//     offset.z() = current_pose->z();
//   }
//   voxelMap = voxel_map::VoxelMap(xyz, offset, config.voxelWidth);

//   // 每 0.1 秒执行一次
//   planner_timer_ =
//       nh->create_wall_timer(std::chrono::milliseconds(100), // 时间间隔参数
//                             [&]() { planner_callback(); }          // 回调函数
//     );

// }
// void GlobalPlanner::planner_callback() { 
//   GlobalPlanner::plan_omni(); 
//   //fsm_.plan();
// }
// void GlobalPlanner::odomCallBack(
//     const nav_msgs::msg::Odometry::SharedPtr &msg) {
//   if (!current_pose.has_value()) {
//     current_pose = Eigen::Vector3d::Zero();
//   }
//   current_pose->x() = msg->pose.pose.position.x;
//   current_pose->y() = msg->pose.pose.position.y;
//   current_pose->z() = msg->pose.pose.position.z;

//   //spdlog::info("callback current_plose_:x:{},y:{}",current_pose->x(),current_pose->y());
//   // spdlog::info("Received current pose with position ({},{},{})",
//   //              msg->pose.pose.position.x, msg->pose.pose.position.y,
//   //              msg->pose.pose.position.z);
// };

// void GlobalPlanner::mapCallBack(
//     const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
//   // RCLCPP_INFO(nh->get_logger(), "Received map point cloud with %zu points",
//   //             msg->data.size() / msg->point_step);
//   std::vector<Eigen::Vector3d> pc;
//   size_t cur = 0;
//   const size_t total = msg->data.size() / msg->point_step;
//   float *fdata = (float *)(&msg->data[0]);
//   for (size_t i = 0; i < total; i++) {
//     cur = msg->point_step / sizeof(float) * i;
//     if (std::isnan(fdata[cur + 0]) || std::isinf(fdata[cur + 0]) ||
//         std::isnan(fdata[cur + 1]) || std::isinf(fdata[cur + 1]) ||
//         std::isnan(fdata[cur + 2]) || std::isinf(fdata[cur + 2])) {
//       spdlog::warn("map continue");
//       continue;
//     }
//     voxelMap.setOccupied(Eigen::Vector3d(fdata[cur + 0], fdata[cur + 1], 0));

//     pc.emplace_back(Eigen::Vector3d(fdata[cur + 0], fdata[cur + 1], 0));
//   }

//   voxelMap.dilate(std::ceil(config.dilateRadius / voxelMap.getScale()));
//   // const auto& v = voxelMap.getVoxels();
//   // size_t occ = std::count(v.begin(), v.end(), voxel_map::Occupied);
//   // size_t dil = std::count(v.begin(), v.end(), voxel_map::Dilated);
//   // spdlog::info("Occupied: {}, Dilated: {}", occ, dil);

//   mapInitialized = true;

//   voxelMap.getOccupiedVoxels(pc);
//   visualizer.visualizeMap(pc);
//   // std::vector<Eigen::Vector3d> pc0;
//   // std::vector<Eigen::Vector3d> pc1;
//   // voxelMap.getSurf(pc0);

//   // voxelMap.getDilatedVoxels(pc1);
//   // visualizer.visualizeSurfMap(pc1);

//   //astar_planner_.setEnvironment(voxelMap);
// }
// void GlobalPlanner::targetCallBack(
//     const geometry_msgs::msg::PoseStamped::SharedPtr &msg) {
//   spdlog::info("Received target pose with position ({},{},{})",
//                msg->pose.position.x, msg->pose.position.y,
//                msg->pose.position.z);
//   if (mapInitialized) {
//     // if (startGoal.size() >= 2)
//     // {
//     //     startGoal.clear();
//     // }
//     const double zGoal =
//         config.mapBound[4] + config.dilateRadius +
//         fabs(msg->pose.orientation.z) *
//             (config.mapBound[5] - config.mapBound[4] - 2 * config.dilateRadius);
//     const Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, 0.0);
//     if (voxelMap.query(goal) == 0) {
//       // visualizer.visualizeStartGoal(goal, 0.5, startGoal.size());
//       // startGoal.emplace_back(goal);
//       goal_pose = goal;

//       spdlog::info("get goal");
//     } else {
//       spdlog::warn("Infeasible Position Selected !!!");
//     }
//   } else {
//     spdlog::warn("map no init");
//   }
//   return;
// }

// void GlobalPlanner::plan_omni() {
//   if (this->goal_pose == std::nullopt) {
//     //spdlog::info("no goal");
//   }
//   if (this->current_pose == std::nullopt) {
//     //spdlog::info("no current_pose");
//   }
//   if (this->goal_pose != std::nullopt && this->current_pose != std::nullopt) {
//     std::vector<Eigen::Vector3d> route;
//     sfc_gen::planPath<voxel_map::VoxelMap>(
//         current_pose.value(), goal_pose.value(), voxelMap.getOrigin(),
//         voxelMap.getCorner(), &voxelMap, 0.01, route);

//     if (route.empty()) {
//       spdlog::warn("path is empty");
//     }
//     route=replan(route);
//     PubPath(route);
//     goal_pose=std::nullopt;
//   spdlog::info(" getObstaclePointCloud end");
//     std::vector<Eigen::MatrixX4d> hPolys;
//     std::vector<Eigen::Vector3d> pc;

//     voxelMap.getDilatedVoxels(pc);
   
//     visualizer.visualizeSurfMap(pc);

//     sfc_gen::convexCover(route, pc, voxelMap.getOrigin(), voxelMap.getCorner(),
//                          7.0, 3.0, hPolys);

//     sfc_gen::shortCut(hPolys);
//     // 假设 refZ 是期望的恒定高度，margin 是允许的微小波动
    
//     const double refZ = current_pose->z();
//     const double margin = 0.2; // 允许走廊厚度 2*margin

//     for (auto &poly : hPolys) {
//       int rows = poly.rows();
//       poly.conservativeResize(rows + 2, Eigen::NoChange);
//       poly.row(rows) << 0.0, 0.0, 1.0, -(refZ + margin); // z ≤ refZ + margin
//       poly.row(rows + 1) << 0.0, 0.0, -1.0,
//           (refZ - margin); // z ≥ refZ - margin
//     }

//     if (route.size() > 1) {
//       visualizer.visualizePolytope(hPolys);
//       spdlog::info("Visualizing route with {} points",
//       route.size());
//       Eigen::Matrix3d iniState;
//       Eigen::Matrix3d finState;
//       // iniState 格式：[位置, 速度, 加速度]
//       iniState << route.front(), Eigen::Vector3d::Zero(),
//           Eigen::Vector3d::Zero();
//       finState << route.back(), Eigen::Vector3d::Zero(),
//           Eigen::Vector3d::Zero();
//       gcopter::OMNI_PolytopeSFC omni_gcopter;
//       // magnitudeBounds = [v_max, omg_max, theta_max,
//       // thrust_min,thrust_max]^T penaltyWeights = [pos_weight, vel_weight,
//       // omg_weight,theta_weight, thrust_weight]^T physicalParams =
//       // [vehicle_mass,gravitational_acceleration, horitonral_drag_coeff,
//       //                 vertical_drag_coeff,
//       //                 parasitic_drag_coeff,speed_smooth_factor]^T

//       // initialize some constraint parameters
//       //初始化一些约束参数
//       Eigen::VectorXd magnitudeBounds(2); // [v_max, a_max] //,ω_max, θ_max]
//       Eigen::VectorXd penaltyWeights(
//           3); // 各成本项权重 [位置权重 速度权重 加速度约束权重]

//       magnitudeBounds(0) = config.maxVelMag; // 最大速度
//       magnitudeBounds(1) = config.maxBdrMag; // 最大加速度

//       penaltyWeights(0) = config.chiVec[0]; // 位置权重
//       penaltyWeights(1) = config.chiVec[1]; // 速度权重
//       penaltyWeights(2) = config.chiVec[2]; // 角速度权重

//       const int quadratureRes = config.integralIntervs; // 积分区间数

//       traj.clear();

//       if (!omni_gcopter.setup(config.weightT, // 时间权重
//                               iniState,       // 初始状态
//                               finState,       // 最终状态
//                               hPolys,         // 凸约束集
//                               INFINITY,       // 最大时间（无限制）
//                               config.smoothingEps, // 平滑系数
//                               quadratureRes,       // 离散化点数
//                               magnitudeBounds,     // 约束边界
//                               penaltyWeights       // 惩罚权重
//                               )) {
//         spdlog::warn("omni_gcopter no setup");
//         return;
//       }
//       // 执行优化
//       if (std::isinf(omni_gcopter.optimize(traj, config.relCostTol))) {
//         spdlog::warn("omni_gcopter优化失败(返回无穷大)");
//         return; // 优化失败（返回无穷大）
//       }

//       if (traj.getPieceNum() > 0) {
//         trajStamp = nh->now().seconds();
//         visualizer.visualize(traj, route);
//         // spdlog::info("Visualizing optimized trajectory");
//       } else {
//         spdlog::info("Failed to optimize trajectory");
//       }

//       // auto tarj = traj.getPositions();
//       //==================================================//
//       //======================MPC========================//
//       //==================================================//
//       int N = 10;
//       auto t_cur = nh->now().seconds();
//       double dt = 0.1;
//       Eigen::MatrixXd desired_states(3, N + 1);

//       for (int i = 0; i <= N; ++i) {
//         double t_sample = t_cur + i * dt;
//         if (t_sample > traj.getTotalDuration()) {
//           // 超出轨迹终点：使用终点状态
//           t_sample = traj.getTotalDuration();
//         }
//         Eigen::Vector3d pos = traj.getPos(t_sample);
//         Eigen::Vector3d vel = traj.getVel(t_sample);
//         double theta = std::atan2(vel(1), vel(0)); // 从速度方向计算偏航角
//         desired_states(0, i) = pos(0);
//         desired_states(1, i) = pos(1);
//         desired_states(2, i) = theta;
//       }

//       if (mpc_->solve(current_pose.value(), desired_states)) {
//         // 5. 获取第一个控制量
//         auto u = mpc_->getFirstU();
//         geometry_msgs::msg::Twist cmd;
//         cmd.linear.x = u[0];
//         cmd.linear.y = u[1];
//         // cmd_vel_pub_->publish(cmd);
//         //spdlog::info("v={},w={}", u[0], u[1]);
//         // 6. 可选：获取预测轨迹
//         std::vector<double> traj = mpc_->getPredictX();
//         // traj 中每两个元素为 (x_i, y_i)
//         for (size_t i = 0; i < traj.size(); i += 2) {
//         //   spdlog::info("predicted step:{},x={},y={}", i / 2, traj[i],
//         //                traj[i + 1]);
//         }
//       } else {
//         spdlog::error("MPC solver failed!");
//       }
//     }
//   }
// }

// void GlobalPlanner::process() {
//   Eigen::VectorXd physicalParams(6);
//   physicalParams(0) = config.vehicleMass;
//   physicalParams(1) = config.gravAcc;
//   physicalParams(2) = config.horizDrag;
//   physicalParams(3) = config.vertDrag;
//   physicalParams(4) = config.parasDrag;
//   physicalParams(5) = config.speedEps;

//   flatness::FlatnessMap flatmap;
//   flatmap.reset(physicalParams(0), physicalParams(1), physicalParams(2),
//                 physicalParams(3), physicalParams(4), physicalParams(5));

//   if (traj.getPieceNum() > 0) {
//     const double delta = nh->now().seconds() - trajStamp;
//     if (delta > 0.0 && delta < traj.getTotalDuration()) {
//       double thr;
//       Eigen::Vector4d quat;
//       Eigen::Vector3d omg;

//       flatmap.forward(traj.getVel(delta), traj.getAcc(delta),
//                       traj.getJer(delta), 0.0, 0.0, thr, quat, omg);
//       double speed = traj.getVel(delta).norm();
//       double bodyratemag = omg.norm();
//       double tiltangle =
//           acos(1.0 - 2.0 * (quat(1) * quat(1) + quat(2) * quat(2)));
//       std_msgs::msg::Float64 speedMsg, thrMsg, tiltMsg, bdrMsg;
//       speedMsg.data = speed;
//       thrMsg.data = thr;
//       tiltMsg.data = tiltangle;
//       bdrMsg.data = bodyratemag;
//       visualizer.speedPub->publish(speedMsg);
//       visualizer.thrPub->publish(thrMsg);
//       visualizer.tiltPub->publish(tiltMsg);
//       visualizer.bdrPub->publish(bdrMsg);

//       visualizer.visualizeSphere(traj.getPos(delta), config.dilateRadius);
//     }
//   }
// }
// } // namespace planner