// #include "ros2/ros2_2d.hpp"
// #include "SafeCorridor/corridor-Inflation_minimumsnap.hpp"
// #include <Eigen/src/Core/Matrix.h>
// #include <cstddef>
// #include <nav_msgs/msg/detail/path__struct.hpp>
// #include <spdlog/spdlog.h>

// namespace planner {
// GlobalPlanner2d::GlobalPlanner2d(rclcpp::Node::SharedPtr nh_)
//     : config(nh_), nh(nh_), mapInitialized(false), visualizer(nh_),
//     grid(0.1, Eigen::Vector2i(20, 20)),astar_() {
//     grid_map_= std::make_shared<grid_map::GridMap>();
//     const double map_size = 20.0;
//     const double resolution = 0.1;
//     grid_map_->init(map_size, map_size, resolution);

//   mapSub = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
//       config.mapTopic, rclcpp::SensorDataQoS(),
//       [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
//         GlobalPlanner2d::mapCallBack(msg);
//       });
//   OdomSub = nh->create_subscription<nav_msgs::msg::Odometry>(
//       config.odomTopic, rclcpp::QoS(10),
//       [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
//         GlobalPlanner2d::odomCallBack(msg);
//       });
//   targetSub = nh->create_subscription<geometry_msgs::msg::PoseStamped>(
//       config.targetTopic, rclcpp::QoS(10),
//       [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
//         targetCallBack(msg);
//       });
//       global_path_pub_=nh->create_publisher<nav_msgs::msg::Path>("global_path",10);
//   // 每 0.1 秒执行一次
//   planner_timer_ =
//       nh->create_wall_timer(std::chrono::milliseconds(100), // 时间间隔参数
//                             [&]() { planner_callback(); }          // 回调函数
//     );

// }
// void GlobalPlanner2d::planner_callback() { 
//   GlobalPlanner2d::plan_omni(); 
// }
// void GlobalPlanner2d::plan_omni(){
//     if (this->goal_pose == std::nullopt) {
//     //spdlog::info("no goal");
//   }
//   if (this->current_pose == std::nullopt) {
//     //spdlog::info("no current_pose");
//   }
//   if (this->goal_pose != std::nullopt && this->current_pose != std::nullopt) {
//       // Start timing
//     auto start_time = std::chrono::high_resolution_clock::now();

//     // ==================== 3. Define Start and Goal ====================
//     const Eigen::Vector2d start(current_pose->x(), current_pose->y());
//     const Eigen::Vector2d goal(goal_pose->x(), goal_pose->y());

//     // ==================== 4. A* Global Search ====================
//     path_planning::AStar astar(*grid_map_, 0.2); // 0.2m safety threshold
//     auto astar_traj = astar.planWithPostProcessing(start, goal, 5000); // 5s timeout
    
//     if (astar_traj.optimized_path.empty()) {
//         std::cerr << "A* planning failed!" << std::endl;
//         return;
//     }
//     PubPath(astar_traj.optimized_path);
//     std::cout << "=== A* Planning Results ===" << std::endl;
//     std::cout << "Optimized path points: " << astar_traj.optimized_path.size() << std::endl;
//     std::cout << "Total length: " << astar_traj.total_length << " m" << std::endl;
//     std::cout << "Total time: " << astar_traj.total_time << " s" << std::endl;
    
//     // End timing
//     auto end_time = std::chrono::high_resolution_clock::now();
//         // ==================== 5. Trajectory Optimization ====================
//     TrajOpt::TrajectoryParams params;
//     params.piece_len = astar_traj.total_length / astar_traj.total_time;
//     params.total_time = astar_traj.total_time;
//     params.total_len = astar_traj.total_length;

//     TrajOpt::TrajectoryOptimizer optimizer(
//         grid_map_,
//         astar_traj.optimized_path,
//         params
//     );

//     if (!optimizer.plan()) {
//         std::cerr << "Trajectory optimization failed!" << std::endl;
//         return;
//     }

//     auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

//     // ==================== 6. Evaluate Optimization Results ====================
//     auto metrics = optimizer.evaluateTrajectory();
//     // std::cout << "\n=== Optimization Metrics ===" << std::endl;
//     // std::cout << "Max velocity: " << metrics.max_velocity << " m/s" << std::endl;
//     // std::cout << "Min clearance: " << metrics.min_clearance << " m" << std::endl;
//     // std::cout << "Path deviation: " << metrics.path_deviation << " m" << std::endl;
//     // std::cout << "Trajectory energy: " << metrics.trajectory_energy << std::endl;
//     // std::cout << "Optimization time: " << duration.count() << " ms" << std::endl;

//     // ==================== 7. Visualize Optimized Trajectory ====================
//     auto opt_path = optimizer.sampleTrajectory(0.1); // Sample every 0.1s
//     //PubPath(opt_path);
//   }
// }

// void GlobalPlanner2d::odomCallBack(
//     const nav_msgs::msg::Odometry::SharedPtr &msg) {
//   if (!current_pose.has_value()) {
//     current_pose = Eigen::Vector3d::Zero();
//   }
//   current_pose->x() = msg->pose.pose.position.x;
//   current_pose->y() = msg->pose.pose.position.y;
//   current_pose->z() = msg->pose.pose.position.z;

//   grid.setCurPose(current_pose->x(),current_pose->y());
// // 判断 grid 是否已经正确初始化（map_size 非零）
// if (grid.mapSize().x() > 0 && grid.mapSize().y() > 0 && !astar_initialized_) {
//     Eigen::Vector2i pool_size = grid.mapSize();
//     // 注意：这里应传入 grid 自身的指针，而不是临时拷贝
//     std::cout << "Map size: " << grid.mapSize().transpose() << std::endl;
//     astar_.initGridMap(std::shared_ptr<GridMap2D>(&grid, [](GridMap2D*){}), pool_size);
//     astar_initialized_ = true;
// }
//   //spdlog::info("callback current_plose_:x:{},y:{}",current_pose->x(),current_pose->y());
//   // spdlog::info("Received current pose with position ({},{},{})",
//   //              msg->pose.pose.position.x, msg->pose.pose.position.y,
//   //              msg->pose.pose.position.z);
// };

// void GlobalPlanner2d::mapCallBack(
//     const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
//   // RCLCPP_INFO(nh->get_logger(), "Received map point cloud with %zu points",
//   //             msg->data.size() / msg->point_step);
//   grid_map::RowMatrixXi occupancy = grid_map::RowMatrixXi::Zero(200, 200);


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
//     //====================================//
//     Eigen::Vector2d obstacle_world=Eigen::Vector2d(fdata[cur + 0], fdata[cur + 1]);
//     Eigen::Vector2i obstacle_idx = grid.worldToGrid(obstacle_world);
//     occupancy(obstacle_idx.x(), obstacle_idx.y()) = 1;
//     grid.setObstacle(obstacle_idx,true);
//     grid.inflateObstacles(0.1);
//     //===============================================//

//     pc.emplace_back(Eigen::Vector3d(fdata[cur + 0], fdata[cur + 1], 0));
//   }
//   grid_map_->setMap(occupancy);
//   mapInitialized = true;

//   visualizer.visualizeMap(pc);

// }
// void GlobalPlanner2d::targetCallBack(
//     const geometry_msgs::msg::PoseStamped::SharedPtr &msg) {
//   spdlog::info("Received target pose with position ({},{},{})",
//                msg->pose.position.x, msg->pose.position.y,
//                msg->pose.position.z);
//   if (mapInitialized) {

//     const double zGoal =
//         config.mapBound[4] + config.dilateRadius +
//         fabs(msg->pose.orientation.z) *
//             (config.mapBound[5] - config.mapBound[4] - 2 * config.dilateRadius);
//     const Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, 0.0);


//       goal_pose = goal;

//       spdlog::info("get goal");
 
//   } else {
//     spdlog::warn("map no init");
//   }
//   return;
// }

// void GlobalPlanner2d::plan_omni_corr() {
//   if (this->goal_pose == std::nullopt) {
//     //spdlog::info("no goal");
//   }
//   if (this->current_pose == std::nullopt) {
//     //spdlog::info("no current_pose");
//   }
//   if (this->goal_pose != std::nullopt && this->current_pose != std::nullopt) {
//     std::vector<Eigen::Vector2d> route;

//     bool success = astar_.AstarSearch(0.1, 
//                                       Eigen::Vector2d(current_pose->x(),current_pose->y()),
//                                     Eigen::Vector2d(goal_pose->x(),goal_pose->y()));
//     route=astar_.getPath();

//     if (route.empty()) {
//       spdlog::warn("path is empty");
//     }
//     //route=replan(route);
//     //PubPath(route);
//     goal_pose=std::nullopt;
//     auto t1 = std::chrono::high_resolution_clock::now();

//     auto simplified = simplify_path(route, 30.0, 1);
//     std::cout<<"Simplified length: "<<simplified.size()<<"\n";

//     // --------------------- build corridor rectangles ---------------------
//     double max_width = 7; double extend = 8;
//     std::vector<std::pair<double,double>> path_xy;
//     for(auto &p: simplified) {
//         path_xy.emplace_back(p.first, p.second);
//     }
//     auto rows=grid.mapSize().x();
//     auto cols=grid.mapSize().y();
//     spdlog::info("convex_corridor start");
//     // 走廊初始膨胀
//     // 注意：当四个顶点只有一个被裁剪时会生成梯形
//     auto rects = convex_corridor(path_xy, rows, cols, max_width, extend);
//      spdlog::info("convex_corridor end");
//     // for (const auto& rect : rects) {
//     //     for (const double& value : rect) {
//     //         std::cout << value << " ";
//     //     }
//     //     std::cout << std::endl;
//     // }
//      spdlog::info("corridor_generator_optimized start");
//     // convert and refine by splitting
//     auto grid_map=grid.get_grid();
//     auto corridors = corridor_generator_optimized(path_xy, rects, grid_map, max_width);
//     spdlog::info("corridor_generator_optimized end");
//     auto t2 = std::chrono::high_resolution_clock::now();
//     std::cout<<"Corridor build time: "<<std::chrono::duration<double>(t2-t1).count()<<"s\n";

//     // --------------------- Minimumsnap ---------------------
//     int N = (int)simplified.size();
//     std::vector<pair<int,int>> path_int;
//     for(auto &p: simplified) 
//         path_int.emplace_back(int(round(p.first)), int(round(p.second)));

//     auto traj = minimum_snap_solver(corridors, grid_map, path_int, N, 2, "OSQP", 1.0);
//     auto t3 = std::chrono::high_resolution_clock::now();  
//     std::cout<<"Minimum-snap time: "<<std::chrono::duration<double>(t3-t2).count()<<"s\n";
//     std::vector<Eigen::Vector2d> path;
//     for(size_t i=0;i<traj.size();i++){
//       path.emplace_back(Eigen::Vector2d(traj[i].first,traj[i].second));
//     }
//     PubPath(path);
// }
// }
// } // namespace planner