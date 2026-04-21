#include "misc/visualizer.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/firi.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"

#include<rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <random>
struct Config
{
    std::string mapTopic = "/map";
    std::string targetTopic = "/target";
    double dilateRadius = 0.2;
    double voxelWidth = 0.1;
    std::vector<double> mapBound = {-10.0, 10.0, -10.0, 10.0, -5.0, 5.0};
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

    Config(const rclcpp::Node::SharedPtr &node)
    {
        // 声明参数（带默认值）
        node->declare_parameter<std::string>("map_topic", mapTopic);
        node->declare_parameter<std::string>("target_topic", targetTopic);
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

class GlobalPlanner
{
private:
    Config config;

    rclcpp::Node::SharedPtr nh;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mapSub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr targetSub;

    bool mapInitialized;
    voxel_map::VoxelMap voxelMap;
    Visualizer visualizer;
    std::vector<Eigen::Vector3d> startGoal;

    Trajectory<5> traj;
    double trajStamp;

public:
    GlobalPlanner(const Config &conf,
                  rclcpp::Node::SharedPtr nh_)
        : config(conf),
          nh(nh_),
          mapInitialized(false),
          visualizer(nh)
    {
       const Eigen::Vector3i xyz((config.mapBound[1] - config.mapBound[0]) / config.voxelWidth,
                                  (config.mapBound[3] - config.mapBound[2]) / config.voxelWidth,
                                  (config.mapBound[5] - config.mapBound[4]) / config.voxelWidth);

        const Eigen::Vector3d offset(config.mapBound[0], config.mapBound[2], config.mapBound[4]);

        voxelMap = voxel_map::VoxelMap(xyz, offset, config.voxelWidth);

        mapSub = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
            config.mapTopic,
            rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                mapCallBack(msg);
            }
        );

        targetSub = nh->create_subscription<geometry_msgs::msg::PoseStamped>(
            config.targetTopic,
            rclcpp::QoS(10),
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                targetCallBack(msg);
            }
        );
    }

    inline void mapCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr &msg)
    {
        //RCLCPP_INFO(nh->get_logger(), "Received map point cloud with %zu points", msg->data.size() / msg->point_step);
        if (!mapInitialized)
        {
            size_t cur = 0;
            const size_t total = msg->data.size() / msg->point_step;
            float *fdata = (float *)(&msg->data[0]);
            for (size_t i = 0; i < total; i++)
            {
                cur = msg->point_step / sizeof(float) * i;

                if (std::isnan(fdata[cur + 0]) || std::isinf(fdata[cur + 0]) ||
                    std::isnan(fdata[cur + 1]) || std::isinf(fdata[cur + 1]) ||
                    std::isnan(fdata[cur + 2]) || std::isinf(fdata[cur + 2]))
                {
                    continue;
                }
                voxelMap.setOccupied(Eigen::Vector3d(fdata[cur + 0],
                                                     fdata[cur + 1],
                                                     fdata[cur + 2]));
            }

            voxelMap.dilate(std::ceil(config.dilateRadius / voxelMap.getScale()));

            mapInitialized = true;
        }
    }

    inline void plan()
    {
        RCLCPP_INFO(nh->get_logger(), "Planning from %zu start/goal pairs", startGoal.size());
        if (startGoal.size() == 2)
        {
            std::vector<Eigen::Vector3d> route;
            sfc_gen::planPath<voxel_map::VoxelMap>(startGoal[0],
                                                   startGoal[1],
                                                   voxelMap.getOrigin(),
                                                   voxelMap.getCorner(),
                                                   &voxelMap, 0.01,
                                                   route);
            std::vector<Eigen::MatrixX4d> hPolys;
            std::vector<Eigen::Vector3d> pc;
            voxelMap.getSurf(pc);

            sfc_gen::convexCover(route,
                                 pc,
                                 voxelMap.getOrigin(),
                                 voxelMap.getCorner(),
                                 7.0,
                                 3.0,
                                 hPolys);
            sfc_gen::shortCut(hPolys);

            if (route.size() > 1)
            {

                visualizer.visualizePolytope(hPolys);
                RCLCPP_INFO(nh->get_logger(), "Visualizing route with %zu points", route.size());
                Eigen::Matrix3d iniState;
                Eigen::Matrix3d finState;
                iniState << route.front(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
                finState << route.back(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

                gcopter::GCOPTER_PolytopeSFC gcopter;

                // magnitudeBounds = [v_max, omg_max, theta_max, thrust_min, thrust_max]^T
                // penaltyWeights = [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight]^T
                // physicalParams = [vehicle_mass, gravitational_acceleration, horitonral_drag_coeff,
                //                   vertical_drag_coeff, parasitic_drag_coeff, speed_smooth_factor]^T
                // initialize some constraint parameters
                Eigen::VectorXd magnitudeBounds(5);
                Eigen::VectorXd penaltyWeights(5);
                Eigen::VectorXd physicalParams(6);
                magnitudeBounds(0) = config.maxVelMag;
                magnitudeBounds(1) = config.maxBdrMag;
                magnitudeBounds(2) = config.maxTiltAngle;
                magnitudeBounds(3) = config.minThrust;
                magnitudeBounds(4) = config.maxThrust;
                penaltyWeights(0) = (config.chiVec)[0];
                penaltyWeights(1) = (config.chiVec)[1];
                penaltyWeights(2) = (config.chiVec)[2];
                penaltyWeights(3) = (config.chiVec)[3];
                penaltyWeights(4) = (config.chiVec)[4];
                physicalParams(0) = config.vehicleMass;
                physicalParams(1) = config.gravAcc;
                physicalParams(2) = config.horizDrag;
                physicalParams(3) = config.vertDrag;
                physicalParams(4) = config.parasDrag;
                physicalParams(5) = config.speedEps;
                const int quadratureRes = config.integralIntervs;

                traj.clear();

                if (!gcopter.setup(config.weightT,
                                   iniState, finState,
                                   hPolys, INFINITY,
                                   config.smoothingEps,
                                   quadratureRes,
                                   magnitudeBounds,
                                   penaltyWeights,
                                   physicalParams))
                {
                    return;
                }

                if (std::isinf(gcopter.optimize(traj, config.relCostTol)))
                {
                    return;
                }

                if (traj.getPieceNum() > 0)
                {
                    trajStamp = nh->now().seconds();
                    visualizer.visualize(traj, route);
                    RCLCPP_INFO(nh->get_logger(), "Visualizing optimized trajectory");
                }else{
                    RCLCPP_WARN(nh->get_logger(), "Failed to optimize trajectory");
                }
            }
        }
    }

    inline void targetCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr &msg)
    {
        RCLCPP_INFO(nh->get_logger(), "Received target pose with position (%.2f, %.2f, %.2f)", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        if (mapInitialized)
        {
            if (startGoal.size() >= 2)
            {
                startGoal.clear();
            }
            const double zGoal = config.mapBound[4] + config.dilateRadius +
                                 fabs(msg->pose.orientation.z) *
                                     (config.mapBound[5] - config.mapBound[4] - 2 * config.dilateRadius);
            const Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, zGoal);
            if (voxelMap.query(goal) == 0)
            {
                visualizer.visualizeStartGoal(goal, 0.5, startGoal.size());
                startGoal.emplace_back(goal);
            }
            else
            {
                RCLCPP_WARN(nh->get_logger(), "Infeasible Position Selected !!!");
            }

            plan();
        }
        return;
    }

    inline void process()
    {
        Eigen::VectorXd physicalParams(6);
        physicalParams(0) = config.vehicleMass;
        physicalParams(1) = config.gravAcc;
        physicalParams(2) = config.horizDrag;
        physicalParams(3) = config.vertDrag;
        physicalParams(4) = config.parasDrag;
        physicalParams(5) = config.speedEps;

        flatness::FlatnessMap flatmap;
        flatmap.reset(physicalParams(0), physicalParams(1), physicalParams(2),
                      physicalParams(3), physicalParams(4), physicalParams(5));

        if (traj.getPieceNum() > 0)
        {
            const double delta = nh->now().seconds() - trajStamp;
            if (delta > 0.0 && delta < traj.getTotalDuration())
            {
                double thr;
                Eigen::Vector4d quat;
                Eigen::Vector3d omg;

                flatmap.forward(traj.getVel(delta),
                                traj.getAcc(delta),
                                traj.getJer(delta),
                                0.0, 0.0,
                                thr, quat, omg);
                double speed = traj.getVel(delta).norm();
                double bodyratemag = omg.norm();
                double tiltangle = acos(1.0 - 2.0 * (quat(1) * quat(1) + quat(2) * quat(2)));
                std_msgs::msg::Float64 speedMsg, thrMsg, tiltMsg, bdrMsg;
                speedMsg.data = speed;
                thrMsg.data = thr;
                tiltMsg.data = tiltangle;
                bdrMsg.data = bodyratemag;
                visualizer.speedPub->publish(speedMsg);
                visualizer.thrPub->publish(thrMsg);
                visualizer.tiltPub->publish(tiltMsg);
                visualizer.bdrPub->publish(bdrMsg);

                visualizer.visualizeSphere(traj.getPos(delta),
                                           config.dilateRadius);
            }
        }
    }
};

int main(int argc, char **argv)
{
    // 1. 初始化 ROS2
    rclcpp::init(argc, argv);

    // 2. 创建节点
    auto nh = std::make_shared<rclcpp::Node>("global_planning_node");

    // 3. 从节点参数加载配置（替代原来的 ros::NodeHandle("~")）
    //    假设 Config 有一个从节点读取参数的构造函数或静态方法
    Config planner_config(nh);
   // 4. 创建规划器实例
    GlobalPlanner global_planner(planner_config, nh);

    // 5. 控制循环（替代 ros::Rate + ros::spinOnce）
    rclcpp::WallRate rate(1000);   // 1000 Hz，与原始 ros::Rate(1000) 相同
    while (rclcpp::ok())
    {
        // 执行主要处理函数
        global_planner.process();

        // 处理订阅回调等（等价于 ros::spinOnce）
        rclcpp::spin_some(nh);

        // 保持周期频率
        rate.sleep();
    }

    // 6. 可选：显式关闭（rclcpp::ok() 退出时会自动清理）
    rclcpp::shutdown();
    return 0;
}