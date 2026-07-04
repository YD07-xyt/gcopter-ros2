/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef SFC_GEN_HPP
#define SFC_GEN_HPP

#include "geo_utils.hpp"
#include "firi.hpp"

#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/util/Console.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <spdlog/spdlog.h>

#include <deque>
#include <memory>
#include <Eigen/Eigen>

namespace sfc_gen
{
    /**
    @brief: rrt 规划器
    @param: s: 起始点       
    @param: g: 目标点  
    @param: lb: 地图起始点   
    @param: hb: 地图的角点
    @param: map: 地图       
    @param: timeout: 规划超时限制
    @param: p: 路径
    @return: 
    */
    template <typename Map>
    inline double planPath(const Eigen::Vector3d &s,
                           const Eigen::Vector3d &g,
                           const Eigen::Vector3d &lb,
                           const Eigen::Vector3d &hb,
                           const Map *mapPtr,
                           const double &timeout,
                           std::vector<Eigen::Vector3d> &p)
    {
        auto space(std::make_shared<ompl::base::RealVectorStateSpace>(2));

        //ompl::base::RealVectorBounds bounds(3);
        ompl::base::RealVectorBounds bounds(2);
        bounds.setLow(0, 0.0);
        bounds.setHigh(0, hb(0) - lb(0));
        bounds.setLow(1, 0.0);
        bounds.setHigh(1, hb(1) - lb(1));
        // bounds.setLow(2, 0.0);
        // bounds.setHigh(2, hb(2) - lb(2));
        space->setBounds(bounds);
        //空间信息
        auto si(std::make_shared<ompl::base::SpaceInformation>(space));
        //设置碰撞检测
        si->setStateValidityChecker(
            [&](const ompl::base::State *state)
            {
                const auto *pos = state->as<ompl::base::RealVectorStateSpace::StateType>();
                // const Eigen::Vector3d position(lb(0) + (*pos)[0],
                //                                lb(1) + (*pos)[1],
                //                                lb(2) + (*pos)[2]);
                const Eigen::Vector3d position(lb(0) + (*pos)[0],
                                               lb(1) + (*pos)[1],s[2]);
                return mapPtr->query(position) == 0;
            });
        //??
        //如果状态有效性检查分辨率尚未设置，将调用 estimateMaxResolution() 来进行估算
        si->setup();
        //设置要输出的最小日志记录级别。低于此级别的消息将不会被记录
        ompl::msg::setLogLevel(ompl::msg::LOG_NONE);

        //初始化起始点，目标点
        ompl::base::ScopedState<> start(space), goal(space);
        start[0] = s(0) - lb(0);
        start[1] = s(1) - lb(1);
        //start[2] = s(2) - lb(2);
        
        goal[0] = g(0) - lb(0);
        goal[1] = g(1) - lb(1);
        //goal[2] = g(2) - lb(2);
        //goal[2] = 0;

        //
        auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si));
        pdef->setStartAndGoalStates(start, goal);
        //在规划过程中设置要考虑的优化目标----最短路径
        pdef->setOptimizationObjective(std::make_shared<ompl::base::PathLengthOptimizationObjective>(si));
        
        //rrt 规划器
        auto planner(std::make_shared<ompl::geometric::InformedRRTstar>(si));
        //为规划器设置问题定义
        planner->setProblemDefinition(pdef);

        planner->setup();
        //求解规划
        ompl::base::PlannerStatus solved;
        solved = planner->ompl::base::Planner::solve(timeout);

        double cost = INFINITY;
        if (solved)
        {
            //p: 传入的路径
            p.clear();
            const ompl::geometric::PathGeometric path_ =
                ompl::geometric::PathGeometric(
                    dynamic_cast<const ompl::geometric::PathGeometric &>(*pdef->getSolutionPath()));
            for (size_t i = 0; i < path_.getStateCount(); i++)
            {
                const auto state = path_.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>()->values;
                p.emplace_back(lb(0) + state[0], lb(1) + state[1],s[2]);
                //spdlog::info("add point to path");
            }
            //获取规划得到的最优路径的成本值
            cost = pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()).value();
        }
        return cost;
    }
    /**
    @brief:将路径周围的自由空间分解为一系列凸多面体
    @param:path // 输入路径（3D点序列）
    @param:points 环境中的障碍物点
    @param:lowCorner 空间包围盒下角
    @param:highCorner 空间包围盒上角
    @param:progress 路径分段步长
    @param:range 区域扩张范围
    @param:hpolys 输出：凸多面体集合
    @param:eps  数值容差
    */
    inline void convexCover(const std::vector<Eigen::Vector3d> &path,
                            const std::vector<Eigen::Vector3d> &points,
                            const Eigen::Vector3d &lowCorner,
                            const Eigen::Vector3d &highCorner,
                            const double &progress,
                            const double &range,
                            std::vector<Eigen::MatrixX4d> &hpolys,
                            const double eps = 1.0e-6)
    {
        //spdlog::info("start convexCover ");
        hpolys.clear();
        const int n = path.size();
        Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();
        bd(0, 0) = 1.0;
        bd(1, 0) = -1.0;
        bd(2, 1) = 1.0;
        bd(3, 1) = -1.0;
        bd(4, 2) = 1.0;
        bd(5, 2) = -1.0;

        Eigen::MatrixX4d hp, gap;
        Eigen::Vector3d a, b = path[0];
        std::vector<Eigen::Vector3d> valid_pc;
        std::vector<Eigen::Vector3d> bs;
        valid_pc.reserve(points.size());

        //spdlog::info("convexCover init param success");
        // 将原始路径按 progress 步长重新采样
        for (int i = 1; i < n;)
        {
            a = b;
            // 两点距离超过步长，插入中间点
            if ((a - path[i]).norm() > progress)
            {
                b = (path[i] - a).normalized() * progress + a;
            }
            // 两点距离小于步长，直接使用路径点
            else
            {
                b = path[i];
                i++;
            }
            bs.emplace_back(b);
            //为每个分段构建AABB包围盒（带range扩展）
            bd(0, 3) = -std::min(std::max(a(0), b(0)) + range, highCorner(0));
            bd(1, 3) = +std::max(std::min(a(0), b(0)) - range, lowCorner(0));
            bd(2, 3) = -std::min(std::max(a(1), b(1)) + range, highCorner(1));
            bd(3, 3) = +std::max(std::min(a(1), b(1)) - range, lowCorner(1));
            bd(4, 3) = -std::min(std::max(a(2), b(2)) + range, highCorner(2));
            bd(5, 3) = +std::max(std::min(a(2), b(2)) - range, lowCorner(2));
            // 筛选有效障碍物
            valid_pc.clear();
            for (const Eigen::Vector3d &p : points)
            {
                if ((bd.leftCols<3>() * p + bd.rightCols<1>()).maxCoeff() < 0.0)
                {
                    valid_pc.emplace_back(p);
                }
            }
            Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pc(valid_pc[0].data(), 3, valid_pc.size());
            //spdlog::info("success 原始路径按 progress 步长重新采样");
            // 核心：调用 firi::firi 生成凸多面体
            // 输入：边界框(bd)、有效障碍物(pc)、分段起点(a)、终点(b)
            // 输出：凸多面体约束(hp)
            firi::firi(bd, pc, a, b, hp);
            // 检查当前区域与上一个区域是否有足够重叠
            if (hpolys.size() != 0)
            {
                const Eigen::Vector4d ah(a(0), a(1), a(2), 1.0);
                 // 检查起点 a 是否同时满足两个区域的约束
                if (3 <= ((hp * ah).array() > -eps).cast<int>().sum() +
                             ((hpolys.back() * ah).array() > -eps).cast<int>().sum())
                {
                    // 插入过渡区域确保连续性
                    firi::firi(bd, pc, a, a, gap, 1);
                    hpolys.emplace_back(gap);
                }
            }

            hpolys.emplace_back(hp);
        }
        //spdlog::info("convexCover success");
    }
    /**
    @brief:将连续的凸多面体序列进行贪心合并，移除中间不必要的区域，只保留关键的分界点
    @param:凸多面体
    */
    inline void shortCut(std::vector<Eigen::MatrixX4d> &hpolys)
    {
        /**从最后一个区域开始，尽可能向前跳跃到能直接连接的非重叠区域，从而跳过中间的冗余区域**/


        std::vector<Eigen::MatrixX4d> htemp = hpolys;
        //处理单区域特殊情况
        if (htemp.size() == 1)
        {
            Eigen::MatrixX4d headPoly = htemp.front();
            // 复制一份，变成2个相同区域
            htemp.insert(htemp.begin(), headPoly);
        }
        hpolys.clear();

        int M = htemp.size();
        Eigen::MatrixX4d hPoly;
        bool overlap;

        //贪心跳跃
        std::deque<int> idices;
        idices.push_front(M - 1);
        for (int i = M - 1; i >= 0; i--)
        {
            for (int j = 0; j < i; j++)
            {
                // 检查区域 i 和区域 j 是否重叠
                if (j < i - 1)
                {
                    // geo_utils::overlap 检查两个凸多面体是否有交集
                    overlap = geo_utils::overlap(htemp[i], htemp[j], 0.01);
                }
                else
                {
                    overlap = true; // 相邻区域强制认为重叠
                }
                if (overlap)
                {
                    idices.push_front(j); // 记录跳跃点
                    i = j + 1; // 跳跃到 j+1
                    break;
                }
            }
        }
        for (const auto &ele : idices)
        {
            hpolys.push_back(htemp[ele]);
        }
    }

}

#endif
