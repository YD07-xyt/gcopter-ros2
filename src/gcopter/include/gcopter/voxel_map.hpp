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
#pragma once

#include "voxel_dilater.hpp"
#include <cstdint>
#include <memory>
#include <vector>
#include <Eigen/Eigen>

namespace voxel_map
{

    constexpr uint8_t Unoccupied = 0;
    constexpr uint8_t Occupied = 1;
    constexpr uint8_t Unkown =3;
    constexpr uint8_t Dilated = 2; // 膨胀后的障碍物

    struct voxelState
    {
        uint8_t state;
        float height = 0.0f;
        float slope = 0.0f;
        voxelState() : state(Unoccupied) {}
    };

    class VoxelMap
    {

    public:
        VoxelMap() = default;
        VoxelMap(const Eigen::Vector3i &size,
                 const Eigen::Vector3d &origin,
                 const double &voxScale)
            : mapSize(size),
              origin(origin),//地图原点
              scale(voxScale),//每个体素的边长（分辨率）
              voxNum(mapSize.prod()),//总体素数
              step(1, mapSize(0), mapSize(1) * mapSize(0)),//索引步长 (1, sizeX, sizeX*sizeY)，用于将 3D 索引展平为 1D
              oc(origin + Eigen::Vector3d::Constant(0.5 * scale)),//体素中心偏移
              bounds((mapSize.array() - 1) * step.array()),//最大合法索引边界
              stepScale(step.cast<double>().cwiseInverse() * scale),//从体素索引到世界坐标的转换系数
              voxels(voxNum, Unoccupied) {}//体素状态数组（展平的一维存储）

    private:
        Eigen::Vector3i mapSize;
        Eigen::Vector3d origin;//地图原点
        double scale;//每个体素的边长（分辨率）
        int voxNum;//总体素数
        Eigen::Vector3i step;//索引步长 (1, sizeX, sizeX*sizeY)，用于将 3D 索引展平为 1D
        Eigen::Vector3d oc;//体素中心偏移
        Eigen::Vector3i bounds;//最大合法索引边界
        Eigen::Vector3d stepScale;//从体素索引到世界坐标的转换系数
        std::vector<uint8_t> voxels;//体素状态数组（展平的一维存储）
        std::vector<Eigen::Vector3i> surf; //膨胀后的表面体素列表

    public:
        inline Eigen::Vector3i getSize(void) const
        {
            return mapSize;
        }

        inline double getScale(void) const
        {
            return scale;
        }

        inline Eigen::Vector3d getOrigin(void) const
        {
            return origin;
        }

        inline Eigen::Vector3d getCorner(void) const
        {
            return mapSize.cast<double>() * scale + origin;
        }

        inline const std::vector<uint8_t> &getVoxels(void) const
        {
            return voxels;
        }

        inline void setOccupied(const Eigen::Vector3d &pos)
        {
            const Eigen::Vector3i id = ((pos - origin) / scale).cast<int>();
            if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 &&
                id(0) < mapSize(0) && id(1) < mapSize(1) && id(2) < mapSize(2))
            {
                voxels[id.dot(step)] = Occupied;
            }
        }

        inline void setOccupied(const Eigen::Vector3i &id)
        {
            if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 &&
                id(0) < mapSize(0) && id(1) < mapSize(1) && id(2) < mapSize(2))
            {
                voxels[id.dot(step)] = Occupied;
            }
        }
        //形态学膨胀（Dilation），将障碍物向外扩展 r 个体素
        inline void dilate(const int &r)
        {
            if (r <= 0)
            {
                return;
            }
            else
            {
                std::vector<Eigen::Vector3i> lvec, cvec;
                lvec.reserve(voxNum);
                cvec.reserve(voxNum);
                int i, j, k, idx;
                bool check;
                for (int x = 0; x <= bounds(0); x++)
                {
                    for (int y = 0; y <= bounds(1); y += step(1))
                    {
                        for (int z = 0; z <= bounds(2); z += step(2))
                        {
                            if (voxels[x + y + z] == Occupied)
                            {
                                VOXEL_DILATER(i, j, k,
                                              x, y, z,
                                              step(1), step(2),
                                              bounds(0), bounds(1), bounds(2),
                                              check, voxels, idx, Dilated, cvec)
                            }
                        }
                    }
                }

                for (int loop = 1; loop < r; loop++)
                {
                    std::swap(cvec, lvec);
                    for (const Eigen::Vector3i &id : lvec)
                    {
                        VOXEL_DILATER(i, j, k,
                                      id(0), id(1), id(2),
                                      step(1), step(2),
                                      bounds(0), bounds(1), bounds(2),
                                      check, voxels, idx, Dilated, cvec)
                    }
                    lvec.clear();
                }

                surf = cvec;
            }
        }
        //获取中心点附近 halfWidth 范围内的膨胀表面点
        inline void getSurfInBox(const Eigen::Vector3i &center,
                                 const int &halfWidth,
                                 std::vector<Eigen::Vector3d> &points) const
        {
            for (const Eigen::Vector3i &id : surf)
            {
                if (std::abs(id(0) - center(0)) <= halfWidth &&
                    std::abs(id(1) / step(1) - center(1)) <= halfWidth &&
                    std::abs(id(2) / step(2) - center(2)) <= halfWidth)
                {
                    points.push_back(id.cast<double>().cwiseProduct(stepScale) + oc);
                }
            }

            return;
        }
        //获取全部膨胀表面点的世界坐标
        inline void getSurf(std::vector<Eigen::Vector3d> &points) const
        {
            points.reserve(surf.size());
            for (const Eigen::Vector3i &id : surf)
            {
                points.push_back(id.cast<double>().cwiseProduct(stepScale) + oc);
            }
            return;
        }
        //碰撞查询
        inline bool query(const Eigen::Vector3d &pos) const
        {
            const Eigen::Vector3i id = ((pos - origin) / scale).cast<int>();
            if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 &&
                id(0) < mapSize(0) && id(1) < mapSize(1) && id(2) < mapSize(2))
            {
                return voxels[id.dot(step)];
            }
            else
            {
                return true;
            }
        }
        //碰撞查询
        inline bool query(const Eigen::Vector3i &id) const
        {
            if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 &&
                id(0) < mapSize(0) && id(1) < mapSize(1) && id(2) < mapSize(2))
            {
                return voxels[id.dot(step)];
            }
            else
            {
                return true;
            }
        }
        //世界坐标 → 体素索引（向下取整）
        inline Eigen::Vector3d posI2D(const Eigen::Vector3i &id) const
        {
            return id.cast<double>() * scale + oc;
        }
        //体素索引 → 世界坐标（返回体素中心）
        inline Eigen::Vector3i posD2I(const Eigen::Vector3d &pos) const
        {
            return ((pos - origin) / scale).cast<int>();
        }
    };
}

