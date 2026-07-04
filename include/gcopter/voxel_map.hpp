/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to
   deal in the Software without restriction, including without limitation the
   rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
   sell copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
   FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
   IN THE SOFTWARE.
*/

#ifndef VOXEL_MAP_HPP
#define VOXEL_MAP_HPP

#include "voxel_dilater.hpp"
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <cstddef>
#include <memory>
#include <spdlog/spdlog.h>
#include <vector>
namespace voxel_map {

constexpr uint8_t Unoccupied = 0;
constexpr uint8_t Occupied = 1;
constexpr uint8_t Dilated = 2;

class VoxelMap {

public:
  VoxelMap() = default;
  VoxelMap(const Eigen::Vector3i &size, const Eigen::Vector3d &origin,
           const double &voxScale)
      : mapSize(size), o(origin), scale(voxScale), voxNum(mapSize.prod()),
        step(1, mapSize(0), mapSize(1) * mapSize(0)),
        oc(o + Eigen::Vector3d::Constant(0.5 * scale)),
        bounds((mapSize.array() - 1) * step.array()),
        stepScale(step.cast<double>().cwiseInverse() * scale),
        voxels(voxNum, Unoccupied) {
    spdlog::info("map origin x:{},y:{},z:{}", o.x(), o.y(), o.z());
    spdlog::info("map Size x:{},y:{},z:{}", mapSize.x(), mapSize.y(),
                 mapSize.z());
  }
private:
  Eigen::Vector3i mapSize; // 体素网格尺寸，如 (100, 100, 100)
  Eigen::Vector3d o;      // 地图原点（世界坐标系下的左下角点）
  double scale;           // 体素分辨率（每个格子边长，米）
  int voxNum;             // 总体素数量 = mapSize.x * mapSize.y * mapSize.z
  Eigen::Vector3i step;   // 索引步长，用于3D转1D索引
  Eigen::Vector3d oc;     // 体素中心偏移量
  Eigen::Vector3i bounds; // 边界索引
  Eigen::Vector3d stepScale;   // 用于坐标转换的缩放因子
  std::vector<uint8_t> voxels; // 体素数据（0:空闲 1:占用 2:膨胀）
  std::vector<Eigen::Vector3i> surf; // 膨胀后的表面体素列表

public:
  inline Eigen::Vector3i getSize(void) const { return mapSize; }

  inline double getScale(void) const { return scale; }

  inline Eigen::Vector3d getOrigin(void) const { return o; }
  //获取地图角点
  inline Eigen::Vector3d getCorner(void) const {
    // cast 类型转换 int->double
    return mapSize.cast<double>() * scale + o;
  }

  inline const std::vector<uint8_t> &getVoxels(void) const { return voxels; }

  inline void setOccupied(const Eigen::Vector3d &pos) {
    //const Eigen::Vector3i id = ((pos - o) / scale).cast<int>();
    const Eigen::Vector3i id = ((pos - o) / scale + Eigen::Vector3d::Constant(1e-6)).cast<int>();
    if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 && id(0) < mapSize(0) &&
        id(1) < mapSize(1) && id(2) < mapSize(2)) {
      voxels[id.dot(step)] = Occupied;
    }
  }
  inline void getOccupiedVoxels(std::vector<Eigen::Vector3d> &points) const {
    for (int x = 0; x < mapSize(0); ++x) {
      for (int y = 0; y < mapSize(1); ++y) {
        for (int z = 0; z < mapSize(2); ++z) {
          const int idx = x + y * step(1) + z * step(2);
          if (voxels[idx] == Occupied) {
            // 直接使用真实索引乘 scale 加中心偏移
            points.emplace_back(Eigen::Vector3d(x, y, z) * scale + oc);
          }
        }
      }
    }
  }

  inline void setOccupied(const Eigen::Vector3i &id) {
    if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 && id(0) < mapSize(0) &&
        id(1) < mapSize(1) && id(2) < mapSize(2)) {
      voxels[id.dot(step)] = Occupied;
    }
  }

  //膨胀地图
  inline void dilate(const int &r) {
    if (r <= 0) {
      return;
    } else {
      std::vector<Eigen::Vector3i> lvec, cvec;
      lvec.reserve(voxNum);
      cvec.reserve(voxNum);
      int i, j, k, idx;
      bool check;
      for (int x = 0; x <= bounds(0); x++) {
        for (int y = 0; y <= bounds(1); y += step(1)) {
          for (int z = 0; z <= bounds(2); z += step(2)) {
            if (voxels[x + y + z] == Occupied) {
              VOXEL_DILATER(i, j, k, x, y, z, step(1), step(2), bounds(0),
                            bounds(1), bounds(2), check, voxels, idx, Dilated,
                            cvec)
            }
          }
        }
      }

      for (int loop = 1; loop < r; loop++) {
        std::swap(cvec, lvec);
        for (const Eigen::Vector3i &id : lvec) {
          VOXEL_DILATER(i, j, k, id(0), id(1), id(2), step(1), step(2),
                        bounds(0), bounds(1), bounds(2), check, voxels, idx,
                        Dilated, cvec)
        }
        lvec.clear();
      }

      surf = cvec;
      // spdlog::info("voxelMap::dilate中 surf:{}", surf.size());
    }
  }

  // 获取指定立方体内的表面点
  inline void getSurfInBox(const Eigen::Vector3i &center, const int &halfWidth,
                           std::vector<Eigen::Vector3d> &points) const {
    for (const Eigen::Vector3i &id : surf) {
      if (std::abs(id(0) - center(0)) <= halfWidth &&
          std::abs(id(1) / step(1) - center(1)) <= halfWidth &&
          std::abs(id(2) / step(2) - center(2)) <= halfWidth) {
        points.push_back(id.cast<double>().cwiseProduct(stepScale) + oc);
      }
    }

    return;
  }
  inline void getDilatedVoxels(std::vector<Eigen::Vector3d> &points) const {
    for (int x = 0; x < mapSize(0); ++x) {
      for (int y = 0; y < mapSize(1); ++y) {
        for (int z = 0; z < mapSize(2); ++z) {
          const int idx = x + y * step(1) + z * step(2);
          if (voxels[idx] == Dilated) {
            points.emplace_back(Eigen::Vector3d(x, y, z) * scale + oc);
          }
        }
      }
    }
  }
  // 获取所有膨胀后的表面点
  inline void getSurf(std::vector<Eigen::Vector3d> &points) const {
    points.reserve(surf.size());
    for (const Eigen::Vector3i &id : surf) {
      // spdlog::info("add point to surf map ");
      points.push_back(id.cast<double>().cwiseProduct(stepScale) + oc);
    }
    return;
  }
  //查询
  inline bool query(const Eigen::Vector3d &pos) const {
    //const Eigen::Vector3i id = ((pos - o) / scale).cast<int>();
    const Eigen::Vector3i id = ((pos - o) / scale + Eigen::Vector3d::Constant(1e-6)).cast<int>();
    if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 && id(0) < mapSize(0) &&
        id(1) < mapSize(1) && id(2) < mapSize(2)) {
      return voxels[id.dot(step)];
    } else {
      return true;
    }
  }
  //查询
  inline bool query(const Eigen::Vector3i &id) const {
    if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 && id(0) < mapSize(0) &&
        id(1) < mapSize(1) && id(2) < mapSize(2)) {
      return voxels[id.dot(step)];
    } else {
      return true;
    }
  }
  // inline bool query(const Eigen::Vector2d &id) const {
  //   if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 && id(0) < mapSize(0) &&
  //       id(1) < mapSize(1) && id(2) < mapSize(2)) {
  //     return voxels[id.dot(step)];
  //   } else {
  //     return true;
  //   }
  // }
  inline Eigen::Vector3d posI2D(const Eigen::Vector3i &id) const {
    return id.cast<double>() * scale + oc;
  }

  inline Eigen::Vector3i posD2I(const Eigen::Vector3d &pos) const {
    return ((pos - o) / scale).cast<int>();
  }
};
} // namespace voxel_map

#endif
