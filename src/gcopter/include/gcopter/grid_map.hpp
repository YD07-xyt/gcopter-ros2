#pragma once

#include "voxel_dilater.hpp"
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/src/Core/Matrix.h>
#include <cfloat>
#include <cstdint>
#include <memory>
#include <pcl-1.12/pcl/impl/point_types.hpp>
#include <pcl-1.12/pcl/point_cloud.h>
#include <queue>
#include <sys/types.h>
#include <vector>
#define logit(x) (log((x) / (1 - (x))))
namespace grid_map {

constexpr uint8_t Unoccupied = 0;
constexpr uint8_t Occupied = 1;
constexpr uint8_t Unknown = 3;
constexpr uint8_t Dilated = 2; // 膨胀后的障碍物

struct voxelState {
  uint8_t state;
  float height = 0.0f;
  float slope = 0.0f;
  voxelState() : state(Unoccupied) {}
};
struct GridMapCofig {
  double p_hit, p_miss, p_min, p_max, p_occ;
  Eigen::Vector3d origin;
  Eigen::Vector3i size;
  double voxScale; //分辨率
  double detection_range;
  double global_x_upper_ = -DBL_MAX, global_y_upper_ = -DBL_MAX;
  double global_x_lower_ = DBL_MAX, global_y_lower_ = DBL_MAX;
};
class GridMap {

public:
  GridMap() = default;
  GridMap(const GridMapCofig &map_config) {
    params_ = map_config;
    // init map
    GLX_SIZE_ = ceil((params_.global_x_upper_ - params_.global_x_lower_) /
                     params_.voxScale);
    GLY_SIZE_ = ceil((params_.global_y_upper_ - params_.global_y_lower_) /
                     params_.voxScale);
    GLXY_SIZE_ = GLX_SIZE_ * GLY_SIZE_;
    gridmap_ = new uint8_t[GLXY_SIZE_];
    memset(gridmap_, Unknown, GLXY_SIZE_ * sizeof(uint8_t));

    lcoal_map_size_.x() = ceil(params_.detection_range / params_.voxScale) * 2;
    lcoal_map_size_.y() = ceil(params_.detection_range / params_.voxScale) * 2;

    // Occupancy grid map
    double p_hit, p_miss, p_min, p_max, p_occ;
    p_hit = map_config.p_hit;
    p_miss = map_config.p_miss;
    p_min = map_config.p_min;
    p_max = map_config.p_max;
    p_occ = map_config.p_occ;

    prob_hit_log_ = logit(p_hit);
    prob_miss_log_ = logit(p_miss);
    clamp_min_log_ = logit(p_min);
    clamp_max_log_ = logit(p_max);
    min_occupancy_log_ = logit(p_occ);
    unknown_flag_ = 0.01;
    occupancy_map_ =
        std::vector<double>(GLXY_SIZE_, clamp_min_log_ - unknown_flag_);

    count_hit_ = std::vector<short>(GLXY_SIZE_, 0);
    count_hit_and_miss_ = std::vector<short>(GLXY_SIZE_, 0);
  }
  inline void update_odom(Eigen::Vector3d odom){
    this->odom_pos_=odom;
  }
  inline void update_cloud(pcl::PointCloud<pcl::PointXYZ> cloud){
    this->cloud_=cloud;
  }
  ~GridMap() {
    delete[] gridmap_;
    gridmap_ = nullptr;
  }

private:
  GridMapCofig params_;
  double local_x_upper_ = -DBL_MAX, local_y_upper_ = -DBL_MAX;
  double local_x_lower_ = DBL_MAX, local_y_lower_ = DBL_MAX;
  std::vector<double> occupancy_map_;
  std::vector<short> count_hit_, count_hit_and_miss_;
  std::queue<Eigen::Vector2i> cache_voxel_;
  double prob_hit_log_, prob_miss_log_;
  double clamp_max_log_, clamp_min_log_;
  double unknown_flag_;
  double min_occupancy_log_;
  Eigen::Vector3d odom_pos_;
  pcl::PointCloud<pcl::PointXYZ> cloud_;
  Eigen::Vector2d odom_pos_xy_;
  uint8_t *gridmap_ = nullptr;
  int GLX_SIZE_, GLY_SIZE_;
  int GLXY_SIZE_;
  double inv_grid_interval_ = 1 / params_.voxScale;
  Eigen::Vector2i lcoal_map_size_;
  bool occ_need_update_=false;

public:
  inline void update() {
    if(!occ_need_update_) 
    {
        return;
    }
    raycastProcess();

    RemoveOutliers();

    // Update the map based on occupancy_map_
    Eigen::Vector2i min_id, max_id;
    min_id = coord2gridIndex(Eigen::Vector2d(local_x_lower_, local_y_lower_));
    max_id = coord2gridIndex(Eigen::Vector2d(local_x_upper_, local_y_upper_));

    // Will not treat obstacles as free space!!
    for (int x = min_id.x(); x <= max_id.x(); x++) {
      for (int y = min_id.y(); y <= max_id.y(); y++) {
        int vecIndex = Index2Vectornum(x, y);
        if (gridmap_[vecIndex] == Unknown &&
            occupancy_map_[vecIndex] >= clamp_min_log_ &&
            occupancy_map_[vecIndex] <= min_occupancy_log_) {
          gridmap_[vecIndex] = Unoccupied;
        } else if (occupancy_map_[vecIndex] > min_occupancy_log_) {
          gridmap_[vecIndex] = Occupied;
        }
      }
    }
    occ_need_update_ = false;

  }

  inline void init_local_map() {
    local_x_upper_ = std::max(
        odom_pos_.x() -
            ceil(params_.detection_range / params_.voxScale) * params_.voxScale,
        params_.global_x_lower_);
    local_x_upper_ = std::min(
        odom_pos_.x() +
            ceil(params_.detection_range / params_.voxScale) * params_.voxScale,
        params_.global_x_upper_);
    local_y_lower_ = std::max(
        odom_pos_.y() -
            ceil(params_.detection_range / params_.voxScale) * params_.voxScale,
        params_.global_y_lower_);
    local_y_upper_ = std::min(
        odom_pos_.y() +
            ceil(params_.detection_range / params_.voxScale) * params_.voxScale,
        params_.global_y_upper_);

    lcoal_map_size_.x() =
        ceil((local_x_upper_ - local_x_lower_) / params_.voxScale);
    lcoal_map_size_.y() =
        ceil((local_y_upper_ - local_y_lower_) / params_.voxScale);
  };
  inline void raycastProcess() {
    int points_cnt = cloud_.points.size();

    odom_pos_xy_ = odom_pos_.head(2);
    Eigen::Vector2d cur_point;
    int vox_idx;
    double length;

    Eigen::Vector3d ray_pt;
    Eigen::Vector2d half = Eigen::Vector2d(0.5, 0.5);

    for (int i = 0; i < points_cnt; ++i) {
      cur_point << cloud_.points[i].x, cloud_.points[i].y;
      if (!isInGloMap(cur_point)) {
        cur_point = closetPointInMap(cur_point, odom_pos_xy_);
        //length = (cur_point - odom_pos_xy_).norm();
        length = this->calc_length(cur_point, odom_pos_xy_);
        if (length > params_.detection_range) {
          cur_point =
              (cur_point - odom_pos_xy_) / length * params_.detection_range +
              odom_pos_xy_;
        }
        vox_idx = setCacheOccupancy(cur_point, 0);
      } else {
        length = this->calc_length(cur_point, odom_pos_xy_);
        if (length > params_.detection_range) {
          cur_point =
              (cur_point - odom_pos_xy_) / length * params_.detection_range +
              odom_pos_xy_;
          vox_idx = setCacheOccupancy(cur_point, 0);
        } else {
          vox_idx = setCacheOccupancy(cur_point, 1);
        }
      }
      std::vector<Eigen::Vector2i> line = getGridsBetweenPoints2D(
          coord2gridIndex(odom_pos_xy_), coord2gridIndex(cur_point));

      int size = line.size() - 1;

      for (int i = 0; i < size; i++) {
        vox_idx = setCacheOccupancy(line[i], 0);
      }
    }

    updateOccupancyMap();
  }

  inline void updateOccupancyMap() {
    Eigen::Vector2i min_id, max_id;
    min_id = coord2gridIndex(Eigen::Vector2d(local_x_lower_, local_y_lower_));
    max_id = coord2gridIndex(Eigen::Vector2d(local_x_upper_, local_y_upper_));

    while (!cache_voxel_.empty()) {
      Eigen::Vector2i idx = cache_voxel_.front();
      int idx_ctns = Index2Vectornum(idx);
      cache_voxel_.pop();

      double log_odds_update =
          count_hit_[idx_ctns] >=
                  count_hit_and_miss_[idx_ctns] - 3 * count_hit_[idx_ctns]
              ? prob_hit_log_
              : prob_miss_log_;

      count_hit_[idx_ctns] = count_hit_and_miss_[idx_ctns] = 0;

      if (log_odds_update >= 0 && occupancy_map_[idx_ctns] >= clamp_max_log_) {
        continue;
      } else if (log_odds_update <= 0 &&
                 occupancy_map_[idx_ctns] <= clamp_min_log_) {
        occupancy_map_[idx_ctns] = clamp_min_log_;
        continue;
      }

      bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) &&
                      idx(1) >= min_id(1) && idx(1) <= max_id(1);
      if (!in_local) {
        occupancy_map_[idx_ctns] = clamp_min_log_;
      }

      occupancy_map_[idx_ctns] = std::min(
          std::max(occupancy_map_[idx_ctns] + log_odds_update, clamp_min_log_),
          clamp_max_log_);
    }
  }

  inline void RemoveOutliers() {
    std::vector<Eigen::Vector2d> cir_points;
    for (double x = odom_pos_.x() - params_.detection_range;
         x < odom_pos_.x() + params_.detection_range + 1e-10;
         x += params_.voxScale) {
      for (double y = odom_pos_.y() - params_.detection_range;
           y < odom_pos_.y() + params_.detection_range + 1e-10;
           y += params_.voxScale) {
        cir_points.emplace_back(x, y);
      }
    }

    double xlow = params_.global_x_lower_ + params_.voxScale;
    double xup = params_.global_x_upper_ - params_.voxScale;
    double ylow = params_.global_y_lower_ + params_.voxScale;
    double yup = params_.global_y_upper_ - params_.voxScale;
    for (auto cir_point : cir_points) {
      if (cir_point.x() > xlow && cir_point.x() < xup && cir_point.y() > ylow &&
          cir_point.y() < yup) {
        if (gridmap_[Index2Vectornum(coord2gridIndex(cir_point))] == Unknown) {
          if (gridmap_[Index2Vectornum(coord2gridIndex(cir_point)) + 1] ==
                  Unoccupied &&
              gridmap_[Index2Vectornum(coord2gridIndex(cir_point)) - 1] ==
                  Unoccupied &&
              gridmap_[Index2Vectornum(coord2gridIndex(cir_point)) +
                       GLY_SIZE_] == Unoccupied &&
              gridmap_[Index2Vectornum(coord2gridIndex(cir_point)) -
                       GLY_SIZE_] == Unoccupied) {
            gridmap_[Index2Vectornum(coord2gridIndex(cir_point))] = Unoccupied;
          }
        }
      }
    }
    Eigen::Vector2i idx =
        coord2gridIndex(Eigen::Vector2d(odom_pos_.x(), odom_pos_.y()));
    for (int i = -1; i <= 1; i++) {
      for (int j = -1; j <= 1; j++) {
        if (gridmap_[Index2Vectornum(idx.x() + i, idx.y() + j)] == Unknown) {
          gridmap_[Index2Vectornum(idx.x() + i, idx.y() + j)] = Unoccupied;
        }
      }
    }
  }
  inline Eigen::Vector2d closetPointInMap(const Eigen::Vector2d &pt,
                                          const Eigen::Vector2d &pos) {
    Eigen::Vector2d diff = pt - pos;
    Eigen::Vector2d max_tc =
        Eigen::Vector2d(params_.global_x_upper_, params_.global_y_upper_) - pos;
    Eigen::Vector2d min_tc =
        Eigen::Vector2d(params_.global_x_lower_, params_.global_y_lower_) - pos;
    // TODO: 数值优化
    double min_t = 1000000;

    for (int i = 0; i < 2; ++i) {
      if (fabs(diff[i]) > 0) {

        double t1 = max_tc[i] / diff[i];
        if (t1 > 0 && t1 < min_t)
          min_t = t1;

        double t2 = min_tc[i] / diff[i];
        if (t2 > 0 && t2 < min_t)
          min_t = t2;
      }
    }

    return pos + (min_t - 1e-3) * diff;
  }
  inline int setCacheOccupancy(Eigen::Vector2d pos, int occ) {
    if (occ != 1 && occ != 0)
      return -1;

    Eigen::Vector2i idx = coord2gridIndex(pos);
    int idx_ctns = Index2Vectornum(idx);

    count_hit_and_miss_[idx_ctns] += 1;

    if (count_hit_and_miss_[idx_ctns] == 1) {
      cache_voxel_.push(idx);
    }

    if (occ == 1)
      count_hit_[idx_ctns] += 1;

    return idx_ctns;
  }

  inline int setCacheOccupancy(Eigen::Vector2i idx, int occ) {
    if (occ != 1 && occ != 0)
      return -1;

    int idx_ctns = Index2Vectornum(idx);

    count_hit_and_miss_[idx_ctns] += 1;

    if (count_hit_and_miss_[idx_ctns] == 1) {
      cache_voxel_.push(idx);
    }

    if (occ == 1)
      count_hit_[idx_ctns] += 1;

    return idx_ctns;
  }
  Eigen::Vector2i coord2gridIndex(const Eigen::Vector2d &pt) {
    Eigen::Vector2i idx;
    idx << std::min(
        std::max(int((pt(0) - params_.global_x_lower_) * inv_grid_interval_),
                 0),
        GLX_SIZE_ - 1),
        std::min(
            std::max(
                int((pt(1) - params_.global_y_lower_) * inv_grid_interval_), 0),
            GLY_SIZE_ - 1);
    return idx;
  }

  inline bool isInGloMap(const Eigen::Vector2d &pt) {
    return pt.x() < params_.global_x_upper_ &&
           pt.x() > params_.global_x_lower_ &&
           pt.y() < params_.global_y_upper_ && pt.y() > params_.global_y_lower_;
  }

  inline double calc_length3d(Eigen::Vector3d start, Eigen::Vector3d end) {
    return (end - start).norm();
  };
  template <typename PointT>
  inline double calc_length(PointT start, PointT end) {
    return (end - start).norm();
  };
  inline Eigen::Vector2d closetPointIn2dMap(const Eigen::Vector2d &pt,
                                            const Eigen::Vector2d &pos) {
    Eigen::Vector2d diff = pt - pos;
    Eigen::Vector2d max_tc =
        Eigen::Vector2d(params_.global_x_upper_, params_.global_y_upper_) - pos;
    Eigen::Vector2d min_tc =
        Eigen::Vector2d(params_.global_x_lower_, params_.global_x_upper_) - pos;

    double min_t = 1000000;

    for (int i = 0; i < 2; ++i) {
      if (fabs(diff[i]) > 0) {

        double t1 = max_tc[i] / diff[i];
        if (t1 > 0 && t1 < min_t)
          min_t = t1;

        double t2 = min_tc[i] / diff[i];
        if (t2 > 0 && t2 < min_t)
          min_t = t2;
      }
    }

    return pos + (min_t - 1e-3) * diff;
  }
  int Index2Vectornum(const int &x, const int &y) { return x * GLY_SIZE_ + y; }

  int Index2Vectornum(const Eigen::Vector2i &index) {
    return index.x() * GLY_SIZE_ + index.y();
  }
  inline std::vector<Eigen::Vector2i>
  getGridsBetweenPoints2D(const Eigen::Vector2i &start,
                          const Eigen::Vector2i &end) {
    std::vector<Eigen::Vector2i> line;

    int dx = abs(end.x() - start.x());
    int dy = abs(end.y() - start.y());
    int sx = (start.x() < end.x()) ? 1 : -1;
    int sy = (start.y() < end.y()) ? 1 : -1;
    int err = dx - dy;

    double x0 = start.x();
    double y0 = start.y();

    while (true) {
      line.emplace_back(x0, y0);
      if (x0 == end.x() && y0 == end.y()) {
        break;
      }
      int e2 = 2 * err;
      if (e2 > -dy) {
        err -= dy;
        x0 += sx;
      }
      if (e2 < dx) {
        err += dx;
        y0 += sy;
      }
    }
    return line;
  }
};
} // namespace grid_map
