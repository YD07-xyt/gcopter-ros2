// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#ifndef NANOGRID_BRIDGE_ROS_IMPL_HPP
#define NANOGRID_BRIDGE_ROS_IMPL_HPP

#include <nanogrid/GridMap.hpp>

#include <cstring>
#include <string>
#include <vector>

namespace nanogrid::detail {

// ── fillGridMapMsg ──────────────────────────────────────────────────────────
//
// Fills GridMap message geometry, layers, data, and start indices.
// Does NOT set header (caller handles it — ROS1: msg.info.header, ROS2: msg.header).
//
template <typename GridMapMsgT, typename Float32MultiArrayT>
void fillGridMapMsg(GridMapMsgT& msg, const GridMap& map,
                    const std::vector<std::string>& layers) {
  // Geometry
  msg.info.resolution = map.getResolution();
  msg.info.length_x = map.getLength().x();
  msg.info.length_y = map.getLength().y();
  msg.info.pose.position.x = map.getPosition().x();
  msg.info.pose.position.y = map.getPosition().y();
  msg.info.pose.position.z = 0.0;
  msg.info.pose.orientation.w = 1.0;
  msg.info.pose.orientation.x = 0.0;
  msg.info.pose.orientation.y = 0.0;
  msg.info.pose.orientation.z = 0.0;

  // Layers
  msg.layers = layers;

  // Data
  for (const auto& layer_name : msg.layers) {
    Float32MultiArrayT data_array;
    const auto& layer_data = map.get(layer_name);

    // Layout descriptor (grid_map convention)
    data_array.layout.dim.resize(2);
    data_array.layout.dim[0].label = "column_index";
    data_array.layout.dim[0].size = map.getSize()(0);
    data_array.layout.dim[0].stride = map.getSize()(0) * map.getSize()(1);
    data_array.layout.dim[1].label = "row_index";
    data_array.layout.dim[1].size = map.getSize()(1);
    data_array.layout.dim[1].stride = map.getSize()(1);

    // Copy data (Eigen column-major == grid_map message data order)
    data_array.data.resize(layer_data.size());
    std::memcpy(data_array.data.data(), layer_data.data(),
                layer_data.size() * sizeof(float));

    msg.data.push_back(std::move(data_array));
  }

  // Circular buffer start indices
  msg.outer_start_index = map.getStartIndex()(0);
  msg.inner_start_index = map.getStartIndex()(1);
}

// ── fromGridMapMsg ──────────────────────────────────────────────────────────
//
// Reconstructs GridMap from message geometry, layers, data, and start indices.
// Does NOT read header (caller handles it — ROS1: msg.info.header, ROS2: msg.header).
//
template <typename GridMapMsgT, typename Float32MultiArrayT>
GridMap fromGridMapMsg(const GridMapMsgT& msg) {
  // Geometry
  Length length;
  length.x() = msg.info.length_x;
  length.y() = msg.info.length_y;
  Position position;
  position.x() = msg.info.pose.position.x;
  position.y() = msg.info.pose.position.y;

  GridMap map;
  map.setGeometry(length, msg.info.resolution, position);

  // Layers + data
  const int rows = map.getSize()(0);
  const int cols = map.getSize()(1);

  for (size_t i = 0; i < msg.layers.size(); ++i) {
    const auto& layer_name = msg.layers[i];
    const auto& arr = msg.data[i];

    Matrix mat(rows, cols);
    std::memcpy(mat.data(), arr.data.data(), rows * cols * sizeof(float));

    map.add(layer_name, mat);
  }

  // Circular buffer start index
  Index startIndex;
  startIndex(0) = msg.outer_start_index;
  startIndex(1) = msg.inner_start_index;
  map.setStartIndex(startIndex);

  return map;
}

// ── toMapBoundaryImpl ───────────────────────────────────────────────────────
//
// Creates a LINE_STRIP marker showing the map boundary rectangle.
//
template <typename MarkerT, typename PointT, typename TimeT>
MarkerT toMapBoundaryImpl(const GridMap& map, const TimeT& stamp) {
  MarkerT marker;
  marker.header.stamp = stamp;
  marker.header.frame_id = map.getFrameId();
  marker.ns = "nanogrid";
  marker.id = 0;
  marker.type = MarkerT::LINE_STRIP;
  marker.action = MarkerT::ADD;
  marker.scale.x = 0.01;
  marker.color.r = 1.0f;
  marker.color.g = 1.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0f;
  marker.pose.orientation.w = 1.0;

  const double cx = map.getPosition().x();
  const double cy = map.getPosition().y();
  const double hx = map.getLength().x() / 2.0;
  const double hy = map.getLength().y() / 2.0;

  PointT p;
  p.z = 0.0;
  const double corners[][2] = {{cx - hx, cy - hy},
                                {cx + hx, cy - hy},
                                {cx + hx, cy + hy},
                                {cx - hx, cy + hy},
                                {cx - hx, cy - hy}};
  for (const auto& c : corners) {
    p.x = c[0];
    p.y = c[1];
    marker.points.push_back(p);
  }

  return marker;
}

}  // namespace nanogrid::detail

#endif  // NANOGRID_BRIDGE_ROS_IMPL_HPP
