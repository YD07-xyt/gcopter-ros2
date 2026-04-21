// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#ifndef NANOGRID_BRIDGE_ROS2_HPP
#define NANOGRID_BRIDGE_ROS2_HPP

#include <grid_map_msgs/msg/grid_map.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <nanogrid/bridge/ros/impl.hpp>

namespace nanogrid::ros2 {

namespace detail {
inline builtin_interfaces::msg::Time toStamp(uint64_t ns) {
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(ns / 1000000000ULL);
  t.nanosec = static_cast<uint32_t>(ns % 1000000000ULL);
  return t;
}
inline uint64_t fromStamp(const builtin_interfaces::msg::Time& t) {
  return static_cast<uint64_t>(t.sec) * 1000000000ULL + t.nanosec;
}
}  // namespace detail

/// Convert nanogrid::GridMap to grid_map_msgs::msg::GridMap.
/// If layers is empty (default), all layers are serialized.
inline grid_map_msgs::msg::GridMap toMsg(
    const GridMap& map, const std::vector<std::string>& layers = {}) {
  grid_map_msgs::msg::GridMap msg;
  msg.header.stamp = detail::toStamp(map.getTimestamp());
  msg.header.frame_id = map.getFrameId();
  nanogrid::detail::fillGridMapMsg<grid_map_msgs::msg::GridMap,
                                   std_msgs::msg::Float32MultiArray>(
      msg, map, layers.empty() ? map.getLayers() : layers);
  return msg;
}

/// Convert grid_map_msgs::msg::GridMap to nanogrid::GridMap.
inline GridMap fromMsg(const grid_map_msgs::msg::GridMap& msg) {
  GridMap map =
      nanogrid::detail::fromGridMapMsg<grid_map_msgs::msg::GridMap,
                                       std_msgs::msg::Float32MultiArray>(msg);
  map.setTimestamp(detail::fromStamp(msg.header.stamp));
  map.setFrameId(msg.header.frame_id);
  return map;
}

/// Create map boundary marker for RViz visualization.
inline visualization_msgs::msg::Marker toBoundaryMarker(const GridMap& map) {
  return nanogrid::detail::toMapBoundaryImpl<visualization_msgs::msg::Marker,
                                             geometry_msgs::msg::Point>(
      map, detail::toStamp(map.getTimestamp()));
}

}  // namespace nanogrid::ros2

#endif  // NANOGRID_BRIDGE_ROS2_HPP
