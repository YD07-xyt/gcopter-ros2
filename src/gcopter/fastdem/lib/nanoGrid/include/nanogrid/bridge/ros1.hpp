// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#ifndef NANOGRID_BRIDGE_ROS1_HPP
#define NANOGRID_BRIDGE_ROS1_HPP

#include <grid_map_msgs/GridMap.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h>

#include <nanogrid/bridge/ros/impl.hpp>

namespace nanogrid::ros1 {

namespace detail {
inline ros::Time toStamp(uint64_t ns) {
  ros::Time t;
  t.fromNSec(ns);
  return t;
}
inline uint64_t fromStamp(const ros::Time& t) { return t.toNSec(); }
}  // namespace detail

/// Convert nanogrid::GridMap to grid_map_msgs::GridMap.
/// If layers is empty (default), all layers are serialized.
inline grid_map_msgs::GridMap toMsg(
    const GridMap& map, const std::vector<std::string>& layers = {}) {
  grid_map_msgs::GridMap msg;
  msg.info.header.stamp = detail::toStamp(map.getTimestamp());
  msg.info.header.frame_id = map.getFrameId();
  nanogrid::detail::fillGridMapMsg<grid_map_msgs::GridMap,
                                   std_msgs::Float32MultiArray>(
      msg, map, layers.empty() ? map.getLayers() : layers);
  return msg;
}

/// Convert grid_map_msgs::GridMap to nanogrid::GridMap.
inline GridMap fromMsg(const grid_map_msgs::GridMap& msg) {
  GridMap map =
      nanogrid::detail::fromGridMapMsg<grid_map_msgs::GridMap,
                                       std_msgs::Float32MultiArray>(msg);
  map.setTimestamp(detail::fromStamp(msg.info.header.stamp));
  map.setFrameId(msg.info.header.frame_id);
  return map;
}

/// Create map boundary marker for RViz visualization.
inline visualization_msgs::Marker toBoundaryMarker(const GridMap& map) {
  return nanogrid::detail::toMapBoundaryImpl<visualization_msgs::Marker,
                                             geometry_msgs::Point>(
      map, detail::toStamp(map.getTimestamp()));
}

}  // namespace nanogrid::ros1

#endif  // NANOGRID_BRIDGE_ROS1_HPP
