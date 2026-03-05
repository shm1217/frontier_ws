#pragma once
#include "frontier_explorer/common.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace fe {

inline bool inBounds(const nav_msgs::msg::OccupancyGrid& map, int x, int y) {
  return (0 <= x && x < (int)map.info.width && 0 <= y && y < (int)map.info.height);
}

inline GridPose worldToGrid(const nav_msgs::msg::OccupancyGrid& map, double wx, double wy) {
  const auto& info = map.info;
  int gx = (int)std::floor((wx - info.origin.position.x) / info.resolution);
  int gy = (int)std::floor((wy - info.origin.position.y) / info.resolution);
  return {gx, gy};
}

inline std::pair<double,double> gridToWorld(const nav_msgs::msg::OccupancyGrid& map, int gx, int gy) {
  const auto& info = map.info;
  double wx = info.origin.position.x + (gx + 0.5) * info.resolution;
  double wy = info.origin.position.y + (gy + 0.5) * info.resolution;
  return {wx, wy};
}

} // namespace fe
