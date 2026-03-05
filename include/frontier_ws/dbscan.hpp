#pragma once
#include "frontier_explorer/common.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>

namespace fe {

double distMeters(const nav_msgs::msg::OccupancyGrid& map, const GridPose& a, const GridPose& b);

std::vector<int> dbscanCluster(const nav_msgs::msg::OccupancyGrid& map,
                               const std::vector<GridPose>& pts,
                               double eps_m,
                               int min_pts);

std::vector<GridPose> computeClusterRepresentatives(const std::vector<GridPose>& pts,
                                                    const std::vector<int>& labels);

} // namespace fe
 