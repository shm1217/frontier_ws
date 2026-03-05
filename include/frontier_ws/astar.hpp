#pragma once
#include "frontier_explorer/common.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>

namespace fe {

std::vector<GridPose> astar(const nav_msgs::msg::OccupancyGrid& map,
                            const GridPose& start,
                            const GridPose& goal,
                            const std::vector<uint8_t>& obsMaskInflated);

std::vector<uint8_t> buildReachableMaskFromStart(const nav_msgs::msg::OccupancyGrid& map,
                                                 const GridPose& start,
                                                 const std::vector<uint8_t>& blockedMask);

void filterFrontiersByReachable(const nav_msgs::msg::OccupancyGrid& map,
                                std::vector<GridPose>& frontiers,
                                const std::vector<uint8_t>& reachable);

bool pathTailTooCloseToRawObstacle(const nav_msgs::msg::OccupancyGrid& map,
                                   const std::vector<GridPose>& p,
                                   const std::vector<uint8_t>& obsMaskRaw,
                                   double path_clearance_m,
                                   int obstacle_threshold);

} // namespace fe
