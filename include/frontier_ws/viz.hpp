#pragma once
#include "frontier_explorer/common.hpp"
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace fe {

void publishPathMarker(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                       const rclcpp::Time& stamp,
                       const std::string& frame_id,
                       const nav_msgs::msg::OccupancyGrid& map,
                       const std::vector<GridPose>& path);

void publishFrontierMarkers(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                            const rclcpp::Time& stamp,
                            const std::string& frame_id,
                            const nav_msgs::msg::OccupancyGrid& map,
                            const std::vector<GridPose>& frontiers);

void publishInflationMaskMarker(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                                const rclcpp::Time& stamp,
                                const std::string& frame_id,
                                const nav_msgs::msg::OccupancyGrid& map,
                                const std::vector<uint8_t>& inflated_mask,
                                const GridPose& center_g,
                                double infl_viz_radius_m);

} // namespace fe
