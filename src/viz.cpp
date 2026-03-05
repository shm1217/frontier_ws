#include "frontier_explorer/viz.hpp"
#include "frontier_explorer/grid_utils.hpp"

namespace fe {

void publishPathMarker(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                       const rclcpp::Time& stamp,
                       const std::string& frame_id,
                       const nav_msgs::msg::OccupancyGrid& map,
                       const std::vector<GridPose>& path) {
  if (!pub || path.empty()) return;

  visualization_msgs::msg::Marker m;
  m.header.stamp = stamp;
  m.header.frame_id = frame_id;
  m.ns = "planned_path";
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::LINE_STRIP;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.scale.x = 0.03;
  m.color.r = 0.0f; m.color.g = 1.0f; m.color.b = 0.0f; m.color.a = 1.0f;

  m.points.reserve(path.size());
  for (const auto& gp : path) {
    auto [wx, wy] = gridToWorld(map, gp.x, gp.y);
    geometry_msgs::msg::Point pt;
    pt.x = wx; pt.y = wy; pt.z = 0.05;
    m.points.push_back(pt);
  }
  pub->publish(m);
}

void publishFrontierMarkers(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                            const rclcpp::Time& stamp,
                            const std::string& frame_id,
                            const nav_msgs::msg::OccupancyGrid& map,
                            const std::vector<GridPose>& frontiers) {
  if (!pub) return;

  visualization_msgs::msg::Marker m;
  m.header.stamp = stamp;
  m.header.frame_id = frame_id;
  m.ns = "frontiers";
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::POINTS;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.scale.x = 0.05;
  m.scale.y = 0.05;
  m.color.r = 0.0f; m.color.g = 0.1f; m.color.b = 0.5f; m.color.a = 1.0f;

  m.points.reserve(frontiers.size());
  for (const auto& f : frontiers) {
    auto [wx, wy] = gridToWorld(map, f.x, f.y);
    geometry_msgs::msg::Point p;
    p.x = wx; p.y = wy; p.z = 0.05;
    m.points.push_back(p);
  }
  pub->publish(m);
}

void publishInflationMaskMarker(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                                const rclcpp::Time& stamp,
                                const std::string& frame_id,
                                const nav_msgs::msg::OccupancyGrid& map,
                                const std::vector<uint8_t>& inflated_mask,
                                const GridPose& center_g,
                                double infl_viz_radius_m) {
  if (!pub || inflated_mask.empty()) return;

  const int W = (int)map.info.width;
  const double res = map.info.resolution;

  int r_cells = (int)std::ceil(infl_viz_radius_m / res);
  int x0 = std::max(0, center_g.x - r_cells);
  int x1 = std::min(W - 1, center_g.x + r_cells);
  int y0 = std::max(0, center_g.y - r_cells);
  int y1 = std::min((int)map.info.height - 1, center_g.y + r_cells);

  visualization_msgs::msg::Marker m;
  m.header.stamp = stamp;
  m.header.frame_id = frame_id;
  m.ns = "inflation_mask";
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::CUBE_LIST;
  m.action = visualization_msgs::msg::Marker::ADD;

  m.scale.x = res;
  m.scale.y = res;
  m.scale.z = 0.02;

  m.color.r = 0.5f; m.color.g = 0.0f; m.color.b = 0.5f; m.color.a = 0.8f;

  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      if (!inflated_mask[IDX(x,y,W)]) continue;
      auto [wx, wy] = gridToWorld(map, x, y);
      geometry_msgs::msg::Point p;
      p.x = wx; p.y = wy; p.z = 0.01;
      m.points.push_back(p);
    }
  }
  pub->publish(m);
}

} // namespace fe
