#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "frontier_explorer/common.hpp"

namespace fe {

class FrontierExplorerAStar : public rclcpp::Node {
public:
  FrontierExplorerAStar();

private:
  // ROS
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr frontier_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr infl_marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Params
  std::string map_topic_, cmd_topic_, scan_topic_;
  std::string map_frame_, base_frame_;
  double tf_timeout_s_{0.1};

  int obstacle_threshold_{70};
  int free_threshold_{60};
  double inflation_radius_m_{0.25};
  double frontier_search_radius_m_{8.0};

  double lookahead_m_{0.35};
  double goal_tol_m_{0.35};
  double max_lin_vel_{0.25};
  double max_ang_vel_{0.90};

  double laser_inflation_radius_m_{0.12};
  double laser_block_ttl_{1.0};

  double frontier_clearance_m_{0.30};
  double path_clearance_m_{0.12};

  double infl_viz_radius_m_{3.0};

  double stuck_time_s_{2.0};
  double stuck_dist_m_{0.05};

  bool use_dbscan_{true};
  double dbscan_eps_m_{0.20};
  int dbscan_min_pts_{6};

  bool warmup_enable_{true};
  int warmup_min_known_cells_{12000};
  int warmup_min_occ_cells_{50};
  double warmup_radius_m_{2.0};
  double warmup_yaw_rate_{0.6};
  double warmup_lin_vel_{0.0};
  double warmup_max_time_s_{6.0};

  double avoid_dist_{0.45};
  double avoid_end_dist_{0.60};

  // State
  nav_msgs::msg::OccupancyGrid map_;
  bool has_map_{false};

  sensor_msgs::msg::LaserScan last_scan_;
  bool has_scan_{false};

  std::vector<uint8_t> laser_blocked_;
  std::chrono::steady_clock::time_point last_laser_update_{};

  WorldPose robot_{};
  bool has_pose_{false};

  std::vector<GridPose> path_;
  int wp_idx_{0};
  double reach_dist_{0.12};

  bool avoiding_{false};
  bool warmup_done_{false};
  rclcpp::Time warmup_state_start_;

  rclcpp::Time last_progress_time_;
  double last_progress_x_{0.0}, last_progress_y_{0.0};
  int last_progress_wp_{0};
  bool done_{false};

  // callbacks / helpers
  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void onTimer();

  bool updateRobotPoseFromTF();

  double minRange(double a_min, double a_max);
  void publishStop();
  void publishAvoidCmd();
  void publishWarmupMotion();

  // map processing
  bool isTraversable(int x, int y) const;
  bool isFrontierCell(int x, int y) const;
  std::vector<GridPose> detectFrontiers(const GridPose& robot_g) const;

  std::vector<uint8_t> buildInflatedBlockedMask() const;
  std::vector<uint8_t> buildObstacleInflatedMask() const;
  std::vector<uint8_t> buildObstacleRawMask() const;

  bool isFrontierTooCloseToObstacle(const GridPose& f,
                                    const std::vector<uint8_t>& obsMaskInflated,
                                    int radius_cells) const;

  int findNearestIndexOnPath(const std::vector<GridPose>& path, int start_idx, int window);
  void followPathStep();

  struct LocalMapStats { int known{0}; int occ{0}; int minv{101}; int maxv{-1}; };
  LocalMapStats computeLocalMapStats(const GridPose& center_g, double radius_m) const;

  bool pickNearestFrontier(const GridPose &robot_g,
                           const std::vector<GridPose> &frontiers,
                           const std::vector<uint8_t> &blockedMask,
                           const std::vector<uint8_t> &obsMaskInflated,
                           const std::vector<uint8_t> &obsMaskRaw,
                           GridPose &out_goal,
                           std::vector<GridPose> &out_path) const;
};

} // namespace fe
