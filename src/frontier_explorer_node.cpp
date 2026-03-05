#include "frontier_explorer/frontier_explorer_node.hpp"

#include "frontier_explorer/common.hpp"
#include "frontier_explorer/grid_utils.hpp"
#include "frontier_explorer/astar.hpp"
#include "frontier_explorer/dbscan.hpp"
#include "frontier_explorer/viz.hpp"

#include <tf2/utils.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <queue>
#include <algorithm>
#include <limits>
#include <string>

namespace fe {

FrontierExplorerAStar::FrontierExplorerAStar()
: Node("frontier_explorer_astar")
{
  obstacle_threshold_       = this->declare_parameter<int>("obstacle_threshold", 70);
  free_threshold_           = this->declare_parameter<int>("free_threshold", 60);
  inflation_radius_m_       = this->declare_parameter<double>("inflation_radius_m", 0.25);
  frontier_search_radius_m_ = this->declare_parameter<double>("frontier_search_radius_m", 8.0);

  lookahead_m_              = this->declare_parameter<double>("lookahead_m", 0.35);
  goal_tol_m_               = this->declare_parameter<double>("goal_tolerance_m", 0.35);
  max_lin_vel_              = this->declare_parameter<double>("max_lin_vel", 0.25);
  max_ang_vel_              = this->declare_parameter<double>("max_ang_vel", 0.90);

  map_topic_                = this->declare_parameter<std::string>("map_topic", "/map");
  cmd_topic_                = this->declare_parameter<std::string>("cmd_topic", "/cmd_vel");
  scan_topic_               = this->declare_parameter<std::string>("scan_topic", "/scan");

  map_frame_                = this->declare_parameter<std::string>("map_frame", "map");
  base_frame_               = this->declare_parameter<std::string>("base_frame", "base_footprint");
  tf_timeout_s_             = this->declare_parameter<double>("tf_timeout_s", 0.1);

  laser_inflation_radius_m_ = this->declare_parameter<double>("laser_inflation_radius_m", 0.12);
  laser_block_ttl_          = this->declare_parameter<double>("laser_block_ttl", 1.0);

  frontier_clearance_m_     = this->declare_parameter<double>("frontier_clearance_m", 0.30);
  path_clearance_m_         = this->declare_parameter<double>("path_clearance_m", 0.12);

  infl_viz_radius_m_        = this->declare_parameter<double>("infl_viz_radius_m", 3.0);

  stuck_time_s_             = this->declare_parameter<double>("stuck_time_s", 2.0);
  stuck_dist_m_             = this->declare_parameter<double>("stuck_dist_m", 0.05);

  use_dbscan_               = this->declare_parameter<bool>("use_dbscan", true);
  dbscan_eps_m_             = this->declare_parameter<double>("dbscan_eps_m", 0.20);
  dbscan_min_pts_           = this->declare_parameter<int>("dbscan_min_pts", 6);

  warmup_enable_            = this->declare_parameter<bool>("warmup_enable", true);
  warmup_min_known_cells_   = this->declare_parameter<int>("warmup_min_known_cells", 12000);
  warmup_min_occ_cells_     = this->declare_parameter<int>("warmup_min_occ_cells", 50);
  warmup_radius_m_          = this->declare_parameter<double>("warmup_radius_m", 2.0);
  warmup_yaw_rate_          = this->declare_parameter<double>("warmup_yaw_rate", 0.6);
  warmup_lin_vel_           = this->declare_parameter<double>("warmup_lin_vel", 0.0);
  warmup_max_time_s_        = this->declare_parameter<double>("warmup_max_time_s", 6.0);

  avoid_dist_               = this->declare_parameter<double>("avoid_dist", 0.45);
  avoid_end_dist_           = this->declare_parameter<double>("avoid_end_dist", 0.60);

  std::string infl_marker_topic = this->declare_parameter<std::string>("infl_marker_topic", "/i");
  std::string path_marker_topic = this->declare_parameter<std::string>("path_marker_topic", "/p");
  std::string frontier_marker_topic = this->declare_parameter<std::string>("frontier_marker_topic", "/f");

  // TF
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // SUB & PUB
  auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, map_qos,
    std::bind(&FrontierExplorerAStar::onMap, this, std::placeholders::_1)
  );

  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, 10);
  path_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(path_marker_topic, 10);
  frontier_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(frontier_marker_topic, 10);
  infl_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(infl_marker_topic, 10);

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, 10, std::bind(&FrontierExplorerAStar::onScan, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&FrontierExplorerAStar::onTimer, this)
  );

  warmup_state_start_ = this->now();
  last_progress_time_ = this->now();
  last_progress_x_ = 0.0;
  last_progress_y_ = 0.0;
  last_progress_wp_ = 0;

  RCLCPP_WARN(this->get_logger(),
    "Started map_frame=%s base_frame=%s free<=%d blocked>=%d infl=%.2fm frontierR=%.1fm scan=%s",
    map_frame_.c_str(), base_frame_.c_str(),
    free_threshold_, obstacle_threshold_, inflation_radius_m_, frontier_search_radius_m_,
    scan_topic_.c_str());
}

void FrontierExplorerAStar::onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  map_ = *msg;
  has_map_ = true;
}

bool FrontierExplorerAStar::updateRobotPoseFromTF() {
  try {
    const auto tf = tf_buffer_->lookupTransform(
      map_frame_, base_frame_, tf2::TimePointZero,
      tf2::durationFromSec(tf_timeout_s_)
    );

    robot_.x = tf.transform.translation.x;
    robot_.y = tf.transform.translation.y;
    robot_.yaw = tf2::getYaw(tf.transform.rotation);
    has_pose_ = true;
    return true;
  } catch (const tf2::TransformException &) {
    has_pose_ = false;
    return false;
  }
}

bool FrontierExplorerAStar::isTraversable(int x, int y) const {
  if (!has_map_) return false;
  if (!fe::inBounds(map_, x, y)) return false;
  int v = map_.data[fe::IDX(x, y, (int)map_.info.width)];
  if (v == fe::UNKNOWN) return false;
  return (v >= 0 && v <= free_threshold_);
}

bool FrontierExplorerAStar::isFrontierCell(int x, int y) const {
  if (!isTraversable(x,y)) return false;
  for (int k=0;k<8;k++){
    int nx=x+fe::dx8[k], ny=y+fe::dy8[k];
    if (!fe::inBounds(map_, nx, ny)) continue;
    int nv = map_.data[fe::IDX(nx, ny, (int)map_.info.width)];
    if (nv == fe::UNKNOWN) return true;
  }
  return false;
}

std::vector<GridPose> FrontierExplorerAStar::detectFrontiers(const GridPose &robot_g) const {
  std::vector<GridPose> out;
  int W = (int)map_.info.width;
  int H = (int)map_.info.height;

  int r_cells = (int)std::ceil(frontier_search_radius_m_ / map_.info.resolution);
  int x0 = std::max(0, robot_g.x - r_cells);
  int x1 = std::min(W-1, robot_g.x + r_cells);
  int y0 = std::max(0, robot_g.y - r_cells);
  int y1 = std::min(H-1, robot_g.y + r_cells);

  for (int y=y0;y<=y1;y++){
    for (int x=x0;x<=x1;x++){
      if (isFrontierCell(x,y)) out.push_back({x,y});
    }
  }
  return out;
}

void FrontierExplorerAStar::onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg){
  last_scan_ = *msg;
  has_scan_ = true;

  if (!has_map_ || !has_pose_) return;

  int W = (int)map_.info.width;
  int H = (int)map_.info.height;
  laser_blocked_.assign(W*H, 0);

  geometry_msgs::msg::TransformStamped tf_scan_to_map;
  try {
    tf_scan_to_map = tf_buffer_->lookupTransform(
      map_frame_, msg->header.frame_id, tf2::TimePointZero,
      tf2::durationFromSec(tf_timeout_s_)
    );
  } catch (const tf2::TransformException &) {
    return;
  }

  int lrad = (int)std::ceil(laser_inflation_radius_m_ / map_.info.resolution);

  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    double r = msg->ranges[i];
    if (!std::isfinite(r) || r > avoid_dist_) continue;

    double a = msg->angle_min + i * msg->angle_increment;

    double lx = r * std::cos(a);
    double ly = r * std::sin(a);

    geometry_msgs::msg::PointStamped p_scan, p_map;
    p_scan.header = msg->header;
    p_scan.point.x = lx;
    p_scan.point.y = ly;
    p_scan.point.z = 0.0;

    tf2::doTransform(p_scan, p_map, tf_scan_to_map);

    GridPose g = fe::worldToGrid(map_, p_map.point.x, p_map.point.y);
    if (!fe::inBounds(map_, g.x, g.y)) continue;

    for (int dy=-lrad; dy<=lrad; ++dy) {
      for (int dx=-lrad; dx<=lrad; ++dx) {
        int nx = g.x + dx;
        int ny = g.y + dy;
        if (!fe::inBounds(map_, nx, ny)) continue;

        double dist = std::sqrt((double)dx*dx + (double)dy*dy) * map_.info.resolution;
        if (dist <= laser_inflation_radius_m_) {
          laser_blocked_[fe::IDX(nx, ny, W)] = 1;
        }
      }
    }
  }

  last_laser_update_ = std::chrono::steady_clock::now();
}

std::vector<uint8_t> FrontierExplorerAStar::buildInflatedBlockedMask() const {
  int W = (int)map_.info.width;
  int H = (int)map_.info.height;
  std::vector<uint8_t> blocked(W*H, 0);

  std::vector<uint8_t> unknown(W*H, 0);
  std::vector<uint8_t> obs(W*H, 0);

  for (int y=0;y<H;y++){
    for (int x=0;x<W;x++){
      int v = map_.data[fe::IDX(x,y,W)];
      if (v == fe::UNKNOWN) unknown[fe::IDX(x,y,W)] = 1;
      else if (v >= obstacle_threshold_) obs[fe::IDX(x,y,W)] = 1;
    }
  }

  int rad = (int)std::ceil(inflation_radius_m_ / map_.info.resolution);

  std::vector<uint8_t> inflated_obs = obs;
  if (rad > 0) {
    for (int y=0;y<H;y++){
      for (int x=0;x<W;x++){
        if (!obs[fe::IDX(x,y,W)]) continue;
        for (int dy=-rad; dy<=rad; dy++){
          for (int dx=-rad; dx<=rad; dx++){
            int nx = x+dx, ny = y+dy;
            if (!fe::inBounds(map_, nx, ny)) continue;

            double dist = std::sqrt((double)dx*dx + (double)dy*dy) * map_.info.resolution;
            if (dist <= inflation_radius_m_) inflated_obs[fe::IDX(nx,ny,W)] = 1;
          }
        }
      }
    }
  }

  for (int i=0;i<W*H;i++) blocked[i] = (unknown[i] || inflated_obs[i]) ? 1 : 0;

  // 레이저 블럭 합치기 (TTL 적용)
  auto now = std::chrono::steady_clock::now();
  double dt = std::chrono::duration<double>(now - last_laser_update_).count();
  if (!laser_blocked_.empty() && dt < laser_block_ttl_) {
    GridPose robot_g = fe::worldToGrid(map_, robot_.x, robot_.y);
    int keep = 2;

    for (int y=0; y<H; ++y) {
      for (int x=0; x<W; ++x) {
        int id = fe::IDX(x,y,W);
        if (!laser_blocked_[id]) continue;

        int dx = x - robot_g.x;
        int dy = y - robot_g.y;
        if (std::abs(dx) <= keep && std::abs(dy) <= keep) continue;

        blocked[id] = 1;
      }
    }
  }

  // keep-open: 로봇 주변은 blocked 풀어주기(unknown 제외)
  GridPose rg = fe::worldToGrid(map_, robot_.x, robot_.y);
  int keep = 3;

  for (int dy=-keep; dy<=keep; ++dy) {
    for (int dx=-keep; dx<=keep; ++dx) {
      int nx = rg.x + dx;
      int ny = rg.y + dy;
      if (!fe::inBounds(map_, nx, ny)) continue;

      int v = map_.data[fe::IDX(nx, ny, W)];
      if (v == fe::UNKNOWN) continue;

      blocked[fe::IDX(nx, ny, W)] = 0;
    }
  }

  return blocked;
}

std::vector<uint8_t> FrontierExplorerAStar::buildObstacleInflatedMask() const {
  int W = (int)map_.info.width;
  int H = (int)map_.info.height;

  std::vector<uint8_t> obs(W*H, 0);
  for (int y=0;y<H;y++){
    for (int x=0;x<W;x++){
      int v = map_.data[fe::IDX(x,y,W)];
      if (v != fe::UNKNOWN && v >= obstacle_threshold_) obs[fe::IDX(x,y,W)] = 1;
    }
  }

  int rad = (int)std::ceil(inflation_radius_m_ / map_.info.resolution);
  if (rad <= 0) return obs;

  std::vector<uint8_t> inflated = obs;
  for (int y=0;y<H;y++){
    for (int x=0;x<W;x++){
      if (!obs[fe::IDX(x,y,W)]) continue;
      for (int dy=-rad; dy<=rad; dy++){
        for (int dx=-rad; dx<=rad; dx++){
          int nx=x+dx, ny=y+dy;
          if (!fe::inBounds(map_, nx, ny)) continue;
          double dist = std::sqrt((double)dx*dx + (double)dy*dy) * map_.info.resolution;
          if (dist <= inflation_radius_m_) inflated[fe::IDX(nx,ny,W)] = 1;
        }
      }
    }
  }
  return inflated;
}

std::vector<uint8_t> FrontierExplorerAStar::buildObstacleRawMask() const {
  int W = (int)map_.info.width;
  int H = (int)map_.info.height;
  std::vector<uint8_t> obs(W*H, 0);

  for (int y=0;y<H;y++){
    for (int x=0;x<W;x++){
      int v = map_.data[fe::IDX(x,y,W)];
      if (v != fe::UNKNOWN && v >= obstacle_threshold_) obs[fe::IDX(x,y,W)] = 1;
    }
  }
  return obs;
}

bool FrontierExplorerAStar::isFrontierTooCloseToObstacle(const GridPose& f,
                                                         const std::vector<uint8_t>& obsMaskInflated,
                                                         int radius_cells) const {
  int W = (int)map_.info.width;
  for (int dy=-radius_cells; dy<=radius_cells; ++dy) {
    for (int dx=-radius_cells; dx<=radius_cells; ++dx) {
      int nx = f.x + dx;
      int ny = f.y + dy;
      if (!fe::inBounds(map_, nx, ny)) continue;
      if (obsMaskInflated[fe::IDX(nx, ny, W)]) return true;
    }
  }
  return false;
}

double FrontierExplorerAStar::minRange(double a_min, double a_max) {
  if (!has_scan_) return 1e9;
  double min_r = 1e9;
  for (size_t i = 0; i < last_scan_.ranges.size(); ++i) {
    double r = last_scan_.ranges[i];
    if (!std::isfinite(r)) continue;

    double a = last_scan_.angle_min + i * last_scan_.angle_increment;
    if (a < a_min || a > a_max) continue;
    min_r = std::min(min_r, r);
  }
  return min_r;
}

void FrontierExplorerAStar::publishStop() {
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.0;
  cmd.angular.z = 0.0;
  cmd_pub_->publish(cmd);
}

void FrontierExplorerAStar::publishAvoidCmd() {
  geometry_msgs::msg::Twist cmd;

  double front = minRange(-0.3, 0.3);
  double left  = minRange(0.3, 1.0);
  double right = minRange(-1.0, -0.3);

  if (front < avoid_dist_) {
    cmd.linear.x = -0.05;
    cmd.angular.z = (left > right) ? 0.6 : -0.6;
  } else {
    cmd.linear.x = 0.15;
    cmd.angular.z = 0.0;
  }

  cmd_pub_->publish(cmd);
}

int FrontierExplorerAStar::findNearestIndexOnPath(const std::vector<GridPose>& path, int start_idx, int window) {
  if (path.empty() || !has_map_) return 0;

  const int N = (int)path.size();
  int i0 = std::max(0, start_idx - window);
  int i1 = std::min(N - 1, start_idx + window);

  int best_i = i0;
  double best_d2 = 1e18;

  for (int i = i0; i <= i1; ++i) {
    auto [wx, wy] = fe::gridToWorld(map_, path[i].x, path[i].y);
    double dx = wx - robot_.x;
    double dy = wy - robot_.y;
    double d2 = dx*dx + dy*dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best_i = i;
    }
  }
  return best_i;
}

void FrontierExplorerAStar::followPathStep() {
  geometry_msgs::msg::Twist cmd;
  if (path_.empty()) return;

  wp_idx_ = findNearestIndexOnPath(path_, wp_idx_, 25);

  int lookahead_cells = 3;
  int target_idx = std::min(wp_idx_ + lookahead_cells, (int)path_.size() - 1);

  const auto& target = path_[target_idx];
  auto [tx, ty] = fe::gridToWorld(map_, target.x, target.y);

  double dxw = tx - robot_.x;
  double dyw = ty - robot_.y;

  double heading_x = std::cos(robot_.yaw);
  double heading_y = std::sin(robot_.yaw);

  double dot = dxw * heading_x + dyw * heading_y;

  if (dot < 0.0) {
    double ang_err = fe::normAngle(std::atan2(dyw, dxw) - robot_.yaw);
    cmd.linear.x  = 0.0;
    cmd.angular.z = fe::clampd(2.0 * ang_err, -max_ang_vel_, max_ang_vel_);
    cmd_pub_->publish(cmd);
    return;
  }

  double dist = std::hypot(dxw, dyw);
  double ang_err = fe::normAngle(std::atan2(dyw, dxw) - robot_.yaw);

  if (dist < reach_dist_) {
    wp_idx_ = target_idx + 1;
    if (wp_idx_ >= (int)path_.size()) path_.clear();
    return;
  }

  if (std::abs(ang_err) > 0.6) {
    cmd.linear.x  = 0.0;
    cmd.angular.z = fe::clampd(2.5 * ang_err, -max_ang_vel_, max_ang_vel_);
  } else {
    cmd.linear.x  = fe::clampd(0.6 * dist, 0.05, max_lin_vel_);
    cmd.angular.z = fe::clampd(2.0 * ang_err, -max_ang_vel_, max_ang_vel_);
  }

  cmd_pub_->publish(cmd);
}

FrontierExplorerAStar::LocalMapStats
FrontierExplorerAStar::computeLocalMapStats(const GridPose& center_g, double radius_m) const {
  LocalMapStats s;
  if (!has_map_) return s;

  const int W = (int)map_.info.width;
  const int H = (int)map_.info.height;
  const double res = map_.info.resolution;

  int r = (int)std::ceil(radius_m / res);
  int x0 = std::max(0, center_g.x - r);
  int x1 = std::min(W-1, center_g.x + r);
  int y0 = std::max(0, center_g.y - r);
  int y1 = std::min(H-1, center_g.y + r);

  for (int y=y0; y<=y1; ++y) {
    for (int x=x0; x<=x1; ++x) {
      double dx = (x - center_g.x) * res;
      double dy = (y - center_g.y) * res;
      if (dx*dx + dy*dy > radius_m*radius_m) continue;

      int v = map_.data[fe::IDX(x,y,W)];
      if (v == fe::UNKNOWN) continue;

      s.known++;
      s.minv = std::min(s.minv, v);
      s.maxv = std::max(s.maxv, v);
      if (v >= obstacle_threshold_) s.occ++;
    }
  }
  return s;
}

void FrontierExplorerAStar::publishWarmupMotion() {
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x  = warmup_lin_vel_;
  cmd.angular.z = warmup_yaw_rate_;
  cmd_pub_->publish(cmd);
}

bool FrontierExplorerAStar::pickNearestFrontier(const GridPose &robot_g,
                                                const std::vector<GridPose> &frontiers,
                                                const std::vector<uint8_t> &blockedMask,
                                                const std::vector<uint8_t> &obsMaskInflated,
                                                const std::vector<uint8_t> &obsMaskRaw,
                                                GridPose &out_goal,
                                                std::vector<GridPose> &out_path) const {
  int W = (int)map_.info.width;

  int bestLen = std::numeric_limits<int>::max();
  bool found = false;

  for (const auto &f : frontiers) {
    if (obsMaskInflated[fe::IDX(f.x,f.y,W)]) continue;
    if (blockedMask[fe::IDX(f.x,f.y,W)]) continue;

    auto p = fe::astar(map_, robot_g, f, obsMaskInflated);
    if (p.empty()) continue;

    if (fe::pathTailTooCloseToRawObstacle(map_, p, obsMaskRaw, path_clearance_m_, obstacle_threshold_))
      continue;

    int len = (int)p.size();
    if (len < bestLen) {
      bestLen = len;
      out_goal = f;
      out_path = std::move(p);
      found = true;
    }
  }
  return found;
}

void FrontierExplorerAStar::onTimer() {
  if (done_ || !has_map_) return;

  if (!updateRobotPoseFromTF()) {
    publishStop();
    return;
  }

  GridPose robot_g = fe::worldToGrid(map_, robot_.x, robot_.y);
  if (!fe::inBounds(map_, robot_g.x, robot_g.y)) {
    publishStop();
    return;
  }

  // warmup
  if (warmup_enable_ && !warmup_done_) {
    auto st = computeLocalMapStats(robot_g, warmup_radius_m_);
    double t = (this->now() - warmup_state_start_).seconds();

    bool ready = (st.known >= warmup_min_known_cells_) && (st.occ >= warmup_min_occ_cells_);
    bool timeout = (t >= warmup_max_time_s_);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *this->get_clock(), 1000,
      "warmup: known=%d occ=%d min=%d max=%d t=%.1f ready=%d",
      st.known, st.occ, st.minv, st.maxv, t, (int)ready
    );

    if (!ready && !timeout) {
      path_.clear();
      wp_idx_ = 0;
      avoiding_ = false;
      publishWarmupMotion();
      return;
    }

    warmup_done_ = true;
    publishStop();
  }

  // 1) 회피
  double front = minRange(-0.3, 0.3);

  if (!avoiding_ && has_scan_ && front < avoid_dist_) {
    avoiding_ = true;
    path_.clear();
    wp_idx_ = 0;
  }

  if (avoiding_) {
    if (has_scan_ && front > avoid_end_dist_) {
      avoiding_ = false;
      path_.clear();
      wp_idx_ = 0;
    } else {
      publishAvoidCmd();
      return;
    }
  }

  // masks
  auto blockedMask     = buildInflatedBlockedMask();
  auto obsMaskInflated = buildObstacleInflatedMask();
  auto obsMaskRaw      = buildObstacleRawMask();

  // keep-open around robot for masks
  {
    int W = (int)map_.info.width;
    int keep = 2;
    for (int dy=-keep; dy<=keep; ++dy) {
      for (int dx=-keep; dx<=keep; ++dx) {
        int nx = robot_g.x + dx;
        int ny = robot_g.y + dy;
        if (!fe::inBounds(map_, nx, ny)) continue;
        obsMaskInflated[fe::IDX(nx, ny, W)] = 0;
        blockedMask[fe::IDX(nx, ny, W)] = 0;
      }
    }
  }

  fe::publishInflationMaskMarker(infl_marker_pub_, now(), map_frame_, map_,
                                 blockedMask, robot_g, infl_viz_radius_m_);

  // if we have a path: validate & follow
  if (!path_.empty()) {
    int new_wp = findNearestIndexOnPath(path_, wp_idx_, 25);

    auto isPathInvalidNow = [&](const std::vector<GridPose>& path,
                                int idx,
                                const std::vector<uint8_t>& mask)->bool {
      if (path.empty()) return true;
      int W = (int)map_.info.width;
      int i0 = std::max(0, idx - 1);
      int i1 = std::min((int)path.size() - 1, idx + 3);
      int tail_cut = (int)(path.size() * 0.85);
      i1 = std::min(i1, tail_cut);
      for (int i=i0; i<=i1; ++i) {
        const auto& p = path[i];
        if (!fe::inBounds(map_, p.x, p.y)) return true;
        if (mask[fe::IDX(p.x, p.y, W)]) return true;
      }
      return false;
    };

    if (isPathInvalidNow(path_, new_wp, blockedMask)) {
      RCLCPP_WARN(get_logger(), "path became invalid -> clear & replan");
      path_.clear();
      wp_idx_ = 0;
      publishStop();
      return;
    }

    // stuck detect
    double moved = std::hypot(robot_.x - last_progress_x_, robot_.y - last_progress_y_);
    bool progressed = (new_wp > last_progress_wp_ + 1) || (moved > 0.05);

    if (progressed) {
      last_progress_time_ = this->now();
      last_progress_x_ = robot_.x;
      last_progress_y_ = robot_.y;
      last_progress_wp_ = new_wp;
    } else {
      double dt = (this->now() - last_progress_time_).seconds();
      if (dt > stuck_time_s_) {
        RCLCPP_WARN(get_logger(), "stuck %.2fs -> clear & replan", dt);
        path_.clear();
        wp_idx_ = 0;
        last_progress_time_ = this->now();
        last_progress_x_ = robot_.x;
        last_progress_y_ = robot_.y;
        last_progress_wp_ = 0;
        publishStop();
        return;
      }
    }

    wp_idx_ = new_wp;
    followPathStep();
    return;
  }

  // 2) frontier detect & select
  auto frontiers = detectFrontiers(robot_g);

  if (frontiers.empty()) {
    publishStop();
    return;
  }

  // A) remove frontiers close to obstacle
  int clearance_cells = (int)std::ceil(frontier_clearance_m_ / map_.info.resolution);
  frontiers.erase(
    std::remove_if(frontiers.begin(), frontiers.end(),
      [&](const GridPose& f){
        return isFrontierTooCloseToObstacle(f, obsMaskInflated, clearance_cells);
      }),
    frontiers.end()
  );

  if (frontiers.empty()) {
    publishStop();
    RCLCPP_WARN(get_logger(), "no frontiers left");
    return;
  }

  // reachable filter (using inflated obstacle mask)
  auto obsReach = obsMaskInflated;
  {
    int W = (int)map_.info.width;
    int keep = 2;
    for (int dy=-keep; dy<=keep; ++dy) {
      for (int dx=-keep; dx<=keep; ++dx) {
        int nx = robot_g.x + dx;
        int ny = robot_g.y + dy;
        if (!fe::inBounds(map_, nx, ny)) continue;
        obsReach[fe::IDX(nx, ny, W)] = 0;
      }
    }
  }

  auto reachable = fe::buildReachableMaskFromStart(map_, robot_g, obsReach);
  fe::filterFrontiersByReachable(map_, frontiers, reachable);

  if (frontiers.empty()) {
    publishStop();
    fe::publishFrontierMarkers(frontier_marker_pub_, now(), map_frame_, map_, {});
    RCLCPP_WARN(get_logger(), "no reachable frontiers left (after reachable filter)");
    return;
  }

  // DBSCAN reps
  std::vector<GridPose> reps;
  if (use_dbscan_) {
    auto labels = fe::dbscanCluster(map_, frontiers, dbscan_eps_m_, dbscan_min_pts_);
    reps = fe::computeClusterRepresentatives(frontiers, labels);
    RCLCPP_WARN(get_logger(), "DBSCAN reps=%zu (from %zu)", reps.size(), frontiers.size());
  } else {
    reps = frontiers;
  }

  if (reps.empty()) {
    RCLCPP_WARN(get_logger(), "DBSCAN reps=0 -> fallback to raw reachable frontiers (%zu)", frontiers.size());
    reps = frontiers;
  }

  fe::publishFrontierMarkers(frontier_marker_pub_, now(), map_frame_, map_, reps);

  GridPose goal;
  std::vector<GridPose> new_path;

  bool planned = pickNearestFrontier(robot_g, reps, blockedMask, obsMaskInflated, obsMaskRaw, goal, new_path);
  if (!planned) {
    publishStop();
    RCLCPP_WARN(get_logger(), "no plan");
    return;
  }

  path_ = std::move(new_path);
  wp_idx_ = 0;

  fe::publishPathMarker(path_marker_pub_, now(), map_frame_, map_, path_);
  followPathStep();

  RCLCPP_WARN(get_logger(), "plan: (%d, %d) -> (%d, %d)", robot_g.x, robot_g.y, goal.x, goal.y);
}

} // namespace fe
