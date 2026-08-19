#include "frontier_multi_uwb.hpp"


FrontierExplorerMulti ::FrontierExplorerMulti() 
: Node("frontier_multi_uwb")
  {
    declare_params();
    setup_ros_interfaces();

  }

  void FrontierExplorerMulti::declare_params(){
    robot_id_ = this->declare_parameter<std::string>("robot_id", "robot1");
    map_topic_ = this->declare_parameter<std::string>("map_topic", "map");
    cmd_topic_ = this->declare_parameter<std::string>("cmd_topic", "cmd_vel");
    dwb_cmd_topic_ = this->declare_parameter<std::string>("dwb_cmd_topic", "cmd_vel_dwb");
    dynamic_cmd_topic_ = this->declare_parameter<std::string>("dynamic_cmd_topic", "cmd_vel_dynamic");
    follow_path_action_name_ = this->declare_parameter<std::string>("follow_path_action", "follow_path");
    scan_topic_ = this->declare_parameter<std::string>("scan_topic", "scan");

    map_frame_  = this->declare_parameter<std::string>("map_frame", "map");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_footprint");
    global_frame_ = this->declare_parameter<std::string>("global_frame", "world");
    tf_timeout_s_ = this->declare_parameter<double>("tf_timeout_s", 0.10);

    obstacle_threshold_ = this->declare_parameter<int>("obstacle_threshold", 60);
    free_threshold_     = this->declare_parameter<int>("free_threshold", 50);

    inflation_radius_m_       = this->declare_parameter<double>("inflation_radius_m", 0.40);
    frontier_search_radius_m_ = this->declare_parameter<double>("frontier_search_radius_m", 6.0);
    frontier_extended_search_radius_m_ = this->declare_parameter<double>(
        "frontier_extended_search_radius_m", 12.0);
    frontier_full_map_fallback_ = this->declare_parameter<bool>(
        "frontier_full_map_fallback", true);

    avoid_enter_dist_ = this->declare_parameter<double>("avoid_enter_dist", 0.35);

    frontier_clearance_m_ = this->declare_parameter<double>("frontier_clearance_m", 0.40);
    path_clearance_m_     = this->declare_parameter<double>("path_clearance_m", 0.55);
    path_clearance_cost_weight_ = this->declare_parameter<int>(
        "path_clearance_cost_weight", 25);

    keep_open_cells_ = this->declare_parameter<int>("keep_open_cells", 2);

    use_dbscan_     = this->declare_parameter<bool>("use_dbscan", true);
    dbscan_eps_m_   = this->declare_parameter<double>("dbscan_eps_m", 0.25);
    dbscan_min_pts_ = this->declare_parameter<int>("dbscan_min_pts", 10);
 
    laser_block_ttl_ = this->declare_parameter<double>("laser_block_ttl", 1.0);
    laser_inflation_radius_m_ = this->declare_parameter<double>("laser_inflation_radius_m", 0.40);
    laser_obstacle_max_range_ = this->declare_parameter<double>("laser_obstacle_max_range", 0.70);
    dynamic_max_linear_speed_ = this->declare_parameter<double>("dynamic_max_linear_speed", 0.08);
    dynamic_max_angular_speed_ = this->declare_parameter<double>("dynamic_max_angular_speed", 0.8);
    dynamic_stop_distance_ = this->declare_parameter<double>("dynamic_stop_distance", 0.45);
    dynamic_slow_distance_ = this->declare_parameter<double>("dynamic_slow_distance", 0.70);
    dwb_cmd_timeout_s_ = this->declare_parameter<double>("dwb_cmd_timeout_s", 0.50);

    stuck_timeout_s_ = this->declare_parameter<double>("stuck_timeout_s", 8.0);
    stuck_min_move_m_ = this->declare_parameter<double>("stuck_min_move_m", 0.02);
    stuck_grace_s_ = this->declare_parameter<double>("stuck_grace_s", 8.0);
    path_blocked_lookahead_m_ = this->declare_parameter<double>(
        "path_blocked_lookahead_m", 1.5);
    path_blocked_confirm_s_ = this->declare_parameter<double>(
        "path_blocked_confirm_s", 0.3);

    enable_viz_ = this->declare_parameter<bool>("enable_viz", true);

    info_gain_radius_m_ = this->declare_parameter<double>("info_gain_radius_m", 1.50);
    rendezvous_anchor_topic_ = this->declare_parameter<std::string>(
        "rendezvous_anchor_topic", "rendezvous_anchor");
    rendezvous_command_ttl_s_ = this->declare_parameter<double>(
        "rendezvous_command_ttl_s", 3.0);
    rendezvous_arrival_radius_m_ = this->declare_parameter<double>(
        "rendezvous_arrival_radius_m", 2.0);
    rendezvous_direct_min_distance_m_ = this->declare_parameter<double>(
        "rendezvous_direct_min_distance_m", 0.6);
    rendezvous_utility_weight_ = this->declare_parameter<double>(
        "rendezvous_utility_weight", 4.0);

    alpha_ = this->declare_parameter<double>("alpha_info_gain", 10);
    beta_  = this->declare_parameter<double>("beta_path_len", 2.0);
    delta_ = this->declare_parameter<double>("delta_reserve", 15.0);

    reserve_exclusion_radius_m_ = this->declare_parameter<double>("reserve_exclusion_radius_m", 2.0);
    reserve_ttl_s_ = this->declare_parameter<double>("reserve_ttl_s", 6.0);
    reserve_refresh_period_s_ = this->declare_parameter<double>("reserve_refresh_period_s", 1.0);
    reserve_out_topic_ = this->declare_parameter<std::string>("reserve_out_topic", "/global_goal_reservation");
    robot_position_topic_ = this->declare_parameter<std::string>("robot_position_topic", "/global_robot_positions");
    robot_position_ttl_s_ = this->declare_parameter<double>("robot_position_ttl_s", 1.0);
    robot_position_period_s_ = this->declare_parameter<double>("robot_position_period_s", 0.2);
    other_robot_radius_m_ = this->declare_parameter<double>("other_robot_radius_m", 0.40);
    other_robot_path_lookahead_m_ = this->declare_parameter<double>(
        "other_robot_path_lookahead_m", 1.5);

    path_marker_topic_     = this->declare_parameter<std::string>("path_marker_topic", "path_marker");
    frontier_marker_topic_ = this->declare_parameter<std::string>("frontier_marker_topic", "frontier_markers");
    infl_marker_topic_     = this->declare_parameter<std::string>("infl_marker_topic", "inflation_marker");
    cluster_marker_topic_  = this->declare_parameter<std::string>("cluster_marker_topic", "cluster_marker");
    
    gate_goal_topic_   = this->declare_parameter<std::string>("gate_goal_topic", "goal_assignment");
    map_delta_topic_   = this->declare_parameter<std::string>("map_delta_topic", "map_delta");
    gate_timeout_s_    = this->declare_parameter<double>("gate_timeout_s", 3.0);
    gate_goal_switch_distance_m_ = this->declare_parameter<double>(
        "gate_goal_switch_distance_m", 0.75);
    gate_goal_min_distance_m_ = this->declare_parameter<double>(
        "gate_goal_min_distance_m", 0.75);
    map_delta_period_s_= this->declare_parameter<double>("map_delta_period_s", 1.0);

    blacklist_ttl_s_ = this->declare_parameter<double>("blacklist_ttl_s", 8.0);
    blacklist_radius_m_ = this->declare_parameter<double>("blacklist_radius_m", 0.60);
    dwb_failure_blacklist_ttl_s_ = this->declare_parameter<double>(
        "dwb_failure_blacklist_ttl_s", 30.0);
    dwb_failure_blacklist_radius_m_ = this->declare_parameter<double>(
        "dwb_failure_blacklist_radius_m", 1.0);
    gate_plan_fail_max_ = this->declare_parameter<int>("gate_plan_fail_max", 3);

    local_map_topic_ = this->declare_parameter<std::string>("local_map_topic", "map");
    merge_map_stale_s_ = this->declare_parameter<double>("merge_map_stale_s", 5.0);
  }

  void FrontierExplorerMulti::setup_ros_interfaces(){
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, map_qos, std::bind(&FrontierExplorerMulti::onMap, this, std::placeholders::_1));
    
      auto scan_qos = rclcpp::QoS(rclcpp::KeepLast(10));
      scan_qos.best_effort();

    local_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    local_map_topic_, map_qos,
    std::bind(&FrontierExplorerMulti::onLocalMap, this, std::placeholders::_1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, scan_qos, std::bind(&FrontierExplorerMulti::onScan, this, std::placeholders::_1));
    rendezvous_anchor_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseStamped>(
        rendezvous_anchor_topic_, 10,
        std::bind(&FrontierExplorerMulti::onRendezvousAnchor, this, std::placeholders::_1));

    map_delta_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        map_delta_topic_, rclcpp::QoS(1).reliable().durability_volatile());

    gate_goal_sub_ =
    this->create_subscription<geometry_msgs::msg::PoseStamped>(
        gate_goal_topic_,
        10,
        std::bind(
            &FrontierExplorerMulti::onGateGoal,
            this,
            std::placeholders::_1));

    explore_done_sub_ =
    this->create_subscription<std_msgs::msg::Bool>(
        "/exploration_done", 1,
        [this](const std_msgs::msg::Bool::SharedPtr msg)
        {
            if (msg->data) {
                exploration_done_ = true;
            }
        });


    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, 10);
    dwb_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      dwb_cmd_topic_, 10,
      std::bind(&FrontierExplorerMulti::onDwbCmd, this, std::placeholders::_1));
    dynamic_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(dynamic_cmd_topic_, 10);
    follow_path_client_ = rclcpp_action::create_client<FollowPath>(
      this, follow_path_action_name_);

    path_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(path_marker_topic_, 10);
    frontier_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(frontier_marker_topic_, 10);
    infl_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(infl_marker_topic_, 10);
    cluster_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(cluster_marker_topic_, 10);

    reserve_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(reserve_out_topic_, 10);
    reserve_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          reserve_out_topic_, 10,
          [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
              this->onReservePoint(*msg, msg->header.frame_id); 
          }
      );
    robot_position_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(robot_position_topic_, 10);
    robot_position_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        robot_position_topic_, 10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          this->onRobotPosition(*msg, msg->header.frame_id);
        });

    obs_sub_ = 
    this->create_subscription<frontier_ws::msg::DynamicObstacle>(
        "/" + robot_id_ + "/obs_speed", 
        10,std::bind(
            &FrontierExplorerMulti::obsCallback,
            this, 
            std::placeholders::_1));
    dynamic_controller_ = std::make_shared<Controller>(this->get_clock());

    replan_check_period_s_ = this->declare_parameter<double>("replan_check_period_s", 0.5);
    plan_retry_period_s_ = this->declare_parameter<double>("plan_retry_period_s", 0.75);
    min_commit_time_s_     = this->declare_parameter<double>("min_commit_time_s", 2.0);
    ig_drop_thresh_        = this->declare_parameter<double>("ig_drop_thresh", 0.10);
    ig_drop_ratio_         = this->declare_parameter<double>("ig_drop_ratio", 0.40);
    ig_drop_baseline_min_  = this->declare_parameter<double>("ig_drop_baseline_min", 0.20);
    ig_replan_min_age_s_   = this->declare_parameter<double>("ig_replan_min_age_s", 2.0);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
      std::bind(&FrontierExplorerMulti::onTimer, this));

  }


  bool FrontierExplorerMulti::inBounds(int x, int y) const {
    return (0 <= x && x < (int)map_.info.width && 0 <= y && y < (int)map_.info.height);
  }

  GridPose FrontierExplorerMulti::worldToGrid(double wx, double wy) const {
    const auto &info = map_.info;
    int gx = (int)std::floor((wx - info.origin.position.x) / info.resolution);
    int gy = (int)std::floor((wy - info.origin.position.y) / info.resolution);
    return {gx, gy};
  }

  std::pair<double,double> FrontierExplorerMulti::gridToWorld(int gx, int gy) const {
    const auto &info = map_.info;
    double wx = info.origin.position.x + (gx + 0.5) * info.resolution;
    double wy = info.origin.position.y + (gy + 0.5) * info.resolution;
    return {wx, wy};
  }

  void FrontierExplorerMulti::publishMapDelta() {
    if (!has_map_) return;

    auto now = this->now();
    if ((now - last_delta_pub_time_).seconds() < map_delta_period_s_) return;
    last_delta_pub_time_ = now;

    int W = (int)map_.info.width;
    int H = (int)map_.info.height;

    bool first_time = (prev_map_data_.size() != (size_t)(W * H));
    if (first_time) {
        prev_map_data_.assign(map_.data.begin(), map_.data.end());
    }

    nav_msgs::msg::OccupancyGrid delta = map_;
    delta.header.stamp = now;
    delta.data.assign(W * H, -1);

    int changed = 0;
    for (int i = 0; i < W * H; ++i) {
        if (first_time || map_.data[i] != prev_map_data_[i]) {
            delta.data[i] = map_.data[i];
            prev_map_data_[i] = map_.data[i];
            ++changed;
        }
    }

    if (changed == 0) return;

    delta.header.frame_id = map_frame_;

    map_delta_pub_->publish(delta);
}

    void FrontierExplorerMulti::onGateGoal(
      const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
      double dist = 999.0;

      if (has_gate_goal_) {
          double dx = msg->pose.position.x - gate_goal_.pose.position.x;
          double dy = msg->pose.position.y - gate_goal_.pose.position.y;
          dist = std::hypot(dx, dy);
      }

      // 병합 이후 gate goal은 world 좌표다. 로봇 바로 옆의 frontier를
      // 다시 배정하면 짧은 FollowPath가 즉시 성공하고 재할당이 반복된다.
      if (!using_local_map_ && has_pose_) {
          const double robot_dist = std::hypot(
              msg->pose.position.x - robot_.x,
              msg->pose.position.y - robot_.y);
          if (robot_dist < gate_goal_min_distance_m_) {
              return;
          }
      }

      // 실행 중인 유효 경로는 유지한다. Gate는 매 tick frontier 대표점을
      // 다시 계산하므로 작은 지도 변화가 새 goal/cancel 폭주로 이어질 수 있다.
      if (!path_.empty() && has_gate_goal_ &&
          dist > gate_goal_switch_distance_m_) {
          return;
      }

      last_gate_goal_time_ = this->now();

      if (!has_gate_goal_ || dist > gate_goal_switch_distance_m_) {
          new_gate_goal_ = true;

      }

      gate_goal_ = *msg;
      has_gate_goal_ = true;
  }


  bool FrontierExplorerMulti::toGlobal(double x_local, double y_local, double& x_g, double& y_g) {
    geometry_msgs::msg::PoseStamped in;
    
    in.header.stamp = this->now();
    in.header.frame_id = map_frame_;
    in.pose.position.x = x_local;
    in.pose.position.y = y_local;
    in.pose.position.z = 0.0;
    in.pose.orientation.w = 1.0;

    try {
      auto tf = tf_buffer_->lookupTransform(
        global_frame_, map_frame_, tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_s_)
      );
      geometry_msgs::msg::PoseStamped out;
      tf2::doTransform(in, out, tf);
      x_g = out.pose.position.x;
      y_g = out.pose.position.y;
      return true;
    } catch (...) {
      return false;
    }
  }

  bool FrontierExplorerMulti::toLocal(double x_g, double y_g, double& x_local, double& y_local) {
    geometry_msgs::msg::PoseStamped in;
    in.header.stamp = this->now();
    in.header.frame_id = global_frame_;
    in.pose.position.x = x_g;
    in.pose.position.y = y_g;
    in.pose.position.z = 0.0;
    in.pose.orientation.w = 1.0;

    try {
        auto tf = tf_buffer_->lookupTransform(
            map_frame_, global_frame_, tf2::TimePointZero,
            tf2::durationFromSec(tf_timeout_s_));
        geometry_msgs::msg::PoseStamped out;
        tf2::doTransform(in, out, tf);
        x_local = out.pose.position.x;
        y_local = out.pose.position.y;
        return true;
    } catch (...) {
        return false;
    }
  }

  bool FrontierExplorerMulti::updateRobotPoseFromTF() {
    try {
      const auto tf = tf_buffer_->lookupTransform(
        map_frame_, base_frame_, tf2::TimePointZero
      );
      robot_.x = tf.transform.translation.x;
      robot_.y = tf.transform.translation.y;
      robot_.yaw = tf2::getYaw(tf.transform.rotation);

      has_pose_ = true;
      return true;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "[%s] robot pose TF unavailable: %s", robot_id_.c_str(), ex.what());
      has_pose_ = false;
      return false;
    }
  }

  bool FrontierExplorerMulti::isTraversable(int x, int y) const {
    if (!inBounds(x,y)) return false;
    int v = map_.data[IDX(x, y, (int)map_.info.width)];
    if (v == UNKNOWN) return false;
    return (v >= 0 && v <= free_threshold_);
  }

  bool FrontierExplorerMulti::isFrontierCell(int x, int y) const {
    if (!isTraversable(x,y)) return false;
    for (int k=0;k<8;k++){
      int nx=x+dx8[k], ny=y+dy8[k];
      if (!inBounds(nx,ny)) continue;
      int nv = map_.data[IDX(nx, ny, (int)map_.info.width)];
      if (nv == UNKNOWN) return true;
    }
    return false;
  }

  std::vector<GridPose> FrontierExplorerMulti::detectFrontiers(
      const GridPose &robot_g, double search_radius_m) const {
    std::vector<GridPose> out;
    int W = (int)map_.info.width;
    int H = (int)map_.info.height;

    const bool full_map = search_radius_m <= 0.0;
    int r_cells = full_map
        ? std::max(W, H)
        : (int)std::ceil(search_radius_m / map_.info.resolution);
    int x0 = full_map ? 0 : std::max(0, robot_g.x - r_cells);
    int x1 = full_map ? W - 1 : std::min(W-1, robot_g.x + r_cells);
    int y0 = full_map ? 0 : std::max(0, robot_g.y - r_cells);
    int y1 = full_map ? H - 1 : std::min(H-1, robot_g.y + r_cells);

    int excl = (int)std::ceil(0.6/map_.info.resolution);


    for (int y=y0;y<=y1;y++){
      for (int x=x0;x<=x1;x++){
        int dx = x - robot_g.x;
        int dy = y - robot_g.y;
        if (dx*dx + dy*dy <= excl*excl) continue;
        if (!full_map && dx*dx + dy*dy > r_cells*r_cells) continue;
        if (isFrontierCell(x,y)) out.push_back({x,y});
      }
    }
    return out;
  }

  void FrontierExplorerMulti::applyKeepOpen(std::vector<uint8_t>& mask, const GridPose& robot_g) const {
    int W = (int)map_.info.width;
    if (inBounds(robot_g.x, robot_g.y)) {
      mask[IDX(robot_g.x, robot_g.y, W)] = 0;
    }
    for (int dy=-keep_open_cells_; dy<=keep_open_cells_; ++dy) {
      for (int dx=-keep_open_cells_; dx<=keep_open_cells_; ++dx) {
        int nx = robot_g.x + dx;
        int ny = robot_g.y + dy;
        if (!inBounds(nx, ny)) continue;

        int v = map_.data[IDX(nx, ny, W)];
        if (v >= 0 && v <= free_threshold_) {
        mask[IDX(nx, ny, W)] = 0;
      }

      }
    }
  }

  void FrontierExplorerMulti::onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    last_scan_ = *msg;
    has_scan_ = true;

    if (!has_map_ || !has_pose_) return;

    int W = (int)map_.info.width;
    int H = (int)map_.info.height;

    if ((int)laser_blocked_.size() != W*H) laser_blocked_.assign(W*H, 0);
    else std::fill(laser_blocked_.begin(), laser_blocked_.end(), 0);

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
      if (!std::isfinite(r) || r > laser_obstacle_max_range_) continue;

      double a = msg->angle_min + i * msg->angle_increment;

      geometry_msgs::msg::PointStamped p_scan, p_map;
      p_scan.header = msg->header;
      p_scan.point.x = r * std::cos(a);
      p_scan.point.y = r * std::sin(a);
      p_scan.point.z = 0.0;

      tf2::doTransform(p_scan, p_map, tf_scan_to_map);
      GridPose g = worldToGrid(p_map.point.x, p_map.point.y);
      if (!inBounds(g.x, g.y)) continue;

      for (int dy=-lrad; dy<=lrad; ++dy) {
        for (int dx=-lrad; dx<=lrad; ++dx) {
          int nx = g.x + dx;
          int ny = g.y + dy;
          if (!inBounds(nx, ny)) continue;

          double dist = std::sqrt((double)dx*dx + (double)dy*dy) * map_.info.resolution;
          if (dist <= laser_inflation_radius_m_) {
            laser_blocked_[IDX(nx, ny, W)] = 1;
          }
        }
      }
    }

    last_laser_update_ = std::chrono::steady_clock::now();
  }

  std::vector<uint8_t> FrontierExplorerMulti::buildObstacleInflatedMask(bool include_laser) const {
    int W = (int)map_.info.width;
    int H = (int)map_.info.height;
    std::vector<uint8_t> obs(W*H, 0);
    std::vector<uint8_t> blocked(W*H, 0);

    for (int y=0;y<H;y++){
      for (int x=0;x<W;x++){
        int v = map_.data[IDX(x,y,W)];
        const int id = IDX(x,y,W);
        if (v == UNKNOWN) {
          // A path through unobserved space is not physically validated.  The
          // frontier target itself is a known-free cell adjacent to unknown.
          blocked[id] = 1;
        } else if (v >= obstacle_threshold_) {
          obs[id] = 1;
          blocked[id] = 1;
        }
      }
    }

    int rad = (int)std::ceil(inflation_radius_m_ / map_.info.resolution);
    if (rad <= 0) return blocked;

    std::vector<uint8_t> inflated = blocked;
    for (int y=0;y<H;y++){
      for (int x=0;x<W;x++){
        if (!obs[IDX(x,y,W)]) continue;
        for (int dy=-rad; dy<=rad; dy++){
          for (int dx=-rad; dx<=rad; dx++){
            int nx=x+dx, ny=y+dy;
            if (!inBounds(nx,ny)) continue;
            double dist = std::sqrt((double)dx*dx + (double)dy*dy) * map_.info.resolution;
            if (dist <= inflation_radius_m_) inflated[IDX(nx,ny,W)] = 1;
          }
        }
      }
    }

    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_laser_update_).count();

    if (include_laser && !laser_blocked_.empty() && dt < laser_block_ttl_) {
      GridPose robot_g = worldToGrid(robot_.x, robot_.y);
      int keep = 2; 

      for (int i=0; i<W*H; ++i) {
        if (!laser_blocked_[i]) continue;

        int x = i % W;
        int y = i / W;

        if (std::abs(x - robot_g.x) <= keep &&
            std::abs(y - robot_g.y) <= keep)
          continue;

        inflated[i] = 1;
      }
    }

    return inflated;
  }

  std::vector<uint8_t> FrontierExplorerMulti::buildObstacleRawMask() const {
    int W = (int)map_.info.width;
    int H = (int)map_.info.height;
    std::vector<uint8_t> obs(W*H, 0);
    for (int y=0;y<H;y++){
      for (int x=0;x<W;x++){
        int v = map_.data[IDX(x,y,W)];
        if (v != UNKNOWN && v >= obstacle_threshold_) obs[IDX(x,y,W)] = 1;
      }
    }
    return obs;
  }

  std::vector<int> FrontierExplorerMulti::buildClearanceCostMap(
      const std::vector<uint8_t>& obsRaw) const {
    const int W = (int)map_.info.width;
    const int H = (int)map_.info.height;
    std::vector<int> costs(W * H, 0);
    if (path_clearance_m_ <= inflation_radius_m_ ||
        path_clearance_cost_weight_ <= 0) {
      return costs;
    }

    const double res = map_.info.resolution;
    const int radius_cells = (int)std::ceil(path_clearance_m_ / res);
    const double soft_width = path_clearance_m_ - inflation_radius_m_;
    for (int y = 0; y < H; ++y) {
      for (int x = 0; x < W; ++x) {
        if (!obsRaw[IDX(x, y, W)]) continue;
        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
          for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            const int nx = x + dx;
            const int ny = y + dy;
            if (!inBounds(nx, ny)) continue;
            const double distance = std::hypot(dx, dy) * res;
            if (distance <= inflation_radius_m_ || distance > path_clearance_m_) continue;
            const double ratio = (path_clearance_m_ - distance) / soft_width;
            const int penalty = (int)std::ceil(path_clearance_cost_weight_ * ratio);
            costs[IDX(nx, ny, W)] = std::max(costs[IDX(nx, ny, W)], penalty);
          }
        }
      }
    }
    return costs;
  }

  std::vector<uint8_t> FrontierExplorerMulti::buildBlockedMask() const {
    int W = (int)map_.info.width;
    int H = (int)map_.info.height;
    std::vector<uint8_t> blocked(W*H, 0);

    std::vector<uint8_t> unknown(W*H, 0);
    std::vector<uint8_t> obs(W*H, 0);

    for (int y=0;y<H;y++){
      for (int x=0;x<W;x++){
        int v = map_.data[IDX(x,y,W)];
        if (v == UNKNOWN) unknown[IDX(x,y,W)] = 1;
        if (v >= obstacle_threshold_) obs[IDX(x,y,W)] = 1;
      }
    }

    // rad = 0.20 / 0.05 네칸 정도 인플레이션 주기 ) 장애물에 적용
    int rad = (int)std::ceil(inflation_radius_m_ / map_.info.resolution);

    std::vector<uint8_t> inflated_obs = obs;
    if (rad > 0) {
      for (int y=0;y<H;y++){
        for (int x=0;x<W;x++){
          if (!obs[IDX(x,y,W)]) continue;
          for (int dy=-rad; dy<=rad; dy++){
            for (int dx=-rad; dx<=rad; dx++){
              int nx = x+dx, ny = y+dy;
              if (!inBounds(nx,ny)) continue;

              double dist = std::sqrt((double)dx*dx + (double)dy*dy) * map_.info.resolution;
              if (dist <= inflation_radius_m_) inflated_obs[IDX(nx,ny,W)] = 1;
            }
          }
        }
      }
    }

    for (int i=0;i<W*H;i++) blocked[i] = unknown[i] || inflated_obs[i] ? 1 : 0;

    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_laser_update_).count();

    if (!laser_blocked_.empty() && dt < laser_block_ttl_) {
      GridPose robot_g = worldToGrid(robot_.x, robot_.y);
      int keep = 2;

      for (int i=0; i<W*H; ++i) {
        if (!laser_blocked_[i]) continue;

        int x = i % W;
        int y = i / W;

        if (std::abs(x - robot_g.x) <= keep &&
            std::abs(y - robot_g.y) <= keep)
          continue;

        blocked[i] = 1;
      }
    }

    return blocked;

  }

  void FrontierExplorerMulti::applyOtherRobotFootprints(std::vector<uint8_t>& mask) {
    const auto now = this->now();
    const int width = (int)map_.info.width;
    const int radius_cells = std::max(
        1, (int)std::ceil(other_robot_radius_m_ / map_.info.resolution));

    for (auto it = other_robot_positions_.begin(); it != other_robot_positions_.end();) {
      if ((now - it->second.stamp).seconds() > robot_position_ttl_s_) {
        it = other_robot_positions_.erase(it);
        continue;
      }

      double x_local, y_local;
      if (toLocal(it->second.x, it->second.y, x_local, y_local)) {
        const GridPose other = worldToGrid(x_local, y_local);
        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
          for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            if (std::hypot(dx, dy) * map_.info.resolution > other_robot_radius_m_) continue;
            const int x = other.x + dx;
            const int y = other.y + dy;
            if (inBounds(x, y)) mask[IDX(x, y, width)] = 1;
          }
        }
      }
      ++it;
    }
  }

  bool FrontierExplorerMulti::isFrontierTooCloseToObstacle(const GridPose& f,
                                    const std::vector<uint8_t>& obsRaw,
                                    int radius_cells) const {
    int W = (int)map_.info.width;
    int r2 = radius_cells * radius_cells;
    for (int dy=-radius_cells; dy<=radius_cells; ++dy) {
      for (int dx=-radius_cells; dx<=radius_cells; ++dx) {
        if (dx*dx + dy*dy > r2) continue;
        int nx = f.x + dx;
        int ny = f.y + dy;
        if (!inBounds(nx, ny)) continue;
        if (obsRaw[IDX(nx, ny, W)]) return true;
      }
    }
    return false;
  }

  // Reachable한 지 검사 -> reachable[x,y] == 1 이면 로봇위치에서 (4방향)  막히지 않고 갈 수 있는 칸을 의미
  std::vector<uint8_t> FrontierExplorerMulti::buildReachableMaskFromStart(const GridPose& start,
                                                   const std::vector<uint8_t>& reachmask) const {
    int W = (int)map_.info.width;
    int H = (int)map_.info.height;
    std::vector<uint8_t> reachable(W*H, 0);

    if (!inBounds(start.x, start.y)) return reachable;
    if (reachmask[IDX(start.x, start.y, W)]) return reachable;

    std::queue<GridPose> q;
    q.push(start);
    reachable[IDX(start.x, start.y, W)] = 1;

    while(!q.empty()){
      GridPose c = q.front(); q.pop();
      for(int k=0;k<8;k++){
        int nx = c.x + dx8[k];
        int ny = c.y + dy8[k];
        if (!inBounds(nx, ny)) continue;
        int id = IDX(nx, ny, W);
        if (reachable[id]) continue;
        if (reachmask[id]) continue;
        // no corner-cut for diagonal moves (match A*)
        bool diagonal = (dx8[k] != 0 && dy8[k] != 0);
        if (diagonal) {
          int n1x = c.x + dx8[k];
          int n1y = c.y;
          int n2x = c.x;
          int n2y = c.y + dy8[k];
          if (!inBounds(n1x, n1y) || !inBounds(n2x, n2y)) continue;

          int n1 = IDX(n1x, n1y, W);
          int n2 = IDX(n2x, n2y, W);
          if (reachmask[n1] || reachmask[n2]) continue;
        }

        reachable[id] = 1;
        q.push({nx, ny});
      }
    }
    return reachable;
  }

  void FrontierExplorerMulti::filterFrontiersByReachable(std::vector<GridPose>& frontiers,
                                  const std::vector<uint8_t>& reachable) const {
    int W = (int)map_.info.width;
    frontiers.erase(
      std::remove_if(frontiers.begin(), frontiers.end(),
        [&](const GridPose& f){
          return !reachable[IDX(f.x, f.y, W)];
        }),
      frontiers.end()
    );
  }

  std::vector<GridPose> FrontierExplorerMulti::astar(const GridPose &start, const GridPose &goal,
                             const std::vector<uint8_t> &astarMask) const
  {
    int W = (int)map_.info.width;
    int H = (int)map_.info.height;

    auto inside = [&](int x,int y){ return (0<=x && x<W && 0<=y && y<H); };
    if (!inside(start.x,start.y) || !inside(goal.x,goal.y)) return {};
    if (astarMask[IDX(start.x, start.y, W)]) return {};
    if (astarMask[IDX(goal.x, goal.y, W)]) return {};
    if (astarMask[IDX(start.x,start.y,W)] || astarMask[IDX(goal.x,goal.y,W)]) return {};


    auto h = [&](int x,int y){
      int dx = std::abs(x - goal.x);
      int dy = std::abs(y - goal.y);
      int mn = std::min(dx, dy);
      int mx = std::max(dx, dy);
      return 14*mn + 10*(mx - mn);
    };

    struct Node { int f,g,x,y; };
    struct Cmp { bool operator()(const Node& a, const Node& b) const { return a.f > b.f; } };

    std::priority_queue<Node, std::vector<Node>, Cmp> pq;
    std::vector<int> came(W*H, -1);
    std::vector<int> gscore(W*H, std::numeric_limits<int>::max());

    int sid = IDX(start.x,start.y,W);
    gscore[sid] = 0;
    pq.push({h(start.x,start.y), 0, start.x, start.y});

    while(!pq.empty()){
      Node cur = pq.top(); pq.pop();
      int id = IDX(cur.x,cur.y,W);
      if (cur.g != gscore[id]) continue;

      if (cur.x == goal.x && cur.y == goal.y) {
        std::vector<GridPose> path;
        int cid = id;
        while (cid != -1) {
          int px = cid % W;
          int py = cid / W;
          path.push_back({px,py});
          cid = came[cid];
        }
        std::reverse(path.begin(), path.end());
        return simplifyPath(path, astarMask);
      }

      for (int k=0;k<8;k++){
        int nx = cur.x + dx8[k];
        int ny = cur.y + dy8[k];
        if (!inside(nx,ny)) continue;

        int nid = IDX(nx,ny,W);
        if (astarMask[nid]) continue;

        bool diagonal = (dx8[k] != 0 && dy8[k] != 0);
        int step_cost = diagonal ? 14 : 10;

        int cell = map_.data[nid];
        if (cell == UNKNOWN) step_cost += 10; // unknown은 비용 크게 해서 되도록 안가게하기
        if (cell >= obstacle_threshold_) step_cost += 50; // obs도 비용 크게 해서 되도록 안 가게하기

        // no corner-cut
        if (diagonal) {
          int n1 = IDX(cur.x + dx8[k], cur.y, W);
          int n2 = IDX(cur.x, cur.y + dy8[k], W);
          if (astarMask[n1] || astarMask[n2]) continue;
        }

        int clearance_cost = clearance_cost_map_.size() == map_.data.size()
            ? clearance_cost_map_[nid] : 0;
        int ng = cur.g + step_cost + clearance_cost;
        if (ng < gscore[nid]) {
          gscore[nid] = ng;
          came[nid] = id;
          pq.push({ng + h(nx,ny), ng, nx, ny});
        }
      }
    }
    return {};
  }

  bool FrontierExplorerMulti::lineOfSightCost(
      const GridPose& from, const GridPose& to,
      const std::vector<uint8_t>& obstacle_mask, int& cost) const {
    const int W = (int)map_.info.width;
    const int H = (int)map_.info.height;
    if ((int)obstacle_mask.size() != W * H) return false;

    int x = from.x;
    int y = from.y;
    const int dx = std::abs(to.x - from.x);
    const int dy = std::abs(to.y - from.y);
    const int sx = from.x < to.x ? 1 : -1;
    const int sy = from.y < to.y ? 1 : -1;
    int error = dx - dy;
    cost = 0;

    while (x != to.x || y != to.y) {
      const int previous_x = x;
      const int previous_y = y;
      const int twice_error = 2 * error;
      if (twice_error > -dy) {
        error -= dy;
        x += sx;
      }
      if (twice_error < dx) {
        error += dx;
        y += sy;
      }

      if (x < 0 || x >= W || y < 0 || y >= H ||
          obstacle_mask[IDX(x, y, W)]) {
        return false;
      }

      const bool diagonal = x != previous_x && y != previous_y;
      if (diagonal) {
        // A diagonal segment must not squeeze through blocked cell corners.
        if (obstacle_mask[IDX(x, previous_y, W)] ||
            obstacle_mask[IDX(previous_x, y, W)]) {
          return false;
        }
      }

      int step_cost = diagonal ? 14 : 10;
      if (clearance_cost_map_.size() == map_.data.size()) {
        step_cost += clearance_cost_map_[IDX(x, y, W)];
      }
      cost += step_cost;
    }
    return true;
  }

  std::vector<GridPose> FrontierExplorerMulti::simplifyPath(
      const std::vector<GridPose>& path,
      const std::vector<uint8_t>& obstacle_mask) const {
    if (path.size() <= 2) return path;

    std::vector<int> cumulative_cost(path.size(), 0);
    for (size_t i = 1; i < path.size(); ++i) {
      int edge_cost = 0;
      if (!lineOfSightCost(path[i - 1], path[i], obstacle_mask, edge_cost)) {
        return path;
      }
      cumulative_cost[i] = cumulative_cost[i - 1] + edge_cost;
    }

    std::vector<GridPose> simplified;
    simplified.reserve(path.size());
    simplified.push_back(path.front());
    size_t anchor = 0;
    while (anchor + 1 < path.size()) {
      size_t selected = anchor + 1;
      for (size_t candidate = path.size() - 1; candidate > anchor + 1; --candidate) {
        int shortcut_cost = 0;
        if (!lineOfSightCost(
                path[anchor], path[candidate], obstacle_mask, shortcut_cost)) {
          continue;
        }
        const int original_cost =
            cumulative_cost[candidate] - cumulative_cost[anchor];
        // Do not make a visually straighter path by sacrificing the clearance
        // preference that A* already paid for.
        if (shortcut_cost <= original_cost + 5) {
          selected = candidate;
          break;
        }
      }
      simplified.push_back(path[selected]);
      anchor = selected;
    }
    return simplified;
  }

  double FrontierExplorerMulti::distMeters(const GridPose& a, const GridPose& b) const {
    double dx = (a.x - b.x) * map_.info.resolution;
    double dy = (a.y - b.y) * map_.info.resolution;
    return std::hypot(dx, dy);
  }

  std::vector<int> FrontierExplorerMulti::regionQuery(const std::vector<GridPose>& pts, int idx, double eps_m) const {
    std::vector<int> neighbors;
    neighbors.reserve(64);
    for (int j = 0; j < (int)pts.size(); ++j) {
      if (j == idx) continue;
      if (distMeters(pts[idx], pts[j]) <= eps_m) neighbors.push_back(j);
    }
    return neighbors;
  }

  std::vector<int> FrontierExplorerMulti::dbscanCluster(const std::vector<GridPose>& pts, double eps_m, int min_pts) const {
    const int N = (int)pts.size();
    std::vector<int> labels(N, -2); // -2 unvisited, -1 noise, >=0 cluster id
    int cluster_id = 0;

    for (int i = 0; i < N; ++i) {
      if (labels[i] != -2) continue;
      auto neighbors = regionQuery(pts, i, eps_m);
      if ((int)neighbors.size() + 1 < min_pts) { labels[i] = -1; continue; }

      labels[i] = cluster_id;
      std::queue<int> q;
      for (int nb : neighbors) q.push(nb);

      while (!q.empty()) {
        int p = q.front(); q.pop();

        if (labels[p] == -1) labels[p] = cluster_id;
        if (labels[p] != -2) continue;

        labels[p] = cluster_id;
        auto nbs2 = regionQuery(pts, p, eps_m);
        if ((int)nbs2.size() + 1 >= min_pts) {
          for (int nb2 : nbs2) {
            if (labels[nb2] == -2 || labels[nb2] == -1) q.push(nb2);
          }
        }
      }
      cluster_id++;
    }
    return labels;
  }

  // centroid와 가장 가까운 원본 frontier 점을 대표점으로 삼음
  std::vector<GridPose> FrontierExplorerMulti::computeClusterRepresentatives(const std::vector<GridPose>& pts,
                                                    const std::vector<int>& labels) const
{
  int max_id = -1;
  for (int l : labels) if (l > max_id) max_id = l;
  if (max_id < 0) return {};

  std::vector<std::vector<int>> clusters(max_id + 1);
  for (int i = 0; i < (int)pts.size(); ++i) {
    if (labels[i] >= 0) clusters[labels[i]].push_back(i);
  }

  std::vector<GridPose> reps;
  // 긴 frontier 띠 하나가 DBSCAN cluster 하나가 되더라도 중앙점 하나만
  // 남기지 않는다. 예약 충돌 시 같은 띠의 다른 구간을 선택할 수 있도록
  // 서로 떨어진 복수 후보를 만든다.
  reps.reserve(clusters.size() * 4);
  double res = map_.info.resolution;

  for (const auto& idxs : clusters) {
    // 1. 점 개수 필터링 (dbscan_min_pts_ 활용)
    if (idxs.empty() || (int)idxs.size() < dbscan_min_pts_) continue;

    double sx = 0.0, sy = 0.0;
    for (int id : idxs) { sx += pts[id].x; sy += pts[id].y; }
    double cx = sx / idxs.size();
    double cy = sy / idxs.size();

    // 2. Unknown 영역 필터링 (반지름 필터 대신 사용)
    // 체크 반경은 eps_m보다 조금 넉넉하게 (1.5~2.0배)
    double check_radius_m = dbscan_eps_m_ * 1.8; 
    int check_dist = static_cast<int>(check_radius_m / res);
    int unknown_count = 0;
    int total_scanned = 0;

    for (int dy = -check_dist; dy <= check_dist; ++dy) {
      for (int dx = -check_dist; dx <= check_dist; ++dx) {
        // 원형 범위만 체크
        if (std::hypot(dx, dy) * res > check_radius_m) continue;

        int nx = static_cast<int>(cx) + dx;
        int ny = static_cast<int>(cy) + dy;

        if (nx >= 0 && nx < (int)map_.info.width && ny >= 0 && ny < (int)map_.info.height) {
          if (map_.data[ny * map_.info.width + nx] == -1) { // Unknown 확인
            unknown_count++;
          }
          total_scanned++;
        }
      }
    }

    // 주변에 Unknown이 20%도 안 되면 "이미 다 아는 좁은 틈새"로 판단
    if (total_scanned > 0 && (double)unknown_count / total_scanned < 0.20) {
      continue; 
    }

    // 3. 최적의 대표점(Representative) 계산 (무게중심에서 가장 가까운 실제 점)
    int best = idxs[0];
    double best_d2 = 1e18;
    for (int id : idxs) {
      double dx = pts[id].x - cx;
      double dy = pts[id].y - cy;
      double d2 = dx*dx + dy*dy;
      if (d2 < best_d2) { best_d2 = d2; best = id; }
    }
    std::vector<int> selected{best};
    reps.push_back(pts[best]);

    const double candidate_spacing_m = std::max(0.75, reserve_exclusion_radius_m_ * 0.5);
    constexpr size_t max_candidates_per_cluster = 8;
    while (selected.size() < max_candidates_per_cluster) {
      int farthest = -1;
      double farthest_min_dist = -1.0;
      for (int id : idxs) {
        double min_dist = std::numeric_limits<double>::infinity();
        for (int chosen : selected) {
          min_dist = std::min(min_dist, distMeters(pts[id], pts[chosen]));
        }
        if (min_dist > farthest_min_dist) {
          farthest_min_dist = min_dist;
          farthest = id;
        }
      }
      if (farthest < 0 || farthest_min_dist < candidate_spacing_m) break;
      selected.push_back(farthest);
      reps.push_back(pts[farthest]);
    }
  } 
  return reps;
}

  double FrontierExplorerMulti::infoGainAround(const GridPose& g, int radius_cells) const {
    int W = (int)map_.info.width;
    int tot = 0, unk = 0;

    for (int dy=-radius_cells; dy<=radius_cells; ++dy) {
      for (int dx=-radius_cells; dx<=radius_cells; ++dx) {
        int nx=g.x+dx, ny=g.y+dy;
        if (!inBounds(nx,ny)) continue;
        if (dx*dx + dy*dy > radius_cells*radius_cells) continue;
        tot++;
        if (map_.data[IDX(nx,ny,W)] == UNKNOWN) unk++;
      }
    }
    return (tot>0) ? (double)unk / (double)tot : 0.0;
  }
  

  void FrontierExplorerMulti::publishReservationGlobal(const GridPose& goal_local_g) {
    auto [wx, wy] = gridToWorld(goal_local_g.x, goal_local_g.y);

    double xg, yg;
    if (!toGlobal(wx, wy, xg, yg)) return;

    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = this->now();
    ps.header.frame_id = robot_id_;
    ps.pose.position.x = xg;
    ps.pose.position.y = yg;
    ps.pose.position.z = 0.0;
    ps.pose.orientation.w = 1.0;

    reserve_pub_->publish(ps);
    last_reservation_pub_ = this->now();
  }


  void FrontierExplorerMulti::onReservePoint(const geometry_msgs::msg::PoseStamped& msg, const std::string& sender_id) {
    if (sender_id == robot_id_) return;

    ReservedGoal rg;
    rg.src = sender_id;
    rg.x = msg.pose.position.x;
    rg.y = msg.pose.position.y;
    rg.stamp = this->now();

    reservations_[sender_id] = rg;

  }

  void FrontierExplorerMulti::publishRobotPositionGlobal() {
    if ((this->now() - last_robot_position_pub_).seconds() < robot_position_period_s_) return;
    double x_global, y_global;
    if (!toGlobal(robot_.x, robot_.y, x_global, y_global)) return;

    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = robot_id_;
    msg.pose.position.x = x_global;
    msg.pose.position.y = y_global;
    msg.pose.orientation.w = 1.0;
    robot_position_pub_->publish(msg);
    last_robot_position_pub_ = this->now();
  }

  void FrontierExplorerMulti::onRobotPosition(
      const geometry_msgs::msg::PoseStamped& msg, const std::string& sender_id) {
    if (sender_id.empty() || sender_id == robot_id_) return;
    other_robot_positions_[sender_id] = {
        sender_id, msg.pose.position.x, msg.pose.position.y, this->now()};
  }

  bool FrontierExplorerMulti::hasOtherRobotOnCurrentPath() {
    if (path_.empty()) return false;
    const auto now = this->now();
    const double radius_squared = other_robot_radius_m_ * other_robot_radius_m_;
    for (auto it = other_robot_positions_.begin(); it != other_robot_positions_.end();) {
      if ((now - it->second.stamp).seconds() > robot_position_ttl_s_) {
        it = other_robot_positions_.erase(it);
        continue;
      }
      double x_local, y_local;
      if (toLocal(it->second.x, it->second.y, x_local, y_local)) {
        const int start = std::clamp(wp_idx_, 0, static_cast<int>(path_.size()) - 1);
        double checked_distance = 0.0;
        for (int i = start; i < static_cast<int>(path_.size()); ++i) {
          if (i > start) {
            const auto [prev_x, prev_y] = gridToWorld(path_[i - 1].x, path_[i - 1].y);
            const auto [curr_x, curr_y] = gridToWorld(path_[i].x, path_[i].y);
            checked_distance += std::hypot(curr_x - prev_x, curr_y - prev_y);
            if (checked_distance > other_robot_path_lookahead_m_) break;
          }
          const auto& point = path_[i];
          const auto [path_x, path_y] = gridToWorld(point.x, point.y);
          const double dx = path_x - x_local;
          const double dy = path_y - y_local;
          if (dx * dx + dy * dy <= radius_squared) return true;
        }
      }
      ++it;
    }
    return false;
  }

  bool FrontierExplorerMulti::isCurrentPathBlocked(
      const std::vector<uint8_t>& obstacle_mask) {
    if (path_.empty() || obstacle_mask.size() != map_.data.size()) {
      path_blocked_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      return false;
    }

    const int W = (int)map_.info.width;
    const int nearest = findNearestIndexOnPath(path_, wp_idx_, 25);
    double checked_distance = 0.0;
    bool blocked = false;

    // 로봇 발밑을 열어 주는 구간은 건너뛰고, 실제로 주행할 앞쪽 경로만 본다.
    for (size_t i = (size_t)nearest + 1; i < path_.size(); ++i) {
      checked_distance += distMeters(path_[i - 1], path_[i]);
      if (checked_distance > path_blocked_lookahead_m_) break;
      const auto& point = path_[i];
      if (inBounds(point.x, point.y) && obstacle_mask[IDX(point.x, point.y, W)]) {
        blocked = true;
        break;
      }
    }

    const auto now = this->now();
    if (!blocked) {
      path_blocked_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      return false;
    }

    if (path_blocked_since_.nanoseconds() == 0) {
      path_blocked_since_ = now;
      return path_blocked_confirm_s_ <= 0.0;
    }
    return (now - path_blocked_since_).seconds() >= path_blocked_confirm_s_;
  }


  double FrontierExplorerMulti::reservePenaltyGlobal(double goal_x_g, double goal_y_g) {
    auto now = this->now();

    for (auto it = reservations_.begin(); it != reservations_.end();) {
        double diff = (now - it->second.stamp).seconds();
        if (diff > reserve_ttl_s_ || diff < -1.0) {
            it = reservations_.erase(it);
        } else {
            ++it;
        }
    }

    if (reservations_.empty()) {
        return 0.0;
    }

    double max_penalty = 0.0;
    for (const auto& kv : reservations_) {
        // 동시 선택 시 양쪽이 모두 양보하며 진동하지 않도록 이름이 작은
        // 로봇의 예약만 우선권으로 인정한다 (tb3_0 > tb3_1).
        if (kv.first > robot_id_) continue;
        double d = std::hypot(goal_x_g - kv.second.x, goal_y_g - kv.second.y);
        if (d >= reserve_exclusion_radius_m_) continue;
        // 예약 지점에 가까울수록 1.0에 가까운 페널티, 반경 경계에서 0.0
        double p = 1.0 - (d / reserve_exclusion_radius_m_);
        if (p > max_penalty) max_penalty = p;
    }
    return max_penalty;
}

  bool FrontierExplorerMulti::hasHigherPriorityReservation(
      double goal_x_g, double goal_y_g) {
    return reservePenaltyGlobal(goal_x_g, goal_y_g) > 0.9;
  }


  bool FrontierExplorerMulti::pickBestFrontierByUtility(
    const GridPose &robot_g,
    const std::vector<GridPose> &reps,
    const std::vector<uint8_t> &obsInfl,
    GridPose &out_goal,
    std::vector<GridPose> &out_path
    )
  {

    if (reps.empty()) return false;

    int ig_cells = (int)std::ceil(info_gain_radius_m_ / map_.info.resolution);
    double bestScore = -1e18;
    bool found = false;
    int rejected_traversable = 0;
    int rejected_blocked = 0;
    int rejected_near = 0;
    int rejected_blacklist = 0;
    int rejected_tf = 0;
    int rejected_reservation = 0;
    int rejected_astar = 0;
    const bool rendezvous_active = has_rendezvous_anchor_ && !rendezvous_arrived_ &&
        (this->now() - last_rendezvous_anchor_time_).seconds() <=
            rendezvous_command_ttl_s_ &&
        (rendezvous_anchor_.header.frame_id.empty() ||
            rendezvous_anchor_.header.frame_id == map_frame_);
    const double robot_anchor_distance = rendezvous_active
        ? std::hypot(robot_.x - rendezvous_anchor_.pose.position.x,
                     robot_.y - rendezvous_anchor_.pose.position.y)
        : 0.0;

    for (const auto& rep : reps) {
      const auto& g = rep;
    
    if (!isTraversable(g.x, g.y)) { ++rejected_traversable; continue; }
    
    if (obsInfl[IDX(g.x,g.y,(int)map_.info.width)]) { ++rejected_blocked; continue; }
    
    double min_goal_dist_m = 0.6;  
    double d0 = std::hypot(
      (g.x - robot_g.x) * map_.info.resolution,
      (g.y - robot_g.y) * map_.info.resolution
    );
    if (d0 < min_goal_dist_m) { ++rejected_near; continue; }

    if (isBlacklisted(g)) { ++rejected_blacklist; continue; }


    // 병합 전에는 world -> local_map TF가 아직 존재하지 않는다.
    // 따라서 로컬 맵 fallback 중에는 global reservation을 적용하지 않고
    // 각 로봇이 자기 로컬 맵 안에서 독립적으로 탐사한다.
    double rp = 0.0;
    if (!using_local_map_) {
      auto [wx, wy] = gridToWorld(g.x, g.y);
      double gx, gy;
      if (!toGlobal(wx, wy, gx, gy)) { ++rejected_tf; continue; }

      rp = reservePenaltyGlobal(gx, gy);
      if (rp > 0.9) {
        ++rejected_reservation;
        continue;
      }
    }

    auto astarMask = obsInfl;
    applyKeepOpen(astarMask, robot_g);
  
    auto p = astar(robot_g, g, astarMask);
    if (p.empty()) { ++rejected_astar; continue; }


    double path_len = 0.0;
    for (size_t i = 1; i < p.size(); ++i) {
      path_len += distMeters(p[i - 1], p[i]);
    }
    double ig = infoGainAround(g, ig_cells);

    double score = alpha_ * ig - beta_ * path_len - delta_ * rp;
    if (rendezvous_active) {
      const auto [goal_x, goal_y] = gridToWorld(g.x, g.y);
      const double goal_anchor_distance = std::hypot(
          goal_x - rendezvous_anchor_.pose.position.x,
          goal_y - rendezvous_anchor_.pose.position.y);
      // 앵커에 가까워지는 진행량에 보상을 준다. 기존 정보이득/경로길이
      // 점수는 유지하므로 장애물을 뚫고 앵커 좌표로 직행하지 않는다.
      score += rendezvous_utility_weight_ *
          (robot_anchor_distance - goal_anchor_distance);
    }

    if (score > bestScore) {
      bestScore = score;
      out_goal = g;
      out_path = std::move(p);
      found = true;
    }
  }

    if (!found) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "[%s] no plan detail: reps=%zu traversable=%d blocked=%d near=%d "
          "blacklist=%d tf=%d reservation=%d astar=%d",
          robot_id_.c_str(), reps.size(), rejected_traversable,
          rejected_blocked, rejected_near, rejected_blacklist, rejected_tf,
          rejected_reservation, rejected_astar);
    }
    return found;
  }
  

  double FrontierExplorerMulti::minRange(double a_min, double a_max) const {
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

  void FrontierExplorerMulti::publishStop(const char* reason) {
    clearPathAndCancel();
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_pub_->publish(cmd);

    (void)reason;
  }

  int FrontierExplorerMulti::findNearestIndexOnPath(const std::vector<GridPose>& path, int start_idx, int window) {
    if (path.empty() || !has_map_) return 0;
    const int N = (int)path.size();
    int i0 = std::max(0, start_idx - window);
    int i1 = std::min(N - 1, start_idx + window);

    int best_i = i0;
    double best_d2 = 1e18;

    for (int i = i0; i <= i1; ++i) {
      auto [wx, wy] = gridToWorld(path[i].x, path[i].y);
      double dx = wx - robot_.x;
      double dy = wy - robot_.y;
      double d2 = dx*dx + dy*dy;
      if (d2 < best_d2) { best_d2 = d2; best_i = i; }
    }
    return best_i;
  }

  void FrontierExplorerMulti::followPathStep() {
    if (path_.empty() || path_sent_to_dwb_) return;
    if (!follow_path_client_->action_server_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "[%s] DWB FollowPath action server '%s' is not ready",
          robot_id_.c_str(), follow_path_action_name_.c_str());
      return;
    }

    const auto path_id = ++active_path_id_;
    dwb_motion_cmd_seen_ = false;
    FollowPath::Goal goal;
    auto nav_path = makeNavPath();
    RCLCPP_WARN(
        get_logger(),
        "[%s] SEND PATH frame=%s size=%zu first=(%.2f, %.2f) last=(%.2f, %.2f)",
        robot_id_.c_str(), nav_path.header.frame_id.c_str(), nav_path.poses.size(),
        nav_path.poses.front().pose.position.x,
        nav_path.poses.front().pose.position.y,
        nav_path.poses.back().pose.position.x,
        nav_path.poses.back().pose.position.y);
    RCLCPP_WARN(
        get_logger(), "[%s] ROBOT pose=(%.2f, %.2f)",
        robot_id_.c_str(), robot_.x, robot_.y);
    goal.path = std::move(nav_path);
    goal.controller_id = "FollowPath";
    goal.goal_checker_id = "general_goal_checker";
    auto options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
    options.goal_response_callback = [this, path_id](FollowPathGoalHandle::SharedPtr handle) {
      if (!handle) {
        if (path_id == active_path_id_) path_sent_to_dwb_ = false;
        RCLCPP_ERROR(get_logger(), "[%s] DWB rejected FollowPath goal", robot_id_.c_str());
        return;
      }
      if (path_id == active_path_id_) follow_path_goal_handle_ = handle;
    };
    const GridPose sent_goal = current_goal_;
    options.result_callback = [this, path_id, sent_goal](
        const FollowPathGoalHandle::WrappedResult & result) {
      if (path_id != active_path_id_) return;
      follow_path_goal_handle_.reset();
      path_sent_to_dwb_ = false;
      if (result.code == rclcpp_action::ResultCode::ABORTED) {
        addToBlacklist(sent_goal, dwb_failure_blacklist_ttl_s_,
                       dwb_failure_blacklist_radius_m_);
        RCLCPP_WARN(
            get_logger(),
            "[%s] DWB aborted goal (%d, %d); blacklist %.2fm for %.1fs",
            robot_id_.c_str(), sent_goal.x, sent_goal.y,
            dwb_failure_blacklist_radius_m_, dwb_failure_blacklist_ttl_s_);
      }
      path_.clear();
      has_goal_ = false;
      path_blocked_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        // 도착한 gate goal을 다음 timer에서 다시 계획하지 않는다.
        has_gate_goal_ = false;
        new_gate_goal_ = false;
        if (following_rendezvous_ && has_rendezvous_anchor_) {
          const double anchor_distance = std::hypot(
              rendezvous_anchor_.pose.position.x - robot_.x,
              rendezvous_anchor_.pose.position.y - robot_.y);
          if (anchor_distance <= rendezvous_arrival_radius_m_) {
            rendezvous_arrived_ = true;
            following_rendezvous_ = false;
            RCLCPP_INFO(
                get_logger(),
                "[%s] rendezvous anchor reached after FollowPath success (%.3f m)",
                robot_id_.c_str(), anchor_distance);
          } else {
            RCLCPP_INFO(
                get_logger(),
                "[%s] intermediate rendezvous path reached; anchor still %.3f m away",
                robot_id_.c_str(), anchor_distance);
          }
        }
      }
      if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_WARN(get_logger(), "[%s] DWB FollowPath ended with code %d",
                    robot_id_.c_str(), static_cast<int>(result.code));
      }
    };
    path_sent_to_dwb_ = true;
    follow_path_client_->async_send_goal(goal, options);
  }

  void FrontierExplorerMulti::onDwbCmd(
      const geometry_msgs::msg::Twist::SharedPtr msg) {
    last_dwb_cmd_ = *msg;
    last_dwb_cmd_time_ = this->now();
    if (std::hypot(msg->linear.x, msg->linear.y) > 0.005 ||
        std::abs(msg->angular.z) > 0.01) {
      dwb_motion_cmd_seen_ = true;
    }
  }

  void FrontierExplorerMulti::cancelDwbGoal() {
    ++active_path_id_;
    path_sent_to_dwb_ = false;
    if (follow_path_goal_handle_) {
      follow_path_client_->async_cancel_goal(follow_path_goal_handle_);
      follow_path_goal_handle_.reset();
    }
  }

  void FrontierExplorerMulti::clearPathAndCancel() {
    path_blocked_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    if (path_.empty() && !path_sent_to_dwb_ && !follow_path_goal_handle_) return;
    cancelDwbGoal();
    path_.clear();
    wp_idx_ = 0;
    has_goal_ = false;
  }

  nav_msgs::msg::Path FrontierExplorerMulti::makeNavPath() const {
    nav_msgs::msg::Path nav_path;
    nav_path.header.stamp = this->now();
    nav_path.header.frame_id = map_frame_;
    nav_path.poses.reserve(path_.size());
    for (size_t i = 0; i < path_.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = nav_path.header;
      const auto xy = gridToWorld(path_[i].x, path_[i].y);
      pose.pose.position.x = xy.first;
      pose.pose.position.y = xy.second;
      double yaw = robot_.yaw;
      if (i + 1 < path_.size()) {
        const auto next = gridToWorld(path_[i + 1].x, path_[i + 1].y);
        yaw = std::atan2(next.second - xy.second, next.first - xy.first);
      }
      pose.pose.orientation.z = std::sin(0.5 * yaw);
      pose.pose.orientation.w = std::cos(0.5 * yaw);
      nav_path.poses.push_back(std::move(pose));
    }
    return nav_path;
  }

  geometry_msgs::msg::Twist FrontierExplorerMulti::applyDynamicSafetyFilter(
      geometry_msgs::msg::Twist cmd) const {
    const double direction_range = cmd.linear.x >= 0.0
        ? minRange(-0.45, 0.45) : minRange(2.4, 3.14);
    if (direction_range < dynamic_stop_distance_) return geometry_msgs::msg::Twist();
    if (direction_range < dynamic_slow_distance_) {
      const double scale = (direction_range - dynamic_stop_distance_) /
          (dynamic_slow_distance_ - dynamic_stop_distance_);
      cmd.linear.x *= clampd(scale, 0.0, 1.0);
    }
    cmd.linear.x = clampd(cmd.linear.x, -dynamic_max_linear_speed_, dynamic_max_linear_speed_);
    cmd.angular.z = clampd(cmd.angular.z, -dynamic_max_angular_speed_, dynamic_max_angular_speed_);
    return cmd;
  }

  bool FrontierExplorerMulti::updateDynamicController() {
    // obsCallback이 장애물을 현재 활성 맵 프레임으로 변환한다.
    // 따라서 MERGE(world)와 LOCAL(robot/map) 모드 모두에서 동적 회피를 사용한다.
    if (path_.empty()) return false;
    size_t nearest = 0;
    double best_distance = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < path_.size(); ++i) {
      const auto xy = gridToWorld(path_[i].x, path_[i].y);
      const double distance = std::hypot(xy.first - robot_.x, xy.second - robot_.y);
      if (distance < best_distance) {
        best_distance = distance;
        nearest = i;
      }
    }
    const size_t target = std::min(nearest + 7, path_.size() - 1);
    dynamic_controller_->pose_update(robot_.x, robot_.y, robot_.yaw);
    dynamic_controller_->goal_update(gridToWorld(path_[target].x, path_[target].y));
    if (!dynamic_controller_->has_collision_risk()) return false;

    auto cmd = applyDynamicSafetyFilter(dynamic_controller_->control_cmd_update());
    dynamic_cmd_pub_->publish(cmd);
    cmd_pub_->publish(cmd);
    return true;
  }

  void FrontierExplorerMulti::obsCallback(const frontier_ws::msg::DynamicObstacle::SharedPtr msg){
    if (msg->header.frame_id.empty() || msg->header.frame_id == map_frame_) {
      dynamic_controller_->obs_update(msg);
      return;
    }

    try {
      const auto tf = tf_buffer_->lookupTransform(
          map_frame_, msg->header.frame_id, tf2::TimePointZero,
          tf2::durationFromSec(tf_timeout_s_));

      geometry_msgs::msg::PointStamped position_in, position_out;
      position_in.header = msg->header;
      position_in.point.x = msg->x;
      position_in.point.y = msg->y;
      position_in.point.z = msg->z;
      tf2::doTransform(position_in, position_out, tf);

      geometry_msgs::msg::Vector3Stamped velocity_in, velocity_out;
      velocity_in.header = msg->header;
      velocity_in.vector.x = msg->vx;
      velocity_in.vector.y = msg->vy;
      velocity_in.vector.z = msg->vz;
      tf2::doTransform(velocity_in, velocity_out, tf);

      auto transformed = std::make_shared<frontier_ws::msg::DynamicObstacle>(*msg);
      transformed->header.frame_id = map_frame_;
      transformed->x = position_out.point.x;
      transformed->y = position_out.point.y;
      transformed->z = position_out.point.z;
      transformed->vx = velocity_out.vector.x;
      transformed->vy = velocity_out.vector.y;
      transformed->vz = velocity_out.vector.z;
      dynamic_controller_->obs_update(transformed);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[%s] obstacle TF unavailable (%s -> %s): %s",
          robot_id_.c_str(), msg->header.frame_id.c_str(), map_frame_.c_str(), ex.what());
    }
  }

  bool FrontierExplorerMulti::isRobotStuck() {
    // DWB가 경로를 수락하고 실제 속도 명령을 만들 시간을 먼저 보장한다.
    // 유효 속도를 한 번도 만들지 못한 경우에는 Nav2 progress checker가
    // 원인을 보존한 채 abort하도록 두고 frontier가 경로를 선제 취소하지 않는다.
    if ((this->now() - goal_commit_start_).seconds() < stuck_grace_s_ ||
        !dwb_motion_cmd_seen_) {
      return false;
    }
    if (!progress_inited_) {
      resetStuckCheck();
      return false;
    }

    double moved = std::hypot(robot_.x - last_progress_x_, robot_.y - last_progress_y_);
    
    if (moved > stuck_min_move_m_) {
      resetStuckCheck();
      return false;
    } 
    // 못 움직이고 있을 때만 시간 체크
    double dt = (this->now() - last_progress_time_).seconds();
    return (dt > stuck_timeout_s_);
  }

  void FrontierExplorerMulti::resetStuckCheck() {
    last_progress_time_ = this->now();
    last_progress_x_ = robot_.x;
    last_progress_y_ = robot_.y;
    progress_inited_ = true;
  }

  bool FrontierExplorerMulti::shouldReplanByIG() {
    if (!has_goal_ || !has_map_) return false;
    if (path_.empty()) return false;

    auto now = this->now();

    if ((now - last_replan_check_).seconds() < replan_check_period_s_) return false;
    last_replan_check_ = now;

    // 예약 충돌은 commit time보다 먼저 해소한다. 동시 선택 시 우선순위가
    // 낮은 로봇이 예약 메시지를 받는 즉시 다른 후보로 이동한다.
    if (!has_gate_goal_ && !using_local_map_) {
      auto [wx, wy] = gridToWorld(current_goal_.x, current_goal_.y);
      double gx, gy;
      if (toGlobal(wx, wy, gx, gy) && reservePenaltyGlobal(gx, gy) > 0.5) {
        return true;
      }
    }

    int ig_radius = (int)std::ceil(info_gain_radius_m_ / map_.info.resolution);
    double ig = infoGainAround(current_goal_, ig_radius);
    const double goal_age_s = (now - goal_commit_start_).seconds();

    // 목표 선정 당시보다 unknown 비율이 크게 감소했다면 다른 로봇이 먼저
    // 탐사했거나 현재 센서로 이미 밝혀진 곳이므로 끝까지 가지 않는다.
    const bool ig_sharply_dropped =
        goal_age_s >= ig_replan_min_age_s_ &&
        goal_initial_ig_ >= ig_drop_baseline_min_ &&
        ig <= goal_initial_ig_ * ig_drop_ratio_;
    if (ig_sharply_dropped) {
        return true;
    }

    if (goal_age_s < min_commit_time_s_) return false;

    if (ig < ig_drop_thresh_) {
        return true;
    }

    return false;
  }


  void FrontierExplorerMulti::publishPathMarker(const std::vector<GridPose>& path) {
    if (!enable_viz_ || !has_map_ || !path_marker_pub_ || path.empty()) return;

    visualization_msgs::msg::Marker m;
    m.header.stamp = this->now();
    m.header.frame_id = map_frame_;
    m.ns = robot_id_ + "_path";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.03;
    m.color.r = 0.0f; m.color.g = 1.0f; m.color.b = 0.0f; m.color.a = 1.0f;

    m.points.reserve(path.size());
    for (const auto& gp : path) {
      auto [wx, wy] = gridToWorld(gp.x, gp.y);
      geometry_msgs::msg::Point pt;
      pt.x = wx; pt.y = wy; pt.z = 0.05;
      m.points.push_back(pt);
    }
    path_marker_pub_->publish(m);
  }

  void FrontierExplorerMulti::publishFrontierMarkers(const std::vector<GridPose>& frontiers) {
    if (!enable_viz_ || !has_map_ || !frontier_marker_pub_) return;

    visualization_msgs::msg::Marker m;
    m.header.stamp = this->now();
    m.header.frame_id = map_frame_;
    m.ns = robot_id_ + "_frontiers";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.05; m.scale.y = 0.05;
    m.color.r = 1.0f; m.color.g = 0.0f; m.color.b = 0.0f; m.color.a = 1.0f;

    m.points.reserve(frontiers.size());
    for (const auto& f : frontiers) {
      auto [wx, wy] = gridToWorld(f.x, f.y);
      geometry_msgs::msg::Point p;
      p.x = wx; p.y = wy; p.z = 0.05;
      m.points.push_back(p);
    }
    frontier_marker_pub_->publish(m);
  }

  void FrontierExplorerMulti::publishInflationMaskMarker(const std::vector<uint8_t>& obsInfl, const GridPose& center_g) {
    if (!enable_viz_ || !infl_marker_pub_ || !has_map_ || obsInfl.empty()) return;

    const int W = (int)map_.info.width;
    const int H = (int)map_.info.height;
    const double res = map_.info.resolution;

    double viz_radius_m = 3.0;
    int r = (int)std::ceil(viz_radius_m / res);
    int x0 = std::max(0, center_g.x - r);
    int x1 = std::min(W-1, center_g.x + r);
    int y0 = std::max(0, center_g.y - r);
    int y1 = std::min(H-1, center_g.y + r);

    visualization_msgs::msg::Marker m;
    m.header.stamp = this->now();
    m.header.frame_id = map_frame_;
    m.ns = robot_id_ + "_infl";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::CUBE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;

    m.scale.x = res;
    m.scale.y = res;
    m.scale.z = 0.02;

    m.color.r = 0.6f;
    m.color.g = 0.0f;
    m.color.b = 0.6f;
    m.color.a = 0.6f;

    for (int y=y0; y<=y1; y+=2) {
      for (int x=x0; x<=x1; x+=2) {
        if (!obsInfl[IDX(x,y,W)]) continue;
        auto [wx, wy] = gridToWorld(x,y);
        geometry_msgs::msg::Point p;
        p.x = wx; p.y = wy; p.z = 0.01;
        m.points.push_back(p);
      }
    }

    infl_marker_pub_->publish(m);
  }

  void FrontierExplorerMulti::publishClusterMarkers(
      const std::vector<GridPose>& pts, const std::vector<int>& labels) {
    if (!enable_viz_ || !has_map_ || !cluster_marker_pub_) return;

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    int max_label = -1;
    for (int label : labels) max_label = std::max(max_label, label);
    std::vector<GridPose> centers;
    if (max_label >= 0 && labels.size() == pts.size()) {
      std::vector<double> sum_x(max_label + 1, 0.0);
      std::vector<double> sum_y(max_label + 1, 0.0);
      std::vector<int> counts(max_label + 1, 0);
      for (size_t i = 0; i < pts.size(); ++i) {
        if (labels[i] < 0) continue;
        sum_x[labels[i]] += pts[i].x;
        sum_y[labels[i]] += pts[i].y;
        ++counts[labels[i]];
      }
      for (int label = 0; label <= max_label; ++label) {
        if (counts[label] == 0) continue;
        const double center_x = sum_x[label] / counts[label];
        const double center_y = sum_y[label] / counts[label];
        int nearest = -1;
        double nearest_distance = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < pts.size(); ++i) {
          if (labels[i] != label) continue;
          const double distance = std::hypot(
              pts[i].x - center_x, pts[i].y - center_y);
          if (distance < nearest_distance) {
            nearest_distance = distance;
            nearest = static_cast<int>(i);
          }
        }
        if (nearest >= 0) centers.push_back(pts[nearest]);
      }
    }

    std::sort(centers.begin(), centers.end(), [&](const GridPose& a, const GridPose& b) {
      const auto [a_x, a_y] = gridToWorld(a.x, a.y);
      const auto [b_x, b_y] = gridToWorld(b.x, b.y);
      const double a_distance = std::hypot(a_x - robot_.x, a_y - robot_.y);
      const double b_distance = std::hypot(b_x - robot_.x, b_y - robot_.y);
      return a_distance < b_distance;
    });
    constexpr size_t max_visible_clusters = 5;
    if (centers.size() > max_visible_clusters) centers.resize(max_visible_clusters);

    const auto stamp = this->now();
    for (size_t i = 0; i < centers.size(); ++i) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = map_frame_;
      marker.header.stamp = stamp;
      marker.ns = robot_id_ + "_near_clusters";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      const auto [x, y] = gridToWorld(centers[i].x, centers[i].y);
      marker.pose.position.x = x;
      marker.pose.position.y = y;
      marker.pose.position.z = 0.10;
      marker.scale.x = 0.18;
      marker.scale.y = 0.18;
      marker.scale.z = 0.06;
      if (robot_id_ == "tb3_0") {
        marker.color.r = 1.00f; marker.color.g = 0.68f; marker.color.b = 0.78f;
      } else if (robot_id_ == "tb3_1") {
        marker.color.r = 0.72f; marker.color.g = 0.92f; marker.color.b = 0.58f;
      } else {
        marker.color.r = 1.00f; marker.color.g = 0.88f; marker.color.b = 0.52f;
      }
      marker.color.a = 0.95f;
      marker_array.markers.push_back(marker);
    }
    cluster_marker_pub_->publish(marker_array);
  }

  void FrontierExplorerMulti::publishSelectedGoalMarker(const GridPose& goal) {
    if (!enable_viz_ || !has_map_ || !cluster_marker_pub_) return;
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.header.stamp = this->now();
    marker.ns = robot_id_ + "_selected_goal";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    const auto [x, y] = gridToWorld(goal.x, goal.y);
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.12;
    marker.scale.x = 0.30;
    marker.scale.y = 0.30;
    marker.scale.z = 0.08;
    marker.color.r = 1.00f;
    marker.color.g = 0.36f;
    marker.color.b = 0.40f;
    marker.color.a = 1.00f;
    marker_array.markers.push_back(marker);
    cluster_marker_pub_->publish(marker_array);
  }

  

  void FrontierExplorerMulti::onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    merge_map_ = *msg;
    has_merge_map_ = true;
    last_merge_map_time_ = this->now();
  }

  void FrontierExplorerMulti::onLocalMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    local_map_ = *msg;
    has_local_map_ = true;
    last_local_map_time_ = this->now();
  }

  void FrontierExplorerMulti::onRendezvousAnchor(
      const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    const auto now = this->now();
    const bool was_stale = !has_rendezvous_anchor_ ||
        (now - last_rendezvous_anchor_time_).seconds() > rendezvous_command_ttl_s_;
    const double moved = has_rendezvous_anchor_
        ? std::hypot(
            msg->pose.position.x - rendezvous_anchor_.pose.position.x,
            msg->pose.position.y - rendezvous_anchor_.pose.position.y)
        : std::numeric_limits<double>::infinity();
    rendezvous_anchor_ = *msg;
    last_rendezvous_anchor_time_ = now;
    has_rendezvous_anchor_ = true;
    if (was_stale || moved > 0.5) {
      rendezvous_arrived_ = false;
      rendezvous_replan_requested_ = true;
      RCLCPP_INFO(
          get_logger(), "[%s] rendezvous anchor active in %s: (%.2f, %.2f)",
          robot_id_.c_str(), msg->header.frame_id.c_str(),
          msg->pose.position.x, msg->pose.position.y);
    }
  }

  void FrontierExplorerMulti::selectActiveMap() {
    auto now = this->now();
    bool merge_fresh = has_merge_map_ &&
        (now - last_merge_map_time_).seconds() < merge_map_stale_s_;

    bool want_local = !merge_fresh && has_local_map_;

    if (want_local != using_local_map_) {
        RCLCPP_WARN(get_logger(), "[%s] map source switch -> %s",
                    robot_id_.c_str(), want_local ? "LOCAL" : "MERGE");
        clearPathAndCancel();
        has_gate_goal_ = false;
        blacklisted_goals_.clear();
        prev_map_data_.clear();
        using_local_map_ = want_local;
    }

    if (using_local_map_) {
        map_ = local_map_;
        map_frame_ = local_map_.header.frame_id.empty()
            ? map_frame_ : local_map_.header.frame_id;
        has_map_ = has_local_map_;
    } else if (merge_fresh) {
        map_ = merge_map_;
        map_frame_ = merge_map_.header.frame_id.empty()
            ? map_frame_ : merge_map_.header.frame_id;
        has_map_ = true;
    } else {
        has_map_ = false;
    }

  }

  void FrontierExplorerMulti::addToBlacklist(
      const GridPose& g, double ttl_s, double radius_m) {
    auto now = this->now();
    const double entry_ttl = ttl_s > 0.0 ? ttl_s : blacklist_ttl_s_;
    const double entry_radius = radius_m > 0.0 ? radius_m : blacklist_radius_m_;

    blacklisted_goals_.erase(
        std::remove_if(blacklisted_goals_.begin(), blacklisted_goals_.end(),
            [&](const BlacklistedGoal& b) {
                const double age = (now - b.stamp).seconds();
                const double distance = std::hypot(
                    (g.x - b.g.x) * map_.info.resolution,
                    (g.y - b.g.y) * map_.info.resolution);
                // Gazebo reset처럼 ROS 시간이 뒤로 가면 과거 blacklist를
                // 유지하지 않는다. 같은 영역의 중복 항목도 하나로 갱신한다.
                return age < -0.1 || age > b.ttl_s ||
                    distance < std::max(entry_radius, b.radius_m);
            }),
        blacklisted_goals_.end());

    blacklisted_goals_.push_back({g, now, entry_ttl, entry_radius});
  }

  bool FrontierExplorerMulti::isBlacklisted(const GridPose& g) const {
      auto now = this->now();
      for (const auto& b : blacklisted_goals_) {
          const double age = (now - b.stamp).seconds();
          if (age < -0.1 || age > b.ttl_s) continue;
          double d = std::hypot((g.x - b.g.x) * map_.info.resolution,
                                (g.y - b.g.y) * map_.info.resolution);
          if (d < b.radius_m) return true;
      }
      return false;
  }

  

  void FrontierExplorerMulti::onTimer() {

    selectActiveMap();
    if (!has_map_) return;


    if (exploration_done_) {
        publishStop("exploration done");
        return;
    }

    if (!updateRobotPoseFromTF()) {
        publishStop("no tf!");
        return;
    }

    publishRobotPositionGlobal();
    publishMapDelta();

    GridPose robot_g = worldToGrid(robot_.x, robot_.y);
    if (!inBounds(robot_g.x, robot_g.y)) {
      publishStop("robot pose out of map bounds");
      return;
    }

    auto obsInfl    = buildObstacleInflatedMask();
    auto obsRaw     = buildObstacleRawMask();
    applyOtherRobotFootprints(obsInfl);
    clearance_cost_map_ = buildClearanceCostMap(obsRaw);
    const bool rendezvous_command_active = has_rendezvous_anchor_ &&
        (this->now() - last_rendezvous_anchor_time_).seconds() <=
            rendezvous_command_ttl_s_ &&
        (rendezvous_anchor_.header.frame_id.empty() ||
            rendezvous_anchor_.header.frame_id == map_frame_);
    const bool rendezvous_active = rendezvous_command_active && !rendezvous_arrived_;

    // DWB의 FollowPath 성공은 현재 proxy goal 도착일 뿐이다. 랑데부의
    // 최종 도착은 실제 UWB anchor와의 거리로 별도 판정한다.
    if (rendezvous_command_active && !rendezvous_arrived_) {
        const double anchor_distance = std::hypot(
            rendezvous_anchor_.pose.position.x - robot_.x,
            rendezvous_anchor_.pose.position.y - robot_.y);
        if (anchor_distance <= rendezvous_arrival_radius_m_) {
            rendezvous_arrived_ = true;
            following_rendezvous_ = false;
            RCLCPP_INFO(
                get_logger(),
                "[%s] rendezvous anchor reached by distance (%.3f m)",
                robot_id_.c_str(), anchor_distance);
        }
    }

    if (rendezvous_command_active && rendezvous_arrived_) {
        if (!path_.empty() || path_sent_to_dwb_ || follow_path_goal_handle_) {
            clearPathAndCancel();
        }
        publishStop("rendezvous reached");
        return;
    }

    if (following_rendezvous_ && !rendezvous_active) {
        clearPathAndCancel();
        following_rendezvous_ = false;
        blacklisted_goals_.clear();
        RCLCPP_INFO(
            get_logger(),
            "[%s] rendezvous released; resuming frontier exploration",
            robot_id_.c_str());
        publishStop("rendezvous released");
    }

    if (!path_.empty()) {
        // Gate 없는 분산 탐사에서는 현재 목표 예약을 TTL보다 빠르게 갱신한다.
        if (!using_local_map_ &&
            (this->now() - last_reservation_pub_).seconds() >= reserve_refresh_period_s_) {
            publishReservationGlobal(current_goal_);
        }
        if (new_gate_goal_) {
            clearPathAndCancel();
            new_gate_goal_ = false;
        }
        else {
            const bool path_blocked = isCurrentPathBlocked(obsInfl);
            const bool stuck = isRobotStuck();
            const bool ig_replan = !rendezvous_active && shouldReplanByIG();
            const bool other_robot_on_path = hasOtherRobotOnCurrentPath();
            bool need_replan = rendezvous_replan_requested_ || path_blocked ||
                ig_replan || other_robot_on_path || stuck;

            if (need_replan) {
                RCLCPP_WARN(
                    get_logger(),
                    "[%s] REPLAN reason: rendezvous=%d blocked=%d ig=%d "
                    "other_robot=%d stuck=%d map=%s path_points=%zu",
                    robot_id_.c_str(), rendezvous_replan_requested_, path_blocked,
                    ig_replan, other_robot_on_path, stuck,
                    using_local_map_ ? "LOCAL" : "MERGE", path_.size());
                if (stuck) addToBlacklist(current_goal_);
                if (path_blocked) {
                    RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 1000,
                        "[%s] current path blocked; replanning",
                        robot_id_.c_str());
                }
                clearPathAndCancel();
                rendezvous_replan_requested_ = false;

                if (has_gate_goal_) {
                    // IG가 다 빠진 gate goal은 버리고 로컬 fallback으로 전환
                    has_gate_goal_ = false;
                }

                publishStop("replan");
            }
            else {
                wp_idx_ = findNearestIndexOnPath(path_, wp_idx_, 25);
                if (updateDynamicController()) return;
                followPathStep();
                if ((this->now() - last_dwb_cmd_time_).seconds() <= dwb_cmd_timeout_s_) {
                  cmd_pub_->publish(last_dwb_cmd_);
                } else {
                  geometry_msgs::msg::Twist stop;
                  cmd_pub_->publish(stop);
                  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                      "[%s] stale DWB cmd_vel; holding stop", robot_id_.c_str());
                }
                return;
            }
        }
    }

    if ((this->now() - last_plan_attempt_).seconds() < plan_retry_period_s_) return;
    last_plan_attempt_ = this->now();
    rendezvous_replan_requested_ = false;

        bool gate_goal_valid = false;

        if (has_gate_goal_ && !using_local_map_) {
            double dt = (this->now() - last_gate_goal_time_).seconds();
            if (dt > gate_timeout_s_) {
                has_gate_goal_ = false;
            } else {
                gate_goal_valid = true;
            }
        } else if (has_gate_goal_ && using_local_map_) {
        }

        if (gate_goal_valid) {

            double lx, ly;
            if (!toLocal(gate_goal_.pose.position.x, gate_goal_.pose.position.y, lx, ly)) {
                RCLCPP_WARN(get_logger(), "[%s] gate goal 좌표 변환 실패", robot_id_.c_str());
                has_gate_goal_ = false;
                publishStop("gate goal transform failed");
                return;
            }

        GridPose goal_g = worldToGrid(lx, ly);
            

          
        if (!inBounds(goal_g.x, goal_g.y)){
          has_gate_goal_ = false;
          publishStop("gate goal out of bounds");
          return;
        }

        auto astarMask = obsInfl;
        applyKeepOpen(astarMask, robot_g);

        auto new_path = astar(robot_g, goal_g, astarMask);

        if (!new_path.empty()) {

            gate_plan_fail_count_ = 0;

            path_ = std::move(new_path);
            wp_idx_ = 0;
            progress_inited_ = false;
            current_goal_ = goal_g;
            has_goal_ = true;
            following_rendezvous_ = false;
            goal_commit_start_ = this->now();
            goal_initial_ig_ = infoGainAround(
                current_goal_, (int)std::ceil(info_gain_radius_m_ / map_.info.resolution));

            if (enable_viz_) publishPathMarker(path_);

            followPathStep();
            return;
        }

        gate_plan_fail_count_++;

        if (gate_plan_fail_count_ >= gate_plan_fail_max_) {
            RCLCPP_WARN(get_logger(), "[%s] gate 연속 실패 한도 초과 -> gate goal 포기",
                        robot_id_.c_str());
            has_gate_goal_ = false;
            gate_plan_fail_count_ = 0;
        }

        publishStop("gate retry");
        return;
    }


    const int clearance_cells = (int)std::ceil(
        frontier_clearance_m_ / map_.info.resolution);
    // Unknown과 장애물을 차단한 최종 A* mask로 reachable을 계산한다.
    auto reachMask = obsInfl;
    applyKeepOpen(reachMask, robot_g);
    const auto reachable = buildReachableMaskFromStart(robot_g, reachMask);

    if (rendezvous_active) {
        const GridPose anchor_g = worldToGrid(
            rendezvous_anchor_.pose.position.x,
            rendezvous_anchor_.pose.position.y);
        GridPose rendezvous_goal = robot_g;
        double best_distance_squared = std::numeric_limits<double>::infinity();
        const int width = static_cast<int>(map_.info.width);
        const int height = static_cast<int>(map_.info.height);

        // The physical anchor may be in an unknown or occupied cell. Target the
        // closest known-free cell that is reachable from the robot instead.
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if (!reachable[IDX(x, y, width)] || !isTraversable(x, y) ||
                    obsInfl[IDX(x, y, width)]) {
                    continue;
                }
                const double dx = static_cast<double>(x - anchor_g.x);
                const double dy = static_cast<double>(y - anchor_g.y);
                const double distance_squared = dx * dx + dy * dy;
                if (distance_squared < best_distance_squared) {
                    best_distance_squared = distance_squared;
                    rendezvous_goal = {x, y};
                }
            }
        }

        auto rendezvous_path = astar(robot_g, rendezvous_goal, reachMask);
        const auto [rendezvous_goal_x, rendezvous_goal_y] = gridToWorld(
            rendezvous_goal.x, rendezvous_goal.y);
        const double rendezvous_goal_distance = std::hypot(
            rendezvous_goal_x - robot_.x,
            rendezvous_goal_y - robot_.y);
        if (!rendezvous_path.empty() &&
            (rendezvous_goal.x != robot_g.x || rendezvous_goal.y != robot_g.y) &&
            rendezvous_goal_distance >= rendezvous_direct_min_distance_m_) {
            path_ = std::move(rendezvous_path);
            wp_idx_ = 0;
            progress_inited_ = false;
            current_goal_ = rendezvous_goal;
            has_goal_ = true;
            following_rendezvous_ = true;
            goal_commit_start_ = this->now();
            goal_initial_ig_ = infoGainAround(
                current_goal_,
                static_cast<int>(std::ceil(info_gain_radius_m_ / map_.info.resolution)));
            blacklisted_goals_.clear();

            RCLCPP_INFO(
                get_logger(),
                "[%s] direct rendezvous plan: goal=(%.2f, %.2f), anchor=(%.2f, %.2f)",
                robot_id_.c_str(), rendezvous_goal_x, rendezvous_goal_y,
                rendezvous_anchor_.pose.position.x,
                rendezvous_anchor_.pose.position.y);
            if (enable_viz_) publishPathMarker(path_);
            followPathStep();
            return;
        }

        // DWB tolerance 근처의 같은 proxy goal을 반복 전송하지 않는다.
        // 일반 frontier 선택으로 내려가 anchor 방향의 지도를 더 확장한다.
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "[%s] rendezvous proxy only %.3f m away; expanding frontier toward anchor",
            robot_id_.c_str(), rendezvous_goal_distance);
    }

    // 가까운 유효 frontier를 우선 사용한다. 가까운 후보가 모두 장애물
    // clearance/reachability 검사에서 탈락한 경우에만 탐색 범위를 넓힌다.
    auto find_valid_frontiers = [&](double radius_m) {
        auto candidates = detectFrontiers(robot_g, radius_m);
        candidates.erase(
            std::remove_if(
                candidates.begin(), candidates.end(),
                [&](const GridPose& f) {
                    return isFrontierTooCloseToObstacle(
                        f, obsRaw, clearance_cells);
                }),
            candidates.end());
        filterFrontiersByReachable(candidates, reachable);
        return candidates;
    };

    auto frontiers = find_valid_frontiers(frontier_search_radius_m_);
    double selected_search_radius = frontier_search_radius_m_;

    if (frontiers.empty() &&
        frontier_extended_search_radius_m_ > frontier_search_radius_m_) {
        selected_search_radius = frontier_extended_search_radius_m_;
        frontiers = find_valid_frontiers(selected_search_radius);
    }

    if (frontiers.empty() && frontier_full_map_fallback_) {
        selected_search_radius = 0.0;
        frontiers = find_valid_frontiers(selected_search_radius);
    }

    if (frontiers.empty()) {
        publishStop("no valid reachable frontiers");
        return;
    }

    if (selected_search_radius != frontier_search_radius_m_) {
        if (selected_search_radius > 0.0) {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "[%s] frontier search expanded to %.1fm",
                robot_id_.c_str(), selected_search_radius);
        } else {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "[%s] frontier search expanded to full map",
                robot_id_.c_str());
        }
    }

    std::vector<GridPose> reps;
    std::vector<int> labels;

    if (use_dbscan_) {

        labels =
            dbscanCluster(
                frontiers,
                dbscan_eps_m_,
                dbscan_min_pts_);

        reps =
            computeClusterRepresentatives(
                frontiers,
                labels);

        if (reps.empty()) {
            constexpr size_t max_fallback_candidates = 80;
            const size_t count = std::min(frontiers.size(), max_fallback_candidates);
            reps.reserve(count);
            for (size_t i = 0; i < count; ++i) {
                const size_t index = count == 1
                    ? 0 : i * (frontiers.size() - 1) / (count - 1);
                reps.push_back(frontiers[index]);
            }
        }

    } else {

        reps = frontiers;
    }

    if (enable_viz_) {
        publishClusterMarkers(frontiers, labels);
    }

    GridPose goal;
    std::vector<GridPose> new_path;

    bool planned =
        pickBestFrontierByUtility(
            robot_g,
            reps,
            obsInfl,
            goal,
            new_path);

    if (!planned) {

        publishStop("no plan");

        return;
    }

    path_ = std::move(new_path);

    wp_idx_ = 0;

    progress_inited_ = false;

    current_goal_ = goal;

    has_goal_ = true;
    following_rendezvous_ = false;

    goal_commit_start_ = this->now();
    goal_initial_ig_ = infoGainAround(
        current_goal_, (int)std::ceil(info_gain_radius_m_ / map_.info.resolution));

    // 병합 전에는 공통 world 좌표계가 없으므로 글로벌 예약을 발행하지 않는다.
    if (!using_local_map_) {
      publishReservationGlobal(goal);
    }

    if (enable_viz_) {
        publishSelectedGoalMarker(goal);
        publishPathMarker(path_);
    }

    followPathStep();
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FrontierExplorerMulti>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
