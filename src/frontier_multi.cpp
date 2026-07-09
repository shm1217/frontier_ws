#include "FrontierExplorerMulti.hpp"


FrontierExplorerMulti ::FrontierExplorerMulti() 
: Node("frontier_explorer_multi")
  {
    declare_params();
    setup_ros_interfaces();

    RCLCPP_INFO(this->get_logger(), "[%s] 탐사 노드 준비 완료", robot_id_.c_str()); 
  }

  void FrontierExplorerMulti::declare_params(){
    // 토픽, 로봇, 프레임의 네임스페이스 지정
    robot_id_ = this->declare_parameter<std::string>("robot_id", "robot1");
    map_topic_ = this->declare_parameter<std::string>("map_topic", "map");
    cmd_topic_ = this->declare_parameter<std::string>("cmd_topic", "cmd_vel");
    scan_topic_ = this->declare_parameter<std::string>("scan_topic", "scan");

    map_frame_  = this->declare_parameter<std::string>("map_frame", "map");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_footprint");
    global_frame_ = this->declare_parameter<std::string>("global_frame", "world");
    tf_timeout_s_ = this->declare_parameter<double>("tf_timeout_s", 0.10);

    // 장애물, free의 임계점
    obstacle_threshold_ = this->declare_parameter<int>("obstacle_threshold", 60);
    free_threshold_     = this->declare_parameter<int>("free_threshold", 50);

    // inflation 반경 및 로봇 주변 반경 frontier 추출
    inflation_radius_m_       = this->declare_parameter<double>("inflation_radius_m", 0.25);
    frontier_search_radius_m_ = this->declare_parameter<double>("frontier_search_radius_m", 6.0);

    // control value
    max_lin_vel_ = this->declare_parameter<double>("max_lin_vel", 0.18);
    max_ang_vel_ = this->declare_parameter<double>("max_ang_vel", 0.60);
    reach_dist_  = this->declare_parameter<double>("reach_dist_m", 0.25);

    // 장애물 회피 시
    avoid_enter_dist_ = this->declare_parameter<double>("avoid_enter_dist", 0.35);
    avoid_exit_dist_  = this->declare_parameter<double>("avoid_exit_dist", 0.50);

    // frontier safety
    frontier_clearance_m_ = this->declare_parameter<double>("frontier_clearance_m", 0.25);
    path_clearance_m_     = this->declare_parameter<double>("path_clearance_m", 0.15);

    // 로봇 발 밑 주변 cell 열어주기
    keep_open_cells_ = this->declare_parameter<int>("keep_open_cells", 2);

    // DBSCAN
    use_dbscan_     = this->declare_parameter<bool>("use_dbscan", true);
    dbscan_eps_m_   = this->declare_parameter<double>("dbscan_eps_m", 0.25);
    dbscan_min_pts_ = this->declare_parameter<int>("dbscan_min_pts", 10);
 
    // Laser mask
    laser_block_ttl_ = this->declare_parameter<double>("laser_block_ttl", 1.0);
    laser_inflation_radius_m_ = this->declare_parameter<double>("laser_inflation_radius_m", 0.12);

    // Stuck 됐을 때
    stuck_timeout_s_ = this->declare_parameter<double>("stuck_timeout_s", 3.0);
    stuck_min_move_m_ = this->declare_parameter<double>("stuck_min_move_m", 0.05);

    // Debug viz on/off (마커 퍼블리시는 유지)
    enable_viz_ = this->declare_parameter<bool>("enable_viz", true);

    // ====== Multi-robot utility params ======
    utility_radius_m_ = this->declare_parameter<double>("utility_radius_m", 1.00);
    info_gain_radius_m_ = this->declare_parameter<double>("info_gain_radius_m", 1.50);

    alpha_ = this->declare_parameter<double>("alpha_info_gain", 2.0);
    beta_  = this->declare_parameter<double>("beta_path_len", 1.0);
    delta_ = this->declare_parameter<double>("delta_reserve", 15.0);

    reserve_exclusion_radius_m_ = this->declare_parameter<double>("reserve_exclusion_radius_m", 2.0);
    reserve_ttl_s_ = this->declare_parameter<double>("reserve_ttl_s", 6.0);
    reserve_out_topic_ = this->declare_parameter<std::string>("reserve_out_topic", "/global_goal_reservation");

    path_marker_topic_     = this->declare_parameter<std::string>("path_marker_topic", "path_marker");
    frontier_marker_topic_ = this->declare_parameter<std::string>("frontier_marker_topic", "frontier_markers");
    infl_marker_topic_     = this->declare_parameter<std::string>("infl_marker_topic", "inflation_marker");
    cluster_marker_topic_  = this->declare_parameter<std::string>("cluster_marker_topic", "cluster_marker");
    
    // ====== Gate 관련 파라미터 추가 ======
    gate_goal_topic_   = this->declare_parameter<std::string>("gate_goal_topic", "/goal_assignment");
    map_delta_topic_   = this->declare_parameter<std::string>("map_delta_topic", "map_delta");
    gate_timeout_s_    = this->declare_parameter<double>("gate_timeout_s", 3.0);
    map_delta_period_s_= this->declare_parameter<double>("map_delta_period_s", 1.0);

  }

  void FrontierExplorerMulti::setup_ros_interfaces(){
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, map_qos, std::bind(&FrontierExplorerMulti::onMap, this, std::placeholders::_1));
    
      auto scan_qos = rclcpp::QoS(rclcpp::KeepLast(10));
      scan_qos.best_effort();

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, scan_qos, std::bind(&FrontierExplorerMulti::onScan, this, std::placeholders::_1));

    // Gate로 맵 델타 퍼블리시
    map_delta_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        map_delta_topic_, rclcpp::QoS(1).reliable().durability_volatile());

    // Gate로부터 goal 수신
    gate_goal_sub_ =
    this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/" + robot_id_ + "/goal_assignment",
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
                RCLCPP_WARN(this->get_logger(),
                    "[%s] Exploration DONE 수신",
                    robot_id_.c_str());
            }
        });


    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, 10);

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

    obs_sub_ = 
    this->create_subscription<frontier_ws::msg::DynamicObstacle>(
        "/" + robot_id_ + "/obs_speed", 
        10,std::bind(
            &FrontierExplorerMulti::obsCallback,
            this, 
            std::placeholders::_1));
    controller = std::make_shared<Controller>(this->get_clock());

    replan_check_period_s_ = this->declare_parameter<double>("replan_check_period_s", 0.5);
    min_commit_time_s_     = this->declare_parameter<double>("min_commit_time_s", 2.0);
    ig_drop_thresh_        = this->declare_parameter<double>("ig_drop_thresh", 0.10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
      std::bind(&FrontierExplorerMulti::onTimer, this));

  }


  // ---------- Helpers ----------
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
    // map_delta_period_s_ 간격으로만 보냄
    if ((now - last_delta_pub_time_).seconds() < map_delta_period_s_) return;
    last_delta_pub_time_ = now;

    int W = (int)map_.info.width;
    int H = (int)map_.info.height;

    // 변경된 셀만 추출
    // prev_map_data_가 없으면 전체를 초기 delta로 보냄
    bool first_time = (prev_map_data_.size() != (size_t)(W * H));
    if (first_time) {
        prev_map_data_.assign(map_.data.begin(), map_.data.end());
    }

    // 변경 셀 마스크: 변경된 곳만 원래 값, 나머진 -1(unknown)
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

    if (changed == 0) return; // 변경 없으면 안 보냄

    // frame_id에 robot_id 태깅 (gate가 출처 구분용)
    delta.header.frame_id = map_frame_;
    // RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 2000,
    //     "[%s] Map delta 발송: %d 셀 변경", robot_id_.c_str(), changed);

    map_delta_pub_->publish(delta);
}

    void FrontierExplorerMulti::onGateGoal(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    double dist = 999.0;

    // 이전 goal이 있었으면
    if (has_gate_goal_) {

        double dx =
            msg->pose.position.x -
            gate_goal_.pose.position.x;

        double dy =
            msg->pose.position.y -
            gate_goal_.pose.position.y;

        dist = std::hypot(dx, dy);
    }

    // ------------------------------------------------
    // 진짜 새로운 goal일 때만 timeout 갱신
    // ------------------------------------------------
    if (!has_gate_goal_ || dist > 0.3) {

        last_gate_goal_time_ = this->now();

        new_gate_goal_ = true;

        RCLCPP_WARN(
            get_logger(),
            "[%s] NEW Gate goal received: (%.2f, %.2f)",
            robot_id_.c_str(),
            msg->pose.position.x,
            msg->pose.position.y);
    }

    else {

        RCLCPP_WARN(
            get_logger(),
            "[%s] SAME Gate goal -> timeout 유지",
            robot_id_.c_str());
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
      RCLCPP_ERROR(this->get_logger(), "TF 조회 실패 원인: %s", ex.what());
      has_pose_ = false;
      return false;
    }
  }

  // traversable/free
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

  std::vector<GridPose> FrontierExplorerMulti::detectFrontiers(const GridPose &robot_g) const {
    std::vector<GridPose> out;
    int W = (int)map_.info.width;
    int H = (int)map_.info.height;

    int r_cells = (int)std::ceil(frontier_search_radius_m_ / map_.info.resolution);
    int x0 = std::max(0, robot_g.x - r_cells);
    int x1 = std::min(W-1, robot_g.x + r_cells);
    int y0 = std::max(0, robot_g.y - r_cells);
    int y1 = std::min(H-1, robot_g.y + r_cells);

    // 로봇 주변 제외 반경 셀 -> 자기 자신 근처 셀은 frontier detect 금지
    int excl = (int)std::ceil(0.6/map_.info.resolution);


    for (int y=y0;y<=y1;y++){
      for (int x=x0;x<=x1;x++){
        int dx = x - robot_g.x;
        int dy = y - robot_g.y;
        if (dx*dx + dy*dy <= excl*excl) continue; // 로봇 발밑/근처 프론티어 제거!
        if (isFrontierCell(x,y)) out.push_back({x,y});
      }
    }
    return out;
  }

  void FrontierExplorerMulti::applyKeepOpen(std::vector<uint8_t>& mask, const GridPose& robot_g) const {
    int W = (int)map_.info.width;
    for (int dy=-keep_open_cells_; dy<=keep_open_cells_; ++dy) {
      for (int dx=-keep_open_cells_; dx<=keep_open_cells_; ++dx) {
        int nx = robot_g.x + dx;
        int ny = robot_g.y + dy;
        if (!inBounds(nx, ny)) continue;

        // unknown은 열지 않는 걸 기본으로
        int v = map_.data[IDX(nx, ny, W)];
        // but 로봇 완전 근처만 unknown 열어줌!
        bool near = (std::abs(dx) <=1 && std::abs(dy) <= 1);
        if (v == UNKNOWN && !near) continue;

        if (v == UNKNOWN || (v >= 0 && v <= free_threshold_)) {
        mask[IDX(nx, ny, W)] = 0;
      }

      }
    }
  }

  void FrontierExplorerMulti::applyGoalKeepOpen(
    std::vector<uint8_t>& mask,
    const GridPose& goal_g) const
{
    int W = (int)map_.info.width;

    int R = 4;

    for (int dy=-R; dy<=R; ++dy) {
        for (int dx=-R; dx<=R; ++dx) {

            int nx = goal_g.x + dx;
            int ny = goal_g.y + dy;

            if (!inBounds(nx, ny))
                continue;

            int idx = IDX(nx, ny, W);

            int v = map_.data[idx];

            // frontier goal 주변은 unknown/free 모두 열기
            if (v == UNKNOWN ||
                (v >= 0 && v <= free_threshold_))
            {
                mask[idx] = 0;
            }
        }
    }
}

  // Laser to blocked mask
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
      if (!std::isfinite(r) || r > avoid_enter_dist_) continue;

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

  // obs + laser = blocked + inflation
  std::vector<uint8_t> FrontierExplorerMulti::buildObstacleInflatedMask() const {
    int W = (int)map_.info.width;
    int H = (int)map_.info.height;
    std::vector<uint8_t> obs(W*H, 0);

    // unknown 말고 obs만 막기
    for (int y=0;y<H;y++){
      for (int x=0;x<W;x++){
        int v = map_.data[IDX(x,y,W)];
        if (v != UNKNOWN && v >= obstacle_threshold_) obs[IDX(x,y,W)] = 1;
      }
    }

    // obs 인플레이션
    int rad = (int)std::ceil(inflation_radius_m_ / map_.info.resolution);
    if (rad <= 0) return obs;

    std::vector<uint8_t> inflated = obs;
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

    // 실시간 장애물 (laser) 추가
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_laser_update_).count();

    if (!laser_blocked_.empty() && dt < laser_block_ttl_) {
      GridPose robot_g = worldToGrid(robot_.x, robot_.y);
      int keep = 2; 

      for (int i=0; i<W*H; ++i) {
        if (!laser_blocked_[i]) continue;

        int x = i % W;
        int y = i / W;

        // 로봇 바로 주변은 열어둠
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

  // unknown + obs + laser = blocked + inflation
  std::vector<uint8_t> FrontierExplorerMulti::buildBlockedMask() const {
    int W = (int)map_.info.width;
    int H = (int)map_.info.height;
    std::vector<uint8_t> blocked(W*H, 0);

    // unknown
    std::vector<uint8_t> unknown(W*H, 0);
    // obstacles
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

    // laser block 
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_laser_update_).count();

    if (!laser_blocked_.empty() && dt < laser_block_ttl_) {
      GridPose robot_g = worldToGrid(robot_.x, robot_.y);
      int keep = 2;  // start 탈출용 (goal 마스크에서도 유지할지 선택)

      for (int i=0; i<W*H; ++i) {
        if (!laser_blocked_[i]) continue;

        int x = i % W;
        int y = i / W;

        // 로봇 바로 주변은 열어둠(정책 선택)
        if (std::abs(x - robot_g.x) <= keep &&
            std::abs(y - robot_g.y) <= keep)
          continue;

        blocked[i] = 1;  // 레이저로 goal 후보도 막기
      }
    }

    return blocked;

  }

  // goal 검사
  bool FrontierExplorerMulti::isFrontierTooCloseToObstacle(const GridPose& f,
                                    const std::vector<uint8_t>& obsRaw,
                                    int radius_cells) const {
    int W = (int)map_.info.width;
    int r2 = radius_cells * radius_cells;
    for (int dy=-radius_cells; dy<=radius_cells; ++dy) {
      for (int dx=-radius_cells; dx<=radius_cells; ++dx) {
        if (dx*dx + dy*dy > r2) continue; //원형검사
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

  // frontier 각각 f에 대해 reachable[f] == 1인것만 남김 
  // 즉, frontier가 로봇이 갈 수 있는 연결요소 안에 있는지 보는 것 (위에서 검사한 reachablemask로)
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

  // ---- A* ----
  std::vector<GridPose> FrontierExplorerMulti::astar(const GridPose &start, const GridPose &goal,
                             const std::vector<uint8_t> &astarMask) const
  {
    int W = (int)map_.info.width;
    int H = (int)map_.info.height;

    auto inside = [&](int x,int y){ return (0<=x && x<W && 0<=y && y<H); };
    if (!inside(start.x,start.y) || !inside(goal.x,goal.y)) return {};
    if (astarMask[IDX(start.x, start.y, W)]) RCLCPP_WARN(get_logger(), "[%s] A* start is blocked", robot_id_.c_str());
    if (astarMask[IDX(goal.x, goal.y, W)]) RCLCPP_WARN(get_logger(), "[%s] A* goal is blocked", robot_id_.c_str());
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
        return path;
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

        int ng = cur.g + step_cost;
        if (ng < gscore[nid]) {
          gscore[nid] = ng;
          came[nid] = id;
          pq.push({ng + h(nx,ny), ng, nx, ny});
        }
      }
    }
    return {};
  }

  // ---- DBSCAN ----
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
  reps.reserve(clusters.size());
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
    reps.push_back(pts[best]);
  } 
  return reps;
}

  // ---- Utility ----
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

    RCLCPP_INFO(this->get_logger(),
      "[%s][RES_PUB] topic=%s t=%.3f g=(%.2f,%.2f) grid=(%d,%d)",
      robot_id_.c_str(),
      reserve_pub_->get_topic_name(),
      this->now().seconds(),
      xg, yg,
      goal_local_g.x, goal_local_g.y
    );

    reserve_pub_->publish(ps);
  }


  void FrontierExplorerMulti::onReservePoint(const geometry_msgs::msg::PoseStamped& msg, const std::string& sender_id) {
    if (sender_id == robot_id_) return;
    // // 내 reservation 토픽이면 무시
    // if (src_topic == reserve_pub_->get_topic_name()) return;

    ReservedGoal rg;
    rg.src = sender_id; // 전: global_frame-> sender_id
    rg.x = msg.pose.position.x;
    rg.y = msg.pose.position.y;
    rg.stamp = this->now();

    reservations_[sender_id] = rg; // key: topic

    RCLCPP_INFO(this->get_logger(), "다른 로봇(%s)의 예약 확인: (%.2f, %.2f)", 
                sender_id.c_str(), rg.x, rg.y);
  }


  double FrontierExplorerMulti::reservePenaltyGlobal(double goal_x_g, double goal_y_g) {
    auto now = this->now();

    for (auto it = reservations_.begin(); it != reservations_.end();) {
      double diff = (now - it->second.stamp).seconds();
        // 시뮬레이션에서는 가끔 시간이 튀어서 diff가 마이너스가 될 수 있으니 -1.0초까지 허용
        if (diff > reserve_ttl_s_ || diff < -1.0) {
            it = reservations_.erase(it);
        } else {
            ++it;
        }
    }

    if (reservations_.empty()) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[%s] 장부가 텅 비어있음!", robot_id_.c_str());
        return 0.0;
    }

    for (const auto& kv : reservations_) {
      double d = std::hypot(goal_x_g - kv.second.x, goal_y_g - kv.second.y);
      RCLCPP_WARN(this->get_logger(), "검사중: %s 의 예약위치와 후보지 거리 = %.2f", kv.first.c_str(), d);
      if (d < reserve_exclusion_radius_m_) return 1.0;
    }
    return 0.0;
  }


  bool FrontierExplorerMulti::pickBestFrontierByUtility(
    const GridPose &robot_g,
    const std::vector<GridPose> &reps,
    const std::vector<uint8_t> &blockedMask,
    const std::vector<uint8_t> &obsInfl,
    const std::vector<uint8_t> &obsRaw,
    GridPose &out_goal,
    std::vector<GridPose> &out_path
    )
  {
    // blockedmask - goal 검사
    // obsinfl - astar 경로 검사
    // obsraw - frontier clearance, astar 경로 path tail 검사

    RCLCPP_INFO(get_logger(), "계획 수립 시작!");

    if (reps.empty()) return false;

    int ig_cells = (int)std::ceil(info_gain_radius_m_ / map_.info.resolution);
    int v_cells  = (int)std::ceil(utility_radius_m_   / map_.info.resolution);

    double bestScore = -1e18;
    bool found = false;

    for (const auto& rep : reps) {
      const auto& g = rep;
    
    // goal이 탐사 가능영역인지 확인
    if (!isTraversable(g.x, g.y)) continue;
    
    // goal이 unknown, obs, laser인지 확인
    if (blockedMask[IDX(g.x,g.y,(int)map_.info.width)]) continue; 
    
    // 로봇이 goal에 도달했는지 확인
    double min_goal_dist_m = 0.6;  
    double d0 = std::hypot(
      (g.x - robot_g.x) * map_.info.resolution,
      (g.y - robot_g.y) * map_.info.resolution
    );
    if (d0 < min_goal_dist_m) continue;

    // 블랙리스트 체크
    if (blacklisted_goal_.x >= 0) {
        double d_black = std::hypot(
            (g.x - blacklisted_goal_.x) * map_.info.resolution,
            (g.y - blacklisted_goal_.y) * map_.info.resolution);
        if (d_black < blacklist_radius_m_) continue;
    }

    //  A* 돌리기 전에 좌표 변환부터 먼저
    auto [wx, wy] = gridToWorld(g.x, g.y);
    double gx, gy;
    if (!toGlobal(wx, wy, gx, gy)) continue; 

    // 남이 예약했는지 확인해서, 예약된 곳이면 아예 여기서 컷
    double rp = reservePenaltyGlobal(gx, gy);
    if (rp > 0.5) { 
        // 예약된 곳이면 굳이 A* 경로를 안 짜고 다음 프론티어 후보로
        continue; 
    }

    // 통과한 안전한(예약 안 된) 목표점만 A* 로 경로 생성
    auto astarMask = obsInfl;
    applyKeepOpen(astarMask, robot_g);
  
    auto p = astar(robot_g, g, astarMask);
    if (p.empty()) continue; 


    // utility 활용을 위한 penalty 계산
    double path_len = (double)p.size() * map_.info.resolution;
    double ig = infoGainAround(g, ig_cells);

    // rp 계산은 위에서 이미 했으므로 아래 코드 수정
    double score = alpha_ * ig - beta_ * path_len; 

    // 계산한 score를 기반으로 가장 높은 score를 가지는 goal 값을 선택하여 생성된 경로로 따라가기
    if (score > bestScore) {
      bestScore = score;
      out_goal = g;
      out_path = std::move(p);
      found = true;
    }
  }

    RCLCPP_WARN(this->get_logger(),
    "[%s] pickBest: out_goal=(%d,%d)",
    robot_id_.c_str(), out_goal.x, out_goal.y);

    return found;
  }
  

  // ---- Motion ----
  double FrontierExplorerMulti::normAngle(double a) {
    while (a > M_PI) a -= 2.0*M_PI;
    while (a < -M_PI) a += 2.0*M_PI;
    return a;
  }

  double FrontierExplorerMulti::minRange(double a_min, double a_max) {
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
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_pub_->publish(cmd);

    RCLCPP_ERROR(get_logger(),  "[%s] stop reason=%s", 
        robot_id_.c_str(), reason);
  }

  void FrontierExplorerMulti::publishAvoidCmd() {
    geometry_msgs::msg::Twist cmd;

    // 1. 센서 데이터 세분화
    double front = minRange(-0.2, 0.2); // 정면
    double left  = minRange(0.2, 1.2);  // 왼쪽 사선
    double right = minRange(-1.2, -0.2); // 오른쪽 사선
    double back  = minRange(2.5, 3.14);  // 뒤쪽 (LaserScan 마다 인덱스 확인 필요)

    // 2. 사방이 꽉 막힌 경우 -> 후진
    if (front < 0.3 && left < 0.3 && right < 0.3) {
        cmd.linear.x = -0.08; // 천천히 후진
        cmd.angular.z = 0.0;
        cmd_pub_->publish(cmd);
        return;
    }

    // 3. 회피 방향 결정 (경로 데이터 'path_' 활용)
    bool turn_left = (left > right);
    
    // 만약 좌우 차이가 크지 않다면 (예: 10% 이내), 현재 경로의 방향을 참고
    if (std::abs(left - right) < 0.1 && !path_.empty() && (size_t)wp_idx_ < path_.size()) {
        // 1. 현재 경로상의 목표점 좌표 (Grid -> World 변환 필요)
        auto wp_world = gridToWorld(path_[wp_idx_].x, path_[wp_idx_].y);
        double target_x = wp_world.first;
        double target_y = wp_world.second;

        // 2. 로봇에서 목표점까지의 각도(Absolute Bearing)
        double angle_to_target = std::atan2(target_y - robot_.y, target_x - robot_.x);

        // 3. 로봇의 현재 헤딩을 기준으로 한 상대 각도 (Relative Bearing)
        // normAngle을 사용하여 -PI ~ PI 사이로 정규화합니다.
        double relative_angle = normAngle(angle_to_target - robot_.yaw);

        // 4. 상대 각도가 양수(+)이면 왼쪽, 음수(-)이면 오른쪽에 목표가 있음
        if (relative_angle > 0) {
            turn_left = true;  // 목표가 왼쪽에 있으니 왼쪽으로 회피하는 게 경로 복귀에 유리
        } else {
            turn_left = false; // 목표가 오른쪽에 있으니 오른쪽으로 회피
        }
    }

    // 4. PID 제어 적용 (장애물과의 거리를 일정하게 유지하며 회피)
    double target_dist = 0.5; // 장애물로부터 떨어지고 싶은 거리 (Setpoint)
    double current_dist = turn_left ? right : left; // 반대편 장애물 기준 혹은 빈공간 기준
    
    double error = target_dist - current_dist;
    double e_derivative = error - e_prev_;
    
    // PID 출력 계산 (조향 속도)
    double output = (Kp * error) + (Kd * e_derivative);
    
    cmd.linear.x = 0.05; // 전진 속도는 낮게 유지
    cmd.angular.z = turn_left ? output : -output; 

    // 값 제한 (Clamp)
    cmd.angular.z = clampd(cmd.angular.z, -0.25, 0.25);
    
    e_prev_ = error;
    cmd_pub_->publish(cmd);
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

    if (path_.empty()) return;


    wp_idx_ = findNearestIndexOnPath(path_, wp_idx_, 25);
    int target_idx = std::min(wp_idx_ + 7, (int)path_.size() - 1);
    const auto& target = path_[target_idx];
    // auto [tx, ty] = gridToWorld(target.x, target.y); 
    controller -> goal_update(gridToWorld(target.x, target.y));

    geometry_msgs::msg::Twist cmd;
    cmd = controller -> control_cmd_update();
    // RCLCPP_INFO(this->get_logger(), " controller의 cmd: (%f, %f)", cmd.linear, cmd.angular);
    cmd_pub_->publish(cmd);

  }

  void FrontierExplorerMulti::obsCallback(const frontier_ws::msg::DynamicObstacle::SharedPtr msg){
    controller -> obs_update(msg);
  }

  bool FrontierExplorerMulti::isRobotStuck() {
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

    // 너무 자주 체크하지 않기
    if ((now - last_replan_check_).seconds() < replan_check_period_s_) return false;
    last_replan_check_ = now;

    // 목표를 잡은지 너무 짧으면 안 바꾸기(덜덜 떨림 방지)
    if ((now - goal_commit_start_).seconds() < min_commit_time_s_) return false;

    // ===== NEW =====
    int ig_radius =
        (int)std::ceil(info_gain_radius_m_
            / map_.info.resolution);

    double ig =
        infoGainAround(current_goal_, ig_radius);

    if (ig < ig_drop_thresh_) {

        RCLCPP_WARN(this->get_logger(),
            "[%s][REPLAN] 정보량 감소 ig=%.2f",
            robot_id_.c_str(), ig);

        return true;
    }

    auto [wx, wy] = gridToWorld(current_goal_.x, current_goal_.y);
    double gx, gy;
    if (!toGlobal(wx, wy, gx, gy)) return false;

    // 남이 내 목표 지점을 예약했는지 확인
    double rp = reservePenaltyGlobal(gx, gy);

    if (rp > 0.5) {
        RCLCPP_WARN(this->get_logger(), "[%s][REPLAN] 내 목표 지점이 선점됨! 경로 재설정.", robot_id_.c_str());
        return true; 
    }
    return false;
  }


  // 경로, 프론티어, 인플레이션 마커 시각화
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

  // inflation marker: obsInfl를 큐브 리스트로 간단히 시각화
  void FrontierExplorerMulti::publishInflationMaskMarker(const std::vector<uint8_t>& obsInfl, const GridPose& center_g) {
    if (!enable_viz_ || !infl_marker_pub_ || !has_map_ || obsInfl.empty()) return;

    const int W = (int)map_.info.width;
    const int H = (int)map_.info.height;
    const double res = map_.info.resolution;

    // 로봇 주변만 시각화 (성능)
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

    // 너무 많으면 무거우니까 2칸씩 샘플링
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

   // 이미 필터링된 결과
  void FrontierExplorerMulti::publishClusterRings(const std::vector<GridPose>& pts,
                         const std::vector<int>& labels,
                         const std::vector<GridPose>& representatives){
    if (!enable_viz_ || !has_map_ || representatives.empty()) return;

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    double res = map_.info.resolution;
    auto now = this->now();

    // 필터링을 통과한 'representatives'를 기준
    for (size_t i = 0; i < representatives.size(); ++i) {
      const auto& rep = representatives[i];

      // 1. 이 대표점(rep)이 속한 클러스터의 원래 점들(pts)을 찾아야 합니다.
      // (DBSCAN 결과인 labels에서 해당 좌표와 일치하는 label을 가진 점들을 모읍니다.)
      
      // A. 실제 모양 (POINTS) 마커 설정
      visualization_msgs::msg::Marker points_marker;
      points_marker.header.frame_id = "world";
      points_marker.header.stamp = now;
      points_marker.ns = robot_id_ + "_full_shape";
      points_marker.id = i;
      points_marker.type = visualization_msgs::msg::Marker::POINTS;
      points_marker.scale.x = res * 1.2; 
      points_marker.scale.y = res * 1.2;

      // B. 중심점 (SPHERE) 마커 설정
      visualization_msgs::msg::Marker sphere_marker;
      sphere_marker.header.frame_id = "world";
      sphere_marker.header.stamp = now;
      sphere_marker.ns = robot_id_ + "_centroid";
      sphere_marker.id = i;
      sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
      sphere_marker.scale.x = 0.25; // 가시성을 위해 살짝 키움
      sphere_marker.scale.y = 0.25;
      sphere_marker.scale.z = 0.05;

      // 색상 설정 (기존 로직 유지)
      if (robot_id_ == "tb3_0") {
        points_marker.color.r = 1.0f; points_marker.color.a = 0.6f;
        sphere_marker.color.r = 1.0f; sphere_marker.color.a = 0.9f;
      } else if (robot_id_ == "tb3_1"){
        points_marker.color.g = 1.0f; points_marker.color.a = 0.6f;
        sphere_marker.color.g = 1.0f; sphere_marker.color.a = 0.9f;
      } else {
        points_marker.color.b = 1.0f; points_marker.color.a = 0.6f;
        sphere_marker.color.b = 1.0f; sphere_marker.color.a = 0.9f;
      }

      // 마커 좌표 설정 (대표점 좌표 직접 사용)
      sphere_marker.pose.position.x = map_.info.origin.position.x + rep.x * res;
      sphere_marker.pose.position.y = map_.info.origin.position.y + rep.y * res;
      sphere_marker.pose.position.z = 0.15;

      // POINTS 데이터 채우기: rep 주변 eps_m 거리 내의 점들만 추가 (간단한 매칭)
      for (size_t j = 0; j < pts.size(); ++j) {
        if (distMeters(rep, pts[j]) <= dbscan_eps_m_ * 1.5) { // 클러스터 반경 내 점들
          geometry_msgs::msg::Point p;
          p.x = map_.info.origin.position.x + pts[j].x * res;
          p.y = map_.info.origin.position.y + pts[j].y * res;
          p.z = 0.1;
          points_marker.points.push_back(p);
        }
      }

      marker_array.markers.push_back(points_marker);
      marker_array.markers.push_back(sphere_marker);
    }

    cluster_marker_pub_->publish(marker_array);
  }

  

  void FrontierExplorerMulti::onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    map_ = *msg;
    has_map_ = true;
    RCLCPP_INFO(this->get_logger(), "메시지 frame_id: %s", msg->header.frame_id.c_str());
  }

  

  void FrontierExplorerMulti::onTimer() {

    if (!has_map_) return;

    if (exploration_done_) {
        publishStop("exploration done");
        return;
    }

    // TF
    if (!updateRobotPoseFromTF()) {
        publishStop("no tf!");
        return;
    }

    controller -> pose_update(robot_.x, robot_.y, robot_.yaw);

    publishMapDelta();

    GridPose robot_g = worldToGrid(robot_.x, robot_.y);

    // -------------------------------------------------
    // obstacle avoidance
    // -------------------------------------------------
    double front = minRange(-0.3, 0.3);

    if (has_scan_ && front < avoid_enter_dist_ && has_goal_) {
        avoiding_ = true;
        path_.clear();
    }

    if (avoiding_) {
        if (has_scan_ && front > avoid_exit_dist_) {
            avoiding_ = false;
        } else {
            publishAvoidCmd();
            return;
        }
    }

    // -------------------------------------------------
    // masks
    // -------------------------------------------------
    auto obsInfl    = buildObstacleInflatedMask();
    auto obsRaw     = buildObstacleRawMask();
    auto blockedMask = buildBlockedMask();

    // -------------------------------------------------
    // path follow
    // -------------------------------------------------
    if (!path_.empty()) {

        // 새 gate goal 들어오면
        // 기존 경로 즉시 폐기
        if (new_gate_goal_) {

            RCLCPP_WARN(
                get_logger(),
                "[%s] 새 Gate goal 수신 -> 기존 경로 폐기",
                robot_id_.c_str());

            path_.clear();
            wp_idx_ = 0;

            new_gate_goal_ = false;
        }

        else{
          bool need_replan = false;

          // Gate goal 사용하는 중이면
          // local IG replanning 금지
          if (has_gate_goal_) {
              need_replan = isRobotStuck();
          }
          else {
              need_replan =
                  shouldReplanByIG() ||
                  isRobotStuck();
          }

          if (need_replan) {

              RCLCPP_WARN(
                  get_logger(),
                  "[%s] replanning...",
                  robot_id_.c_str());
              blacklisted_goal_ = current_goal_;  // ← 추가
              path_.clear();
              wp_idx_ = 0;

              publishStop("replan");

              // ↓ return 하지 않음
              // 아래에서 새 계획 수행
          }
          else {

              wp_idx_ =
                  findNearestIndexOnPath(path_, wp_idx_, 25);

              followPathStep();
              return;
          }
    }
  }

    // =================================================
    // Gate goal 확인
    // =================================================
    bool gate_goal_valid = false;

    if (has_gate_goal_) {

        double dt =
            (this->now() - last_gate_goal_time_).seconds();

        if (dt > gate_timeout_s_) {

            has_gate_goal_ = false;

            RCLCPP_WARN(
                get_logger(),
                "[%s] gate goal timeout -> fallback",
                robot_id_.c_str());

        } else {

            gate_goal_valid = true;
        }
    }

    // =================================================
    // Gate goal 우선 수행
    // =================================================
    if (gate_goal_valid) {

        double lx = gate_goal_.pose.position.x;
        double ly = gate_goal_.pose.position.y;


        GridPose goal_g = worldToGrid(lx, ly);
            

          
        if (!inBounds(goal_g.x, goal_g.y)){
          RCLCPP_WARN(get_logger(), "범위 밖!!!!");
        }

        int idx =
            IDX(goal_g.x,
                goal_g.y,
                (int)map_.info.width);

        RCLCPP_ERROR(
            get_logger(),
            "[%s] gate goal occ=%d infl=%d",
            robot_id_.c_str(),
            map_.data[idx],
            obsInfl[idx]);

        if (inBounds(goal_g.x, goal_g.y))
          {

                auto astarMask = obsInfl;

                applyKeepOpen(astarMask, robot_g);
                applyGoalKeepOpen(astarMask, goal_g);

                RCLCPP_WARN(
                  get_logger(),
                  "[%s] gate astar start=(%d,%d) goal=(%d,%d)",
                  robot_id_.c_str(),
                  robot_g.x,
                  robot_g.y,
                  goal_g.x,
                  goal_g.y);

                auto new_path =
                    astar(robot_g, goal_g, astarMask);

                if (!new_path.empty()) {

                    RCLCPP_INFO(
                        get_logger(),
                        "[%s] gate path success",
                        robot_id_.c_str());

                    path_ = std::move(new_path);

                    wp_idx_ = 0;

                    progress_inited_ = false;

                    current_goal_ = goal_g;

                    has_goal_ = true;

                    goal_commit_start_ = this->now();

                    // ❌ 절대 false 하지 마
                    // has_gate_goal_ = false;

                    if (enable_viz_)
                        publishPathMarker(path_);

                    followPathStep();

                    return;
                }
        }


      RCLCPP_WARN(
          get_logger(),
          "[%s] gate planning failed -> retry",
          robot_id_.c_str());
      has_gate_goal_ = false;  // ← 이거 추가
      publishStop("gate retry");

      return;
    }

    // =================================================
    // 로컬 frontier fallback
    // =================================================

    auto frontiers = detectFrontiers(robot_g);
    RCLCPP_WARN(get_logger(), "[%s] frontiers detected: %d", robot_id_.c_str(), (int)frontiers.size());

    if (frontiers.empty()) {

        publishStop("no frontiers");

        return;
    }

    // RCLCPP_WARN(
    //         get_logger(),
    //         "로컬 경로 추종 중");

    int clearance_cells =
        (int)std::ceil(
            frontier_clearance_m_ /
            map_.info.resolution);

    frontiers.erase(
        std::remove_if(
            frontiers.begin(),
            frontiers.end(),
            [&](const GridPose& f){

                return isFrontierTooCloseToObstacle(
                    f,
                    obsRaw,
                    clearance_cells);
            }),
        frontiers.end());
     
      RCLCPP_WARN(get_logger(), "[%s] after clearance: %d", robot_id_.c_str(), (int)frontiers.size());

    if (frontiers.empty()) {

        publishStop("no filtered frontiers");

        return;
    }

    auto reachMask = obsInfl;

    applyKeepOpen(reachMask, robot_g);

    auto reachable =
        buildReachableMaskFromStart(
            robot_g,
            reachMask);

    filterFrontiersByReachable(
        frontiers,
        reachable);
        RCLCPP_WARN(get_logger(), "[%s] after reachable: %d", robot_id_.c_str(), (int)frontiers.size());

    if (frontiers.empty()) {

        publishStop("no reachable");

        return;
    }

    // DBSCAN
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

            publishStop("no cluster");

            return;
        }

    } else {

        reps = frontiers;
    }

    if (enable_viz_) {

        publishClusterRings(
            frontiers,
            labels,
            reps);

        publishFrontierMarkers(reps);
    }

    GridPose goal;
    std::vector<GridPose> new_path;

    bool planned =
        pickBestFrontierByUtility(
            robot_g,
            reps,
            blockedMask,
            obsInfl,
            obsRaw,
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

    goal_commit_start_ = this->now();

    publishReservationGlobal(goal);

    if (enable_viz_)
        publishPathMarker(path_);

    followPathStep();
}
