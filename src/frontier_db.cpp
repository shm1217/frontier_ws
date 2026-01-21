// dbscan clustering으로 프론티어 대표점 추출 후 경로 추종 // 싱글 로봇
// 장애물 회피 + 장애물과 가까운 프론티어셀은 제거하는 식으로 작동

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <cmath>
#include <vector>
#include <queue>
#include <algorithm>
#include <limits>
#include <string>

static constexpr int UNKNOWN = -1;

struct GridPose { int x{0}, y{0}; };
struct WorldPose { double x{0}, y{0}, yaw{0}; };

static inline int IDX(int x, int y, int w) { return y * w + x; }
static inline double clampd(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

static const int dx8[8] = { 1, 1, 0,-1,-1,-1, 0, 1};
static const int dy8[8] = { 0, 1, 1, 1, 0,-1,-1,-1};

static const int dx4[4] = { 1,-1, 0, 0};
static const int dy4[4] = { 0, 0, 1,-1};

class FrontierExplorerAStar : public rclcpp::Node {
public:
  FrontierExplorerAStar() : Node("frontier_explorer_astar") {
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

    map_frame_                = this->declare_parameter<std::string>("map_frame", "map");
    base_frame_               = this->declare_parameter<std::string>("base_frame", "base_footprint");
    tf_timeout_s_             = this->declare_parameter<double>("tf_timeout_s", 0.1);

    laser_inflation_radius_m_ = this->declare_parameter<double>("laser_inflation_radius_m", 0.12);

    // 프론티어가 장애물과 가까우면 제거할 때 활용
    frontier_clearance_m_     = this->declare_parameter<double>("frontier_clearance_m", 0.30);

    // 경로가 안전한지 검사할 때 활용
    path_clearance_m_         = this->declare_parameter<double>("path_clearance_m", 0.12);

    infl_viz_radius_m_        = this->declare_parameter<double>("infl_viz_radius_m", 3.0);
    infl_marker_pub_          = this->create_publisher<visualization_msgs::msg::Marker>("/i", 10);

    // 로봇 스턱을 줄이기 위해서
    stuck_time_s_ = this->declare_parameter<double>("stuck_time_s", 2.0);
    stuck_dist_m_ = this->declare_parameter<double>("stuck_dist_m", 0.05);
    last_progress_time_ = this->now();
    last_progress_x_ = robot_.x;
    last_progress_y_ = robot_.y;
    last_progress_wp_ = 0;

    use_dbscan_      = this->declare_parameter<bool>("use_dbscan", true);
    dbscan_eps_m_    = this->declare_parameter<double>("dbscan_eps_m", 0.20);
    dbscan_min_pts_  = this->declare_parameter<int>("dbscan_min_pts", 6);

    // 초기 맵 탐색 (탐색 목표 지정하지 않고, 어느정도 맵 업데이트를 해두기)
    warmup_enable_ = this->declare_parameter<bool>("warmup_enable", true);
    warmup_min_known_cells_ = this->declare_parameter<int>("warmup_min_known_cells", 12000);
    warmup_min_occ_cells_   = this->declare_parameter<int>("warmup_min_occ_cells", 50);
    warmup_radius_m_        = this->declare_parameter<double>("warmup_radius_m", 2.0);
    warmup_yaw_rate_        = this->declare_parameter<double>("warmup_yaw_rate", 0.6);
    warmup_lin_vel_         = this->declare_parameter<double>("warmup_lin_vel", 0.0); // 기본은 제자리 회전 추천
    warmup_max_time_s_      = this->declare_parameter<double>("warmup_max_time_s", 6.0);
    warmup_state_start_     = this->now();


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
    path_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/p", 10);
    frontier_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/f", 10);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&FrontierExplorerAStar::onScan, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&FrontierExplorerAStar::onTimer, this)
    );

    RCLCPP_WARN(this->get_logger(),
      "Started map_frame=%s base_frame=%s free<=%d blocked>=%d infl=%.2fm frontierR=%.1fm",
      map_frame_.c_str(), base_frame_.c_str(),
      free_threshold_, obstacle_threshold_, inflation_radius_m_, frontier_search_radius_m_);
  }

private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr frontier_marker_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string map_topic_;
  std::string cmd_topic_;
  std::string map_frame_;
  std::string base_frame_;
  double tf_timeout_s_{0.1};

  nav_msgs::msg::OccupancyGrid map_;
  bool has_map_{false};

  sensor_msgs::msg::LaserScan last_scan_;
  bool has_scan_{false};

  std::vector<uint8_t> laser_blocked_;
  std::chrono::steady_clock::time_point last_laser_update_{};
  double laser_block_ttl_{1.0};
  double laser_inflation_radius_m_{0.12};

  WorldPose robot_{};
  bool has_pose_{false};

  std::vector<GridPose> path_;
  bool done_{false};

  int obstacle_threshold_{70};
  int free_threshold_{60};
  double inflation_radius_m_{0.25};
  double frontier_search_radius_m_{8.0};

  double lookahead_m_{0.35};
  double goal_tol_m_{0.35};
  double max_lin_vel_{0.25};
  double max_ang_vel_{0.90};

  double frontier_clearance_m_{0.30};
  double path_clearance_m_{0.12};

  int wp_idx_{0};
  double reach_dist_{0.12};

  bool avoiding_{false};
  double AVOID_DIST{0.45};
  double AVOID_END_DIST{0.60};
  
  double infl_viz_radius_m_{3.0};
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr infl_marker_pub_;

  rclcpp::Time last_progress_time_;
  double last_progress_x_{0.0}, last_progress_y_{0.0};
  int last_progress_wp_{0};
  double stuck_time_s_{2.0};      // 2초 이상 진전 없으면 재플랜
  double stuck_dist_m_{0.05};     // 5cm 미만이면 정체로 간주

  double dbscan_eps_m_{0.20};     // 클러스터 반경 (meters)
  int    dbscan_min_pts_{6};      // 최소 이웃 수 (points)
  bool   use_dbscan_{true};       // 켜고 끄기

  bool warmup_enable_{true};
  int warmup_min_known_cells_{12000};
  int warmup_min_occ_cells_{50};
  double warmup_radius_m_{2.0};
  double warmup_yaw_rate_{0.6};
  double warmup_lin_vel_{0.0};
  double warmup_max_time_s_{6.0};
  rclcpp::Time warmup_state_start_;
  bool warmup_done_{false};


  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    map_ = *msg;
    has_map_ = true;
  }

  bool inBounds(int x, int y) const {
    return (0 <= x && x < (int)map_.info.width && 0 <= y && y < (int)map_.info.height);
  }

  GridPose worldToGrid(double wx, double wy) const {
    const auto &info = map_.info;
    int gx = (int)std::floor((wx - info.origin.position.x) / info.resolution);
    int gy = (int)std::floor((wy - info.origin.position.y) / info.resolution);
    return {gx, gy};
  }

  std::pair<double,double> gridToWorld(int gx, int gy) const {
    const auto &info = map_.info;
    double wx = info.origin.position.x + (gx + 0.5) * info.resolution;
    double wy = info.origin.position.y + (gy + 0.5) * info.resolution;
    return {wx, wy};
  }

  bool updateRobotPoseFromTF() {
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

  
  // 통과 가능한 셀인지 검사 (0이상, free영역 임계값 이하로 설정)
  bool isTraversable(int x, int y) const {
    if (!inBounds(x,y)) return false;
    int v = map_.data[IDX(x, y, (int)map_.info.width)];
    if (v == UNKNOWN) return false;
    return (v >= 0 && v <= free_threshold_);
  }

  // 프론티어 셀인지 검사
  bool isFrontierCell(int x, int y) const {
    if (!isTraversable(x,y)) return false;
    for (int k=0;k<8;k++){
      int nx=x+dx8[k], ny=y+dy8[k];
      if (!inBounds(nx,ny)) continue;
      int nv = map_.data[IDX(nx, ny, (int)map_.info.width)];
      if (nv == UNKNOWN) return true;
    }
    return false;
  }

  // 프론티어 셀 out에 저장하기
  std::vector<GridPose> detectFrontiers(const GridPose &robot_g) const {
    std::vector<GridPose> out;
    int W = (int)map_.info.width;
    int H = (int)map_.info.height;

    // 여기서 r_cell = 0.30 / 0.05 6칸 (map res = 0.05로 확인하였음!)
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

  // 레이저로 스캔 값 확인 -> 로봇이 움직일 때마다 흔들리지 않도록 map frame에 고정시키기 위해 tf 변환 활용
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg){
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
      if (!std::isfinite(r) || r > AVOID_DIST) continue;

      double a = msg->angle_min + i * msg->angle_increment;

      double lx = r * std::cos(a);
      double ly = r * std::sin(a);

      geometry_msgs::msg::PointStamped p_scan, p_map;
      p_scan.header = msg->header;
      p_scan.point.x = lx;
      p_scan.point.y = ly;
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

  
  // blockedMask: unknown + 장애물 + 레이저 -> 경로 계획할 때 활용!!
  std::vector<uint8_t> buildInflatedBlockedMask() const {
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
        else if (v >= obstacle_threshold_) obs[IDX(x,y,W)] = 1;
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

    for (int i=0;i<W*H;i++) blocked[i] = (unknown[i] || inflated_obs[i]) ? 1 : 0;

    // 레이저 값도 인플레이션 주기
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_laser_update_).count();
    if (!laser_blocked_.empty() && dt < laser_block_ttl_) {
      GridPose robot_g = worldToGrid(robot_.x, robot_.y);
      int keep = 2;

      for (int y=0; y<H; ++y) {
        for (int x=0; x<W; ++x) {
          int id = IDX(x,y,W);
          if (!laser_blocked_[id]) continue;

          int dx = x - robot_g.x;
          int dy = y - robot_g.y;
          if (std::abs(dx) <= keep && std::abs(dy) <= keep) continue;

          blocked[id] = 1;
        }
      }
    }

        // --- 로봇 주변 keep-open: 시작점이 inflated/laser에 먹혀도 탈출하게 ---
    GridPose rg = worldToGrid(robot_.x, robot_.y);

    // keep-open 반경(셀). 2~4셀 정도부터
    int keep = 3;

    for (int dy=-keep; dy<=keep; ++dy) {
    for (int dx=-keep; dx<=keep; ++dx) {
        int nx = rg.x + dx;
        int ny = rg.y + dy;
        if (!inBounds(nx, ny)) continue;

        // 단, 여기서 unknown은 열지 않는 걸 추천 (맵 밖/미지로 탈출 방지)
        int v = map_.data[IDX(nx, ny, W)];
        if (v == UNKNOWN) continue;

        blocked[IDX(nx, ny, W)] = 0;
    }
    }

    return blocked;
  }

  // 장애물 마스크 -> 프론티어 셀이 장애물에 너무 가까운지 검사할 때 사용
  std::vector<uint8_t> buildObstacleInflatedMask() const {
    int W = (int)map_.info.width;
    int H = (int)map_.info.height;

    std::vector<uint8_t> obs(W*H, 0);
    for (int y=0;y<H;y++){
      for (int x=0;x<W;x++){
        int v = map_.data[IDX(x,y,W)];
        if (v != UNKNOWN && v >= obstacle_threshold_) obs[IDX(x,y,W)] = 1;
      }
    }

    // rad = 4
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
    return inflated;
  }

  // 장애물 마스크지만 인플레이션 없음!
  std::vector<uint8_t> buildObstacleRawMask() const {
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

  // 프론티어가 장애물에 너무 가까운지 검사하기 -> 프론티어 배열에서 삭제할 때 사용
  bool isFrontierTooCloseToObstacle(const GridPose& f,
                                    const std::vector<uint8_t>& obsMaskInflated,
                                    int radius_cells) const {
    int W = (int)map_.info.width;
    for (int dy=-radius_cells; dy<=radius_cells; ++dy) {
      for (int dx=-radius_cells; dx<=radius_cells; ++dx) {
        int nx = f.x + dx;
        int ny = f.y + dy;
        if (!inBounds(nx, ny)) continue;
        if (obsMaskInflated[IDX(nx, ny, W)]) return true;
      }
    }
    return false;
  }

  // 시작점 기반 4방향 (상하좌우) 검사하여 블럭되지 않은 셀만 방문 가능 표시 (1로)
  std::vector<uint8_t> buildReachableMaskFromStart(const GridPose& start,
                                                   const std::vector<uint8_t>& blockedMask) const {
    int W = (int)map_.info.width;
    int H = (int)map_.info.height;

    std::vector<uint8_t> reachable(W*H, 0);
    if (!inBounds(start.x, start.y)) return reachable;
    if (blockedMask[IDX(start.x, start.y, W)]) return reachable;

    std::queue<GridPose> q;
    q.push(start);
    reachable[IDX(start.x, start.y, W)] = 1;

    while(!q.empty()){
      GridPose c = q.front(); q.pop();
      for(int k=0;k<4;k++){
        int nx = c.x + dx4[k];
        int ny = c.y + dy4[k];
        if (!inBounds(nx, ny)) continue;
        int id = IDX(nx, ny, W);
        if (reachable[id]) continue;
        if (blockedMask[id]) continue;
        reachable[id] = 1;
        q.push({nx, ny});
      }
    }
    return reachable;
  }

  // reachable이 1인 칸에 있는 프론티어만 남기기 위함!
  void filterFrontiersByReachable(std::vector<GridPose>& frontiers,
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

  
  // 경로랑 프론티어셀 시각화
  void publishPathMarker(const std::vector<GridPose>& path) {
    if (!path_marker_pub_ || path.empty() || !has_map_) return;

    visualization_msgs::msg::Marker m;
    m.header.stamp = this->now();
    m.header.frame_id = map_frame_;
    m.ns = "planned_path";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.03;

    m.color.r = 0.0f;
    m.color.g = 1.0f;
    m.color.b = 0.0f;
    m.color.a = 1.0f;

    m.points.reserve(path.size());
    for (const auto& gp : path) {
      auto [wx, wy] = gridToWorld(gp.x, gp.y);
      geometry_msgs::msg::Point pt;
      pt.x = wx;
      pt.y = wy;
      pt.z = 0.05;
      m.points.push_back(pt);
    }
    path_marker_pub_->publish(m);
  }

  void publishFrontierMarkers(const std::vector<GridPose>& frontiers) {
    if (!has_map_ || !frontier_marker_pub_) return;

    visualization_msgs::msg::Marker m;
    m.header.stamp = this->now();
    m.header.frame_id = map_frame_;
    m.ns = "frontiers";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.action = visualization_msgs::msg::Marker::ADD;

    m.scale.x = 0.05;
    m.scale.y = 0.05;

    m.color.r = 0.0f;
    m.color.g = 0.1f;
    m.color.b = 0.5f;
    m.color.a = 1.0f;

    m.points.reserve(frontiers.size());
    for (const auto& f : frontiers) {
      auto [wx, wy] = gridToWorld(f.x, f.y);
      geometry_msgs::msg::Point p;
      p.x = wx;
      p.y = wy;
      p.z = 0.05;
      m.points.push_back(p);
    }
    frontier_marker_pub_->publish(m);
  }

  void publishInflationMaskMarker(const std::vector<uint8_t>& inflated_mask,
                                  const GridPose& center_g)
  {
    if (!has_map_ || !infl_marker_pub_ || inflated_mask.empty()) return;

    const int W = (int)map_.info.width;
    const int H = (int)map_.info.height;
    const double res = map_.info.resolution;

    int r_cells = (int)std::ceil(infl_viz_radius_m_ / res);
    int x0 = std::max(0, center_g.x - r_cells);
    int x1 = std::min(W - 1, center_g.x + r_cells);
    int y0 = std::max(0, center_g.y - r_cells);
    int y1 = std::min(H - 1, center_g.y + r_cells);

    visualization_msgs::msg::Marker m;
    m.header.stamp = this->now();
    m.header.frame_id = map_frame_;
    m.ns = "inflation_mask";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::CUBE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;

    m.scale.x = res;
    m.scale.y = res;
    m.scale.z = 0.02;

    m.color.r = 0.5f;
    m.color.g = 0.0f;
    m.color.b = 0.5f;
    m.color.a = 0.8f;

    m.points.reserve((x1-x0+1)*(y1-y0+1) / 4);

    for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) {
        if (!inflated_mask[IDX(x,y,W)]) continue;

        auto [wx, wy] = gridToWorld(x, y);
        geometry_msgs::msg::Point p;
        p.x = wx;
        p.y = wy;
        p.z = 0.01;
        m.points.push_back(p);
      }
    }

    infl_marker_pub_->publish(m);
  }

  
  std::vector<GridPose> astar(const GridPose &start, const GridPose &goal,
                             const std::vector<uint8_t> &obsMaskInflated) const {
    int W = (int)map_.info.width;
    int H = (int)map_.info.height;

    auto inside = [&](int x,int y){ return (0<=x && x<W && 0<=y && y<H); };
    if (!inside(start.x,start.y) || !inside(goal.x,goal.y)) return {};
    if (obsMaskInflated[IDX(start.x,start.y,W)] || obsMaskInflated[IDX(goal.x,goal.y,W)]) return {};

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
        if (obsMaskInflated[nid]) continue;

        bool diagonal = (dx8[k] != 0 && dy8[k] != 0);
        int step_cost = diagonal ? 14 : 10;

    
        if (diagonal) {
          int n1 = IDX(cur.x + dx8[k], cur.y, W);
          int n2 = IDX(cur.x, cur.y + dy8[k], W);
          if (obsMaskInflated[n1] || obsMaskInflated[n2]) continue;
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

  // 경로 검사하기 (대신 마지막 20% 정도만...)
  // obsraw를 쓰는 이유는 미지, 장애물 인플레이션, 레이저 블럭까지 합쳐지면 길이 막히면서
  // 경로를 생성해도 가지 못할 수 있기 때문에 장애물만 따로
  bool pathTailTooCloseToRawObstacle(const std::vector<GridPose>& p,
                                    const std::vector<uint8_t>& obsMaskRaw) const {
    if (p.empty()) return true;
    int W = (int)map_.info.width;

    int clearance_cells = (int)std::ceil(path_clearance_m_ / map_.info.resolution);

    // 마지막 20%만 검사(최소 6개)
    int tail = std::max(6, (int)(p.size() * 0.2));
    int start_i = std::max(0, (int)p.size() - tail);

    for (int i=start_i; i<(int)p.size(); ++i) {
      const auto& pt = p[i];
      for (int dy=-clearance_cells; dy<=clearance_cells; ++dy) {
        for (int dx=-clearance_cells; dx<=clearance_cells; ++dx) {
          int nx = pt.x + dx;
          int ny = pt.y + dy;
          if (!inBounds(nx,ny)) continue;
          if (obsMaskRaw[IDX(nx,ny,W)]) return true;
        }
      }
    }
    return false;
  }

  bool pickNearestFrontier(const GridPose &robot_g,
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
      // 프론티어 자체 안전거리
      if (obsMaskInflated[IDX(f.x,f.y,W)]) continue;

      // 목표가 blocked면 스킵(unknown/인플레/레이저)
      if (blockedMask[IDX(f.x,f.y,W)]) continue;

      auto p = astar(robot_g, f, obsMaskInflated);
      if (p.empty()) continue;

      if (pathTailTooCloseToRawObstacle(p, obsMaskRaw)) continue;

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

  // 디비 스캔
  // 1. 거리 계산 함수
  double distMeters(const GridPose& a, const GridPose& b) const {
    double dx = (a.x - b.x) * map_.info.resolution;
    double dy = (a.y - b.y) * map_.info.resolution;
    return std::hypot(dx, dy);
  }

  // 반경 이웃 찾기
  std::vector<int> regionQuery(const std::vector<GridPose>& pts, int idx, double eps_m) const {
    std::vector<int> neighbors;
    neighbors.reserve(64);
    for (int j = 0; j < (int)pts.size(); ++j) {
      if (j == idx) continue;
      if (distMeters(pts[idx], pts[j]) <= eps_m) neighbors.push_back(j);
    }
    return neighbors;
  }

  // 클러스터 라벨링
  // -2 : unvisited, -1 : noise 0~ : cluster id
  std::vector<int> dbscanCluster(const std::vector<GridPose>& pts,
                                double eps_m,
                                int min_pts) const
  {
    const int N = (int)pts.size();
    std::vector<int> labels(N, -2);

    int cluster_id = 0;

    for (int i = 0; i < N; ++i) {
      if (labels[i] != -2) continue; // visited

      auto neighbors = regionQuery(pts, i, eps_m);
      if ((int)neighbors.size() + 1 < min_pts) { // +1 includes itself
        labels[i] = -1; // noise
        continue;
      }

      // 새 클러스터 시작
      labels[i] = cluster_id;

      std::queue<int> q;
      for (int nb : neighbors) q.push(nb);

      while (!q.empty()) {
        int p = q.front(); q.pop();

        if (labels[p] == -1) {
          // noise였는데 클러스터에 흡수
          labels[p] = cluster_id;
        }
        if (labels[p] != -2) {
          // 이미 다른 클러스터로 방문됐거나 지금 클러스터에 속함
          continue;
        }

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

  // 센트로이드 셀을 그대로 쓰면 그 점이 막혀있을 가능성 있음 
  // 따라서 센트로이드 셀과 가장 가까운 프론티어 셀을 목표 점으로 설정
  std::vector<GridPose> computeClusterRepresentatives(const std::vector<GridPose>& pts,
                                                      const std::vector<int>& labels) const
  {
    // cluster_id -> indices
    int max_id = -1;
    for (int l : labels) if (l > max_id) max_id = l;
    if (max_id < 0) return {};

    std::vector<std::vector<int>> clusters(max_id + 1);
    for (int i = 0; i < (int)pts.size(); ++i) {
      if (labels[i] >= 0) clusters[labels[i]].push_back(i);
    }

    std::vector<GridPose> reps;
    reps.reserve(clusters.size());

    for (const auto& idxs : clusters) {
      if (idxs.empty()) continue;

      // 1) 센트로이드(평균)
      double sx = 0.0, sy = 0.0;
      for (int id : idxs) { sx += pts[id].x; sy += pts[id].y; }
      double cx = sx / idxs.size();
      double cy = sy / idxs.size();

      // 2) 센트로이드에 가장 가까운 실제 프론티어 점을 대표로
      int best = idxs[0];
      double best_d2 = 1e18;
      for (int id : idxs) {
        double dx = pts[id].x - cx;
        double dy = pts[id].y - cy;
        double d2 = dx*dx + dy*dy;
        if (d2 < best_d2) {
          best_d2 = d2;
          best = id;
        }
      }

      reps.push_back(pts[best]);
    }

    return reps;
  }

  static double normAngle(double a) {
    while (a > M_PI) a -= 2.0*M_PI;
    while (a < -M_PI) a += 2.0*M_PI;
    return a;
  }

  double minRange(double a_min, double a_max) {
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

  void publishStop() {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_pub_->publish(cmd);
  }

  // 장애물 인식시에 회피 함수
  void publishAvoidCmd() {
    geometry_msgs::msg::Twist cmd;

    double front = minRange(-0.3, 0.3);
    double left  = minRange(0.3, 1.0);
    double right = minRange(-1.0, -0.3);

    if (front < AVOID_DIST) {
      cmd.linear.x = -0.05;
      cmd.angular.z = (left > right) ? 0.6 : -0.6;
    } else {
      cmd.linear.x = 0.15;
      cmd.angular.z = 0.0;
    }

    cmd_pub_->publish(cmd);
  }

  // wp_idx를 계속 0으로 초기화하였더니 로봇이 가만히 있는 경우?가 많이 발생함
  // 따라서 초기화대신 근처 인덱스값을 찾아서 그 값으로 대체하여 로봇을 움직이게 하기위함
  int findNearestIndexOnPath(const std::vector<GridPose>& path, int start_idx, int window) {
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
      if (d2 < best_d2) {
        best_d2 = d2;
        best_i = i;
      }
    }
    return best_i;
  }

  void followPathStep() {
    geometry_msgs::msg::Twist cmd;

    if (path_.empty()) return;

    wp_idx_ = findNearestIndexOnPath(path_, wp_idx_, 25);

    int lookahead_cells = 3;
    int target_idx = std::min(wp_idx_ + lookahead_cells, (int)path_.size() - 1);

    const auto& target = path_[target_idx];
    auto [tx, ty] = gridToWorld(target.x, target.y);

    double dxw = tx - robot_.x;
    double dyw = ty - robot_.y;

    double heading_x = std::cos(robot_.yaw);
    double heading_y = std::sin(robot_.yaw);

    double dot = dxw * heading_x + dyw * heading_y;

    if (dot < 0.0) {
      double ang_err = normAngle(std::atan2(dyw, dxw) - robot_.yaw);
      cmd.linear.x  = 0.0;
      cmd.angular.z = clampd(2.0 * ang_err, -max_ang_vel_, max_ang_vel_);
      cmd_pub_->publish(cmd);
      return;
    }

    double dist = std::hypot(dxw, dyw);
    double ang_err = normAngle(std::atan2(dyw, dxw) - robot_.yaw);

    if (dist < reach_dist_) {
      wp_idx_ = target_idx + 1;
      if (wp_idx_ >= (int)path_.size()) path_.clear();
      return;
    }

    if (std::abs(ang_err) > 0.6) {
      cmd.linear.x  = 0.0;
      cmd.angular.z = clampd(2.5 * ang_err, -max_ang_vel_, max_ang_vel_);
    } else {
      cmd.linear.x  = clampd(0.6 * dist, 0.05, max_lin_vel_);
      cmd.angular.z = clampd(2.0 * ang_err, -max_ang_vel_, max_ang_vel_);
    }

    cmd_pub_->publish(cmd);
  }

  struct LocalMapStats {
  int known{0};
  int occ{0};
  int minv{101};
  int maxv{-1};
};

LocalMapStats computeLocalMapStats(const GridPose& center_g, double radius_m) const {
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

      int v = map_.data[IDX(x,y,W)];
      if (v == UNKNOWN) continue;

      s.known++;
      s.minv = std::min(s.minv, v);
      s.maxv = std::max(s.maxv, v);
      if (v >= obstacle_threshold_) s.occ++;
    }
  }
  return s;
}

void publishWarmupMotion() {
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x  = warmup_lin_vel_;   // 0이면 제자리 회전
  cmd.angular.z = warmup_yaw_rate_;
  cmd_pub_->publish(cmd);
}
  
  void onTimer() {
    if (done_ || !has_map_) return;

    if (!updateRobotPoseFromTF()) {
      publishStop();
      return;
    }

    GridPose robot_g = worldToGrid(robot_.x, robot_.y);
    if (!inBounds(robot_g.x, robot_g.y)) {
      publishStop();
      return;
    }

    // 초기 매 업데이트
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
        // 워밍업 동안은 탐색 목표/플래닝을 하지 않음
        path_.clear();
        wp_idx_ = 0;
        avoiding_ = false; 

        publishWarmupMotion();
        return;
    }

    // 준비됐거나 타임아웃이면 워밍업 종료
    warmup_done_ = true;
    publishStop();
    // 다음 tick부터 정상 플래닝 진행
    }


    // 1) 회피
    double front = minRange(-0.3, 0.3);

    if (!avoiding_ && has_scan_ && front < AVOID_DIST) {
      avoiding_ = true;
      path_.clear();
      wp_idx_ = 0;
    }

    if (avoiding_) {
      if (has_scan_ && front > AVOID_END_DIST) {
        avoiding_ = false;
        path_.clear();
        wp_idx_ = 0;
      } else {
        publishAvoidCmd();
        return;
      }
    }

    
    // 1. 장애물+unknown+레이저 마스크, 2. 장애물마스크+인플레이션, 3. 장애물마스크
    auto blockedMask = buildInflatedBlockedMask();
    auto obsMaskInflated = buildObstacleInflatedMask();
    auto obsMaskRaw      = buildObstacleRawMask();

    int W = (int)map_.info.width;
    int keep = 2;
    for (int dy=-keep; dy<=keep; ++dy) {
        for (int dx=-keep; dx<=keep; ++dx) {
        int nx = robot_g.x + dx;
        int ny = robot_g.y + dy;
        if (!inBounds(nx, ny)) continue;
        obsMaskInflated[IDX(nx, ny, W)] = 0; 
        }
    }

    for (int dy=-keep; dy<=keep; ++dy) {
        for (int dx=-keep; dx<=keep; ++dx) {
        int nx = robot_g.x + dx;
        int ny = robot_g.y + dy;
        if (!inBounds(nx, ny)) continue;
        blockedMask[IDX(nx, ny, W)] = 0; 
        }
    }

    publishInflationMaskMarker(blockedMask, robot_g);

    if (!path_.empty()) {
    // wp_idx_ 갱신(현재 로봇 위치 기준으로 path 상의 가장 가까운 인덱스)
    int new_wp = findNearestIndexOnPath(path_, wp_idx_, 25);

    // 1) "경로가 최신 blockedMask 기준으로 막혔는지" 빠른 체크
    //    (wp 주변 ~ 앞쪽 몇 칸만 검사)
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
        if (!inBounds(p.x, p.y)) return true;
        if (mask[IDX(p.x, p.y, W)]) return true;
      }
      return false;
    };

    if (isPathInvalidNow(path_, new_wp, blockedMask)) {
      RCLCPP_WARN(get_logger(), "path became invalid -> clear & replan");
      path_.clear();
      wp_idx_ = 0;
      publishStop();
      return; // 다음 tick에서 새 plan
    }

    // 2) 빙글빙글하거나 정체되면 replan
    static rclcpp::Time last_progress_time = this->now();
    static double last_x = robot_.x, last_y = robot_.y;
    static int last_wp = 0;

  
    auto resetStuckState = [&](){
      last_progress_time = this->now();
      last_x = robot_.x;
      last_y = robot_.y;
      last_wp = 0;
    };

    double moved = std::hypot(robot_.x - last_x, robot_.y - last_y);
    bool progressed = (new_wp > last_wp + 1) || (moved > 0.05); // 5cm 또는 wp 증가

    if (progressed) {
      last_progress_time = this->now();
      last_x = robot_.x; last_y = robot_.y;
      last_wp = new_wp;
    } else {
      double dt = (this->now() - last_progress_time).seconds();
      if (dt > 2.0) { // 2초 이상 진전 없으면 재플랜
        RCLCPP_WARN(get_logger(), "stuck %.2fs -> clear & replan", dt);
        path_.clear();
        wp_idx_ = 0;
        resetStuckState();
        publishStop();
        return; // 다음 tick에서 새 plan
      }
    }

    // 여기까지 통과했으면 정상적으로 추종
    wp_idx_ = new_wp;
    followPathStep();
    return;
  }

  
// 2) 프론티어 감지/선정 

auto frontiers = detectFrontiers(robot_g);

// frontiers가 0이면 "종료"하지 말고 잠깐 대기 (맵 업데이트 타이밍문제일수도 있기때문에)
if (frontiers.empty()) {
  publishStop();
  return;
}

// (A) 장애물 근접 프론티어 제거
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

auto obsReach = obsMaskInflated;
  {
    int W = (int)map_.info.width;
    int keep = 2;
    for (int dy=-keep; dy<=keep; ++dy) {
        for (int dx=-keep; dx<=keep; ++dx) {
        int nx = robot_g.x + dx;
        int ny = robot_g.y + dy;
        if (!inBounds(nx, ny)) continue;
        obsReach[IDX(nx, ny, W)] = 0; 
        }
    }
  }

// reachable frontier 계산 (obstacle inflation mask이용!!)
auto reachable = buildReachableMaskFromStart(robot_g, obsReach);
filterFrontiersByReachable(frontiers, reachable);


if (frontiers.empty()) {
  publishStop();
  publishFrontierMarkers({});
  RCLCPP_WARN(get_logger(), "no reachable frontiers left (after reachable filter)");
  return;
}

// reachable 된 프론티어만 가지고 DBSCAN 대표점(reps) 생성
std::vector<GridPose> reps;

if (use_dbscan_) {
  // 1) DBSCAN 라벨링
  auto labels = dbscanCluster(frontiers, dbscan_eps_m_, dbscan_min_pts_);

  // 2) 라벨에서 대표점(센트로이드에 가장 가까운 실제 frontier 점) 계산
  reps = computeClusterRepresentatives(frontiers, labels);

  RCLCPP_WARN(get_logger(), "DBSCAN reps=%zu (from %zu)", reps.size(), frontiers.size());
} else {
  reps = frontiers;
}

// DBSCAN 실패/너무 빡세서 reps가 0이면 fallback (raw reachable frontiers 사용)
if (reps.empty()) {
  RCLCPP_WARN(get_logger(), "DBSCAN reps=0 -> fallback to raw reachable frontiers (%zu)", frontiers.size());
  reps = frontiers;
}


// 시각화
publishFrontierMarkers(reps);

// reps 중에서 가장 좋은 goal 뽑아서 A*
GridPose goal;
std::vector<GridPose> new_path;

bool planned = pickNearestFrontier(robot_g, reps,
                                   blockedMask, obsMaskInflated, obsMaskRaw,
                                   goal, new_path);

if (!planned) {
  publishStop();
  RCLCPP_WARN(get_logger(), "no plan");
  return;
}

path_ = std::move(new_path);
wp_idx_ = 0;
publishPathMarker(path_);
followPathStep();

RCLCPP_WARN(get_logger(), "plan: (%d, %d) -> (%d, %d)", robot_g.x, robot_g.y, goal.x, goal.y);

}
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FrontierExplorerAStar>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}