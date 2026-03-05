#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>

#include <cmath>
#include <vector>
#include <queue>
#include <algorithm>
#include <limits>
#include <string>
#include <unordered_map>

#include <std_msgs/msg/int64.hpp>
#include <unordered_set>
#include <random>

// =================================== 변경 내용 관리 ===================================
// 2/5)
// reach min dist 값 조정
// follow path step target idx 값 wp idx + 1 로 변경
// pick best frontier 디버그 로그 찍기 (astar 경로 문제 같음)
// ========================================= ========================================= //
// 2/6)
// pick best frontier 내부에 최소 거리 판정 추가해서 자기 발밑 프론티어는 선택 불가하도록 수정
// visited_topic, goal reservation out topic 절대 -> 상대 토픽으로 수정
// follow path step target idx 값 다시 wp_idx + 3 으로 변경 (로봇 안 움직임 문제 해결)
// 웜엄은 제대로 되고 있음
// ========================================= ========================================= //
// 2/9)
// follow path step에서 min dist 도달 시 path clear 후 publish stop 추가
// detect frontier 에서 자기 발 밑이나 근처 프론티어 점들은 detect 하지 않도록 excl 변수 추가
// 로그로 clearance filter에서 frontier가 다 걸러짐을 확인 -> 로봇 멈춤
// 따라서 frontier_clearance 값 낮추고 infl_radius 값도 낮춰서 확인
// 또한, isfrontiertooclosetoobs에서 사각형 검사가 아닌 원형 검사로 변경
// 또한, isfrontiertooclosaetoobs에서 검사를 obsInfl대신 obsRaw로 변경 (inflation까지 반영하니까 너무 보수적이게 됨)
// astarmask = obsinfl 로 해서 applykeepopen 적용하기 (reachable에만 적용하지 않기)
// robot path 검사 시에 path 가 자꾸 걸림 -> path clearance 값 줄이기
// ========================================= ========================================= //
// 2/10)
// visited/reserved penalty가 제대로 작동하는지 확인하기 위해 디버그용 로그 추가
// applykeepopen 중복 문제 제거 및 blockedmask에 다시 unknown 추가해서 goal이 unknown이 되는 것 방지
// astar step cost에서 unknown 비용 줄여서 unknown 쪽에 경로 생성이 되지 않는 문제를 해결해보려고함
// 각각의 맵에서 탐사하는 것을 해결하고자 map topic을 /merge_map으로 변경
// ========================================= ========================================= //
// 2/11)
// pathtail 검사 생략, 이미 astar에서 inflation 반영된 마스크로 검사하고 있기 때문에 pathtail filter 해버리면
// path 끝 쪽에서 탈락해서 astar로는 갈 수 있지만, path 끝 쪽은 안 된다고 판단하는 경우가 생길 수 있음
// ontimer에서 frontier clearance를 이미 거친 점을 대상으로 reps 계산을 하기 때문에 
// pickbestfrontier에서 다시 clearance 검사할 필요 없음, 따라서 생략함
// buildreachablemaskfromstart 검사를 astar와 동일하게 8방향이웃 검사로 확장함
// 2/26)
// 내 visited/reserved pt면 구독 금지하도록 onvisited, onreserved에 src topic 추가
// rp가 높으면 ig 최대한 낮추기 -> 적용이 잘 안되는 것 같음
// cluster 마다 id 부여해서 로봇끼리 같은 id를 가진 클러스터는 가지 않도록 추가해보았음
// 위의 부분도 잘 안되는 것 같아서... 그리고 애초에 다른 로봇이 goal로 도착해서 넓혀놨는데
// 그 쪽에 도착해야 다시 replanning을 하기 때문에 이를 고치고자
// info gain 쪽 수정해서 info gain 늘어나면 바로 replanning 하도록 수정
// 3/4)
// 너무 작은 클러스터때문에 로봇 움직임 불필요함 -> dbscan min pts와 eps m 늘림
// 클러스터 및 대표점 시각화
// visited 효과가 딱히 없어서 penalty 항목에서 제거
// 클러스터 id로는 구분하기 어려워서 그냥 삭제함 


static constexpr int UNKNOWN = -1;

struct GridPose { int x{0}, y{0}; };
struct FrontierRep {
  GridPose g;      // representative cell
  int64_t key;     // cluster signature key
};
struct WorldPose { double x{0}, y{0}, yaw{0}; };

static inline int IDX(int x, int y, int w) { return y * w + x; }
static inline double clampd(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

static const int dx8[8] = { 1, 1, 0,-1,-1,-1, 0, 1};
static const int dy8[8] = { 0, 1, 1, 1, 0,-1,-1,-1};
static const int dx4[4] = { 1,-1, 0, 0};
static const int dy4[4] = { 0, 0, 1,-1};

class FrontierExplorerMulti : public rclcpp::Node {
public:
  FrontierExplorerMulti() : Node("frontier_explorer_multi")
  {
    // ====== params ======

    // 토픽, 로봇, 프레임의 네임스페이스 지정
    robot_id_ = this->declare_parameter<std::string>("robot_id", "robot1");
    map_topic_ = this->declare_parameter<std::string>("map_topic", "map");
    cmd_topic_ = this->declare_parameter<std::string>("cmd_topic", "cmd_vel");
    scan_topic_ = this->declare_parameter<std::string>("scan_topic", "scan");

    map_frame_  = this->declare_parameter<std::string>("map_frame", "map");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_footprint");
    tf_timeout_s_ = this->declare_parameter<double>("tf_timeout_s", 0.10);

    // 장애물, free의 임계점
    obstacle_threshold_ = this->declare_parameter<int>("obstacle_threshold", 60);
    free_threshold_     = this->declare_parameter<int>("free_threshold", 50);

    // inflation 반경 및 로봇 주변 반경 frontier 추출
    inflation_radius_m_       = this->declare_parameter<double>("inflation_radius_m", 0.20);
    frontier_search_radius_m_ = this->declare_parameter<double>("frontier_search_radius_m", 8.0);

    // control value
    max_lin_vel_ = this->declare_parameter<double>("max_lin_vel", 0.18);
    max_ang_vel_ = this->declare_parameter<double>("max_ang_vel", 0.60);
    reach_dist_  = this->declare_parameter<double>("reach_dist_m", 0.25);

    // 장애물 회피 시
    avoid_enter_dist_ = this->declare_parameter<double>("avoid_enter_dist", 0.30);
    avoid_exit_dist_  = this->declare_parameter<double>("avoid_exit_dist", 0.45);

    // frontier safety
    frontier_clearance_m_ = this->declare_parameter<double>("frontier_clearance_m", 0.15);
    path_clearance_m_     = this->declare_parameter<double>("path_clearance_m", 0.08);

    // 로봇 발 밑 주변 cell 열어주기
    keep_open_cells_ = this->declare_parameter<int>("keep_open_cells", 2);

    // DBSCAN
    use_dbscan_     = this->declare_parameter<bool>("use_dbscan", true);
    dbscan_eps_m_   = this->declare_parameter<double>("dbscan_eps_m", 0.40);
    dbscan_min_pts_ = this->declare_parameter<int>("dbscan_min_pts", 10);
 
    // Laser mask
    laser_block_ttl_ = this->declare_parameter<double>("laser_block_ttl", 1.0);
    laser_inflation_radius_m_ = this->declare_parameter<double>("laser_inflation_radius_m", 0.12);

    // Warmup
    warmup_enable_ = this->declare_parameter<bool>("warmup_enable", true);
    warmup_min_known_cells_ = this->declare_parameter<int>("warmup_min_known_cells", 12000);
    warmup_min_occ_cells_   = this->declare_parameter<int>("warmup_min_occ_cells", 50);
    warmup_radius_m_        = this->declare_parameter<double>("warmup_radius_m", 2.0);
    warmup_yaw_rate_        = this->declare_parameter<double>("warmup_yaw_rate", 0.6);
    warmup_lin_vel_         = this->declare_parameter<double>("warmup_lin_vel", 0.0);
    warmup_max_time_s_      = this->declare_parameter<double>("warmup_max_time_s", 6.0);
    warmup_state_start_     = this->now();

    // Stuck 됐을 때
    stuck_timeout_s_ = this->declare_parameter<double>("stuck_timeout_s", 5.0);
    stuck_min_move_m_ = this->declare_parameter<double>("stuck_min_move_m", 0.02);

    // Debug viz on/off (마커 퍼블리시는 유지)
    enable_viz_ = this->declare_parameter<bool>("enable_viz", true);

    // ====== Multi-robot utility params ======
    visit_radius_m_ = this->declare_parameter<double>("visit_radius_m", 0.60);
    utility_radius_m_ = this->declare_parameter<double>("utility_radius_m", 1.00);
    info_gain_radius_m_ = this->declare_parameter<double>("info_gain_radius_m", 1.50);

    alpha_ = this->declare_parameter<double>("alpha_info_gain", 2.0);
    beta_  = this->declare_parameter<double>("beta_path_len", 1.0);
    gamma_ = this->declare_parameter<double>("gamma_visited", 5.0);
    delta_ = this->declare_parameter<double>("delta_reserve", 15.0);

    reserve_exclusion_radius_m_ = this->declare_parameter<double>("reserve_exclusion_radius_m", 2.0);
    reserve_ttl_s_ = this->declare_parameter<double>("reserve_ttl_s", 6.0);

    // visited_topic_ = this->declare_parameter<std::string>("visited_topic", "/global_visited");
    // reserve_topic_ = this->declare_parameter<std::string>("reserve_topic", "/goal_reservation");

    // marker topics
    path_marker_topic_     = this->declare_parameter<std::string>("path_marker_topic", "path_marker");
    frontier_marker_topic_ = this->declare_parameter<std::string>("frontier_marker_topic", "frontier_markers");
    infl_marker_topic_     = this->declare_parameter<std::string>("infl_marker_topic", "inflation_marker");
    cluster_marker_topic_  = this->declare_parameter<std::string>("cluster_marker_topic", "cluster_marker");

    // ====== TF ======
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ====== SUB/PUB ======
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, map_qos, std::bind(&FrontierExplorerMulti::onMap, this, std::placeholders::_1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, 10, std::bind(&FrontierExplorerMulti::onScan, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, 10);

    path_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(path_marker_topic_, 10);
    frontier_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(frontier_marker_topic_, 10);
    infl_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(infl_marker_topic_, 10);
    cluster_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(cluster_marker_topic_, 10);

    global_frame_ = this->declare_parameter<std::string>("global_frame", "tb3_0/map");

    // visited point config
    // visited_out_topic_ = this->declare_parameter<std::string>("visited_out_topic", "visited_point");
    // visited_in_topics_ = this->declare_parameter<std::vector<std::string>>(
    //   "visited_in_topics",
    //   std::vector<std::string>{"/tb3_0/visited_point", "/tb3_1/visited_point"}
    // );

    // visited_ttl_s_ = this->declare_parameter<double>("visited_ttl_s", 60.0);
    // visited_penalty_radius_m_ = this->declare_parameter<double>("visited_penalty_radius_m", 1.0);
    // visited_penalty_target_count_ = this->declare_parameter<int>("visited_penalty_target_count", 30);
    // visited_pub_period_s_ = this->declare_parameter<double>("visited_pub_period_s", 0.3);

    // reserve point config
    reserve_out_topic_ = this->declare_parameter<std::string>("reserve_out_topic", "/global_goal_reservation");
    // reserve_in_topics_ = this->declare_parameter<std::vector<std::string>>(
    //   "reserve_in_topics",
    //   std::vector<std::string>{"/tb3_0/goal_reservation", "/tb3_1/goal_reservation"}
    // );

    // visited point pub/sub
      // visited_point_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(visited_out_topic_, 10);
      // for (const auto& t : visited_in_topics_) {
      //   visited_point_subs_.push_back(
      //     this->create_subscription<geometry_msgs::msg::PoseStamped>(
      //       t, 10,
      //       [this, t](const geometry_msgs::msg::PoseStamped::SharedPtr msg){
      //         this->onVisitedPoint(*msg, t);
      //       }
      //     )
      //   );
      // }

      // reserve pub/sub (topic 분리 버전)
      reserve_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(reserve_out_topic_, 10);
      reserve_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          reserve_out_topic_, 10,
          [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
              // 토픽 이름 대신 msg->header.frame_id를 써서 누가 보냈는지 구분할 거예요.
              this->onReservePoint(*msg, msg->header.frame_id); 
          }
      );

    // // cluster reservation config (NEW)
    // cluster_reserve_out_topic_ = this->declare_parameter<std::string>("cluster_reserve_out_topic", "cluster_reservation");
    // cluster_reserve_in_topics_ = this->declare_parameter<std::vector<std::string>>(
    //   "cluster_reserve_in_topics",
    //   std::vector<std::string>{"/tb3_0/cluster_reservation", "/tb3_1/cluster_reservation"}
    // );
    // cluster_reserve_ttl_s_ = this->declare_parameter<double>("cluster_reserve_ttl_s", 6.0);

    // // cluster reservation pub/sub (NEW)
    // cluster_reserve_pub_ = this->create_publisher<std_msgs::msg::Int64>(cluster_reserve_out_topic_, 10);
    // for (const auto& t : cluster_reserve_in_topics_) {
    //   cluster_reserve_subs_.push_back(
    //     this->create_subscription<std_msgs::msg::Int64>(
    //       t, 10,
    //       [this, t](const std_msgs::msg::Int64::SharedPtr msg){
    //         this->onReserveClusterKey(*msg, t);
    //       }
    //     )
    //   );
    // }

    cand_text_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("cand_text_markers", 10);

    // 2/26 info gain 추가 부분
    replan_check_period_s_ = this->declare_parameter<double>("replan_check_period_s", 0.5);
    min_commit_time_s_     = this->declare_parameter<double>("min_commit_time_s", 2.0);
    ig_drop_thresh_        = this->declare_parameter<double>("ig_drop_thresh", 0.10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
      std::bind(&FrontierExplorerMulti::onTimer, this));

    RCLCPP_WARN(this->get_logger(),
      "[%s] Started ver.1 map=%s scan=%s cmd=%s base=%s (map_frame=%s)",
      robot_id_.c_str(), map_topic_.c_str(), scan_topic_.c_str(), cmd_topic_.c_str(),
      base_frame_.c_str(), map_frame_.c_str());
  }

private:
  // ---------- ROS interfaces ----------
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr frontier_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr infl_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_marker_pub_;

  // rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr visited_pub_;
  // rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr visited_sub_;
  // rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr reserve_pub_;
  // rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr reserve_sub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ---------- Params / topics ----------
  std::string robot_id_;
  std::string map_topic_, cmd_topic_, scan_topic_;
  std::string map_frame_, base_frame_;
  double tf_timeout_s_{0.1};

  std::string path_marker_topic_, frontier_marker_topic_, infl_marker_topic_, cluster_marker_topic_;
  bool enable_viz_{true};

  int obstacle_threshold_{60};
  int free_threshold_{50};
  double inflation_radius_m_{0.20};
  double frontier_search_radius_m_{8.0};

  double max_lin_vel_{0.18};
  double max_ang_vel_{0.60};
  double reach_dist_{0.25};

  // avoid (간단화)
  bool avoiding_{false};
  double avoid_enter_dist_{0.30};
  double avoid_exit_dist_{0.45};

  // frontier safety
  double frontier_clearance_m_{0.15};
  double path_clearance_m_{0.16};

  // keep-open
  int keep_open_cells_{2};

  // DBSCAN
  double dbscan_eps_m_{0.40};
  int dbscan_min_pts_{10};
  bool use_dbscan_{true};

  // Laser mask
  sensor_msgs::msg::LaserScan last_scan_;
  bool has_scan_{false};
  std::vector<uint8_t> laser_blocked_;
  std::chrono::steady_clock::time_point last_laser_update_{};
  double laser_block_ttl_{1.0};
  double laser_inflation_radius_m_{0.12};

  // Warmup
  bool warmup_enable_{true};
  int warmup_min_known_cells_{12000};
  int warmup_min_occ_cells_{50};
  double warmup_radius_m_{2.0};
  double warmup_yaw_rate_{0.6};
  double warmup_lin_vel_{0.0};
  double warmup_max_time_s_{6.0};
  rclcpp::Time warmup_state_start_;
  bool warmup_done_{false};

  // Stuck
  double stuck_timeout_s_{5.0};
  double stuck_min_move_m_{0.02};

  // ---- Stuck tracking (member) ----
  rclcpp::Time last_progress_time_{0, 0, RCL_ROS_TIME};
  double last_progress_x_{0.0}, last_progress_y_{0.0};
  bool progress_inited_{false};


  // Map / pose / path
  nav_msgs::msg::OccupancyGrid map_;
  bool has_map_{false};

  WorldPose robot_{};
  bool has_pose_{false};

  std::vector<GridPose> path_;
  int wp_idx_{0};

  double visit_radius_m_{0.6};
  double utility_radius_m_{1.0};
  double info_gain_radius_m_{1.5};

  double alpha_{2.0}, beta_{1.0}, gamma_{5.0}, delta_{15.0};

  double reserve_exclusion_radius_m_{1.2};
  double reserve_ttl_s_{6.0};

  // ---- Global-frame coordination (point-based visited/reserve) ----
  std::string global_frame_{"world"};

  // visited point topics
  // std::string visited_out_topic_;
  // std::vector<std::string> visited_in_topics_;
  // rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr visited_point_pub_;
  // std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> visited_point_subs_;

  // reserve point topics
  std::string reserve_out_topic_;
  std::vector<std::string> reserve_in_topics_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr reserve_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr reserve_sub_;

  // cluster reservation topics (NEW)
  // std::string cluster_reserve_out_topic_;
  // std::vector<std::string> cluster_reserve_in_topics_;
  // rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr cluster_reserve_pub_;
  // std::vector<rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr> cluster_reserve_subs_;
  // double cluster_reserve_ttl_s_{6.0}; // NEW (파라미터로 받을 예정)

  // // reserved cluster key store (NEW)
  // std::unordered_map<int64_t, rclcpp::Time> reserved_cluster_keys_;
  //   struct TimedPoint {
  //     double x{0}, y{0};
  //     rclcpp::Time stamp;
  //     std::string src; // topic name
  //   };

  // std::vector<TimedPoint> visited_points_;
  // double visited_ttl_s_{60.0};
  // double visited_penalty_radius_m_{1.0};
  // int visited_penalty_target_count_{30};
  // double visited_pub_period_s_{0.3};
  // rclcpp::Time last_visited_pub_time_{0,0,RCL_ROS_TIME};


  // utility 디버그용
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cand_text_pub_;

  struct ReservedGoal {
    std::string src; //topic name
    double x{0}, y{0};
    rclcpp::Time stamp;
  };
  std::unordered_map<std::string, ReservedGoal> reservations_;

  struct CandDbg {
    GridPose g;
    double score{0.0};
    double ig{0.0};
    //double vp{0.0};
    double rp{0.0};
    double path_len{0.0};
  };

  // ---- Replan by IG (NEW) ----
  GridPose current_goal_{0,0};
  bool has_goal_{false};

  rclcpp::Time last_replan_check_{0,0,RCL_ROS_TIME};
  double replan_check_period_s_{0.5};   // 0.3~1.0 추천

  rclcpp::Time goal_commit_start_{0,0,RCL_ROS_TIME};
  double min_commit_time_s_{2.0};       // 너무 자주 바뀌는거 방지

  double ig_drop_thresh_{0.10};         // 0.08~0.15 정도에서 튜닝

  // ---------- Helpers ----------
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

  // traversable/free
  bool isTraversable(int x, int y) const {
    if (!inBounds(x,y)) return false;
    int v = map_.data[IDX(x, y, (int)map_.info.width)];
    if (v == UNKNOWN) return false;
    return (v >= 0 && v <= free_threshold_);
  }

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

  std::vector<GridPose> detectFrontiers(const GridPose &robot_g) const {
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

  void applyKeepOpen(std::vector<uint8_t>& mask, const GridPose& robot_g) const {
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

  // Laser to blocked mask
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg){
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
  std::vector<uint8_t> buildObstacleInflatedMask() const {
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

    // 3) 레이저 블록 (TTL 내에서만, 셀별 적용)
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_laser_update_).count();

    if (!laser_blocked_.empty() && dt < laser_block_ttl_) {
      GridPose robot_g = worldToGrid(robot_.x, robot_.y);
      int keep = 2;  // start 탈출용

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

  // unknown + obs + laser = blocked + inflation
  std::vector<uint8_t> buildBlockedMask() const {
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

    // laser block (TTL 내에서만)
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
  bool isFrontierTooCloseToObstacle(const GridPose& f,
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
  std::vector<uint8_t> buildReachableMaskFromStart(const GridPose& start,
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

  // ---- A* ----
  std::vector<GridPose> astar(const GridPose &start, const GridPose &goal,
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

  // bool pathTailTooCloseToRawObstacle(const std::vector<GridPose>& p,
  //                                   const std::vector<uint8_t>& obsRaw) const {
  //   if (p.empty()) return true;
  //   int W = (int)map_.info.width;

  //   int clearance_cells = (int)std::ceil(path_clearance_m_ / map_.info.resolution);
  //   int tail = std::max(6, (int)(p.size() * 0.2));
  //   int start_i = std::max(0, (int)p.size() - tail);

  //   for (int i=start_i; i<(int)p.size(); ++i) {
  //     const auto& pt = p[i];
  //     for (int dy=-clearance_cells; dy<=clearance_cells; ++dy) {
  //       for (int dx=-clearance_cells; dx<=clearance_cells; ++dx) {
  //         int nx = pt.x + dx;
  //         int ny = pt.y + dy;
  //         if (!inBounds(nx,ny)) continue;
  //         if (obsRaw[IDX(nx,ny,W)]) return true;
  //       }
  //     }
  //   }
  //   return false;
  // }

  // ---- DBSCAN ----
  double distMeters(const GridPose& a, const GridPose& b) const {
    double dx = (a.x - b.x) * map_.info.resolution;
    double dy = (a.y - b.y) * map_.info.resolution;
    return std::hypot(dx, dy);
  }

  std::vector<int> regionQuery(const std::vector<GridPose>& pts, int idx, double eps_m) const {
    std::vector<int> neighbors;
    neighbors.reserve(64);
    for (int j = 0; j < (int)pts.size(); ++j) {
      if (j == idx) continue;
      if (distMeters(pts[idx], pts[j]) <= eps_m) neighbors.push_back(j);
    }
    return neighbors;
  }

  std::vector<int> dbscanCluster(const std::vector<GridPose>& pts, double eps_m, int min_pts) const {
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
  std::vector<GridPose> computeClusterRepresentatives(const std::vector<GridPose>& pts,
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
    // 1. 점 개수 필터링 (설정하신 dbscan_min_pts_ 활용)
    if (idxs.empty() || (int)idxs.size() < dbscan_min_pts_) continue;

    double sx = 0.0, sy = 0.0;
    for (int id : idxs) { sx += pts[id].x; sy += pts[id].y; }
    double cx = sx / idxs.size();
    double cy = sy / idxs.size();

    // 2. 물리적 반지름 계산 및 필터링
    double max_dist_sq = 0.0;
    for (int id : idxs) {
      double dx = (pts[id].x - cx) * res;
      double dy = (pts[id].y - cy) * res;
      double d2 = dx*dx + dy*dy;
      if (d2 > max_dist_sq) max_dist_sq = d2;
    }
    double radius = std::sqrt(max_dist_sq);

    // 반지름이 너무 작으면 가짜 골로 판단하고 무시 (eps_m의 절반 기준)
    if (radius < (dbscan_eps_m_ * 0.5)) continue;

    // 필터를 통과한 경우에만 최적의 점(Representative) 계산
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

  // static inline int64_t makeClusterKeyFromCellCentroid(
  //   double cx_cell, double cy_cell, double res, double quant_m)
  // {
  //   // quant_m: 0.6m 추천
  //   double qcell = std::max(1.0, quant_m / res);
  //   int64_t qx = (int64_t)std::llround(cx_cell / qcell);
  //   int64_t qy = (int64_t)std::llround(cy_cell / qcell);
  //   return (qx << 32) ^ (qy & 0xffffffff);
  // }

  // std::vector<FrontierRep> computeClusterRepresentativesWithKey(
  //   const std::vector<GridPose>& pts,
  //   const std::vector<int>& labels) const
  // {
  //   int max_id = -1;
  //   for (int l : labels) if (l > max_id) max_id = l;
  //   if (max_id < 0) return {};

  //   std::vector<std::vector<int>> clusters(max_id + 1);
  //   for (int i = 0; i < (int)pts.size(); ++i) {
  //       if (labels[i] >= 0) clusters[labels[i]].push_back(i);
  //   }

  //   std::vector<FrontierRep> reps;
  //   const double quant_m = 0.6;
  //   double res = map_.info.resolution;

  //   for (int i = 0; i < (int)clusters.size(); ++i) {
  //       const auto& idxs = clusters[i];
        
  //       // 1. 이미 설정하신 dbscan_min_pts_를 여기서 필터로 사용!
  //       if (idxs.empty() || (int)idxs.size() < dbscan_min_pts_) continue;

  //       double sx = 0.0, sy = 0.0;
  //       for (int id : idxs) { sx += pts[id].x; sy += pts[id].y; }
  //       double cx = sx / idxs.size();
  //       double cy = sy / idxs.size();

  //       // 2. 물리적 반지름 계산 (노이즈 제거 핵심)
  //       double max_dist_sq = 0.0;
  //       for (int id : idxs) {
  //           double dx = (pts[id].x - cx) * res;
  //           double dy = (pts[id].y - cy) * res;
  //           double d2 = dx*dx + dy*dy;
  //           if (d2 > max_dist_sq) max_dist_sq = d2;
  //       }
  //       double radius = std::sqrt(max_dist_sq);

  //       // [추가] 반지름이 eps_m의 절반보다 작으면 너무 파편화된 노이즈로 간주
  //       // 이 임계값(0.5)은 상황에 따라 조절 가능합니다람쥐!
  //       if (radius < (dbscan_eps_m_ * 0.5)) continue;

  //       int best = idxs[0];
  //       double best_d2 = 1e18;
  //       for (int id : idxs) {
  //           double dx = pts[id].x - cx;
  //           double dy = pts[id].y - cy;
  //           double d2 = dx*dx + dy*dy;
  //           if (d2 < best_d2) { best_d2 = d2; best = id; }
  //       }

  //       int64_t key = makeClusterKeyFromCellCentroid(cx, cy, res, quant_m);
  //       reps.push_back(FrontierRep{pts[best], key});
  //   }
  //   return reps;
  // }

  // ---- Utility ----
  double infoGainAround(const GridPose& g, int radius_cells) const {
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

  // void publishVisitedPointIfDue() {
  //   auto now = this->now();
  //   if ((now - last_visited_pub_time_).seconds() < visited_pub_period_s_) return;

  //   double xg, yg;
  //   if (!toGlobal(robot_.x, robot_.y, xg, yg)) return;

  //   geometry_msgs::msg::PoseStamped ps;
  //   ps.header.stamp = now;
  //   ps.header.frame_id = global_frame_;
  //   ps.pose.position.x = xg;
  //   ps.pose.position.y = yg;
  //   ps.pose.position.z = 0.0;
  //   ps.pose.orientation.w = 1.0;

  //   RCLCPP_INFO(this->get_logger(),
  //     "[%s][VIS_PUB] topic=%s t=%.3f g=(%.2f,%.2f)",
  //     robot_id_.c_str(),
  //     visited_point_pub_->get_topic_name(),
  //     this->now().seconds(),
  //     xg, yg
  //   );

  //   visited_point_pub_->publish(ps);
  //   last_visited_pub_time_ = now;
  // }

  // void onVisitedPoint(const geometry_msgs::msg::PoseStamped& msg, const std::string& src_topic) {
  //   // frame 통일 안 되어 있으면 버림
  //   if (msg.header.frame_id != global_frame_) return;
  //   // ✅ 내가 발행하는 visited 토픽이면 무시
  //   if (src_topic == visited_point_pub_->get_topic_name()) return;

  //   visited_points_.push_back(TimedPoint{msg.pose.position.x, msg.pose.position.y, this->now(), src_topic});

  //   RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
  //     "[%s][VIS_SUB] from=%s frame=%s p=(%.2f,%.2f) n=%zu",
  //     robot_id_.c_str(),
  //     src_topic.c_str(),
  //     msg.header.frame_id.c_str(),
  //     msg.pose.position.x, msg.pose.position.y,
  //     visited_points_.size()
  //   );

  //   // 메모리 폭주 방지(적당히)
  //   if (visited_points_.size() > 20000) {
  //     visited_points_.erase(visited_points_.begin(), visited_points_.begin() + 5000);
  //   }
  // }

  // double visitedPenaltyGlobal(double goal_x_g, double goal_y_g) {
  //   auto now = this->now();

  //   // TTL cleanup
  //   visited_points_.erase(
  //     std::remove_if(visited_points_.begin(), visited_points_.end(),
  //       [&](const TimedPoint& p){
  //         return (now - p.stamp).seconds() > visited_ttl_s_;
  //       }),
  //     visited_points_.end()
  //   );

  //   int cnt = 0;
  //   for (const auto& p : visited_points_) {
  //     double d = std::hypot(goal_x_g - p.x, goal_y_g - p.y);
  //     if (d <= visited_penalty_radius_m_) cnt++;
  //   }

  //   if (visited_penalty_target_count_ <= 0) return 0.0;
  //   return std::min(1.0, (double)cnt / (double)visited_penalty_target_count_);
  // }
  

  void publishReservationGlobal(const GridPose& goal_local_g) {
    auto [wx, wy] = gridToWorld(goal_local_g.x, goal_local_g.y);

    double xg, yg;
    if (!toGlobal(wx, wy, xg, yg)) return;

    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = this->now();
    ps.header.frame_id = robot_id_; // 수정 전: global_frame_
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

  // void publishClusterReservationKey(int64_t key) {
  //   std_msgs::msg::Int64 msg;
  //   msg.data = key;
  //   cluster_reserve_pub_->publish(msg);

  //   RCLCPP_INFO(this->get_logger(),
  //     "[%s][CL_RES_PUB] topic=%s key=%ld",
  //     robot_id_.c_str(), cluster_reserve_pub_->get_topic_name(), (long)key);
  // }

  void onReservePoint(const geometry_msgs::msg::PoseStamped& msg, const std::string& sender_id) {
    if (sender_id == robot_id_) return; // 수정 전: msg.header.frame_id -> sender_id, global_frame_ -> robot_id_
    // // ✅ 내 reservation 토픽이면 무시
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

  // void onReserveClusterKey(const std_msgs::msg::Int64& msg, const std::string& src_topic) {
  //   // ✅ 내가 발행하는 토픽이면 무시
  //   if (src_topic == cluster_reserve_pub_->get_topic_name()) return;

  //   reserved_cluster_keys_[msg.data] = this->now();

  //   RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
  //     "[%s][CL_RES_SUB] from=%s key=%ld size=%zu",
  //     robot_id_.c_str(), src_topic.c_str(), (long)msg.data, reserved_cluster_keys_.size());
  // }

  double reservePenaltyGlobal(double goal_x_g, double goal_y_g) {
    auto now = this->now();

    for (auto it = reservations_.begin(); it != reservations_.end();) {
      if ((now - it->second.stamp).seconds() > reserve_ttl_s_) it = reservations_.erase(it);
      else ++it;
    }

    if (reservations_.empty()) {
        // 이 로그가 계속 뜬다면 onReservePoint에서 저장이 안 되고 있는 겁니다.
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


  // void cleanupClusterReservations() {
  //   auto now = this->now();
  //   for (auto it = reserved_cluster_keys_.begin(); it != reserved_cluster_keys_.end(); ) {
  //     if ((now - it->second).seconds() > cluster_reserve_ttl_s_) it = reserved_cluster_keys_.erase(it);
  //     else ++it;
  //   }
  // }

  // bool isClusterReserved(int64_t key) {
  //   cleanupClusterReservations();
  //   return reserved_cluster_keys_.find(key) != reserved_cluster_keys_.end();
  // }


  bool pickBestFrontierByUtility(
    const GridPose &robot_g,
    const std::vector<GridPose> &reps,
    const std::vector<uint8_t> &blockedMask,
    const std::vector<uint8_t> &obsInfl,
    const std::vector<uint8_t> &obsRaw,
    GridPose &out_goal,
    std::vector<GridPose> &out_path,
    std::vector<CandDbg> &out_dbg
    //int64_t &out_cluster_key   // ⭐ 추가
    )
  {
    // blockedmask - goal 검사
    // obsinfl - astar 경로 검사
    // obsraw - frontier clearance, astar 경로 path tail 검사

    RCLCPP_INFO(get_logger(), "계획 수립 시작!");
    // 디버그용
    int c_total = 0;
    int c_not_trav = 0;
    int c_clearance = 0;
    int c_blocked = 0;
    int c_astar_empty = 0;
    int c_tail_close = 0;
    int c_toglobal_fail = 0;
    int c_scored = 0;
    int c_bad_score = 0;
    int c_selected = 0;
    int c_too_close_robot = 0;
    // 디버그용

    if (reps.empty()) return false;

    int ig_cells = (int)std::ceil(info_gain_radius_m_ / map_.info.resolution);
    RCLCPP_WARN(this->get_logger(), "[%s] ig_cells=%d res=%f info_gain_radius_m=%f",
            robot_id_.c_str(), ig_cells, map_.info.resolution, info_gain_radius_m_);
    int v_cells  = (int)std::ceil(utility_radius_m_   / map_.info.resolution);

    double bestScore = -1e18;
    bool found = false;
    //int64_t best_key = 0;

    for (const auto& rep : reps) {
      const auto& g = rep;

      // // ✅ 핵심: 상대가 예약한 클러스터면 통째로 제외
      // if (isClusterReserved(rep.key)) continue;
      // c_total++;
    
    // goal이 탐사 가능영역인지 확인
    if (!isTraversable(g.x, g.y)) { c_not_trav++; continue; }
    
    // goal이 clearance + obs의 inflation를 거쳤을 때, 도달할 수 있는 영역인지 확인 (ontimer에서 이미 검사했으므로 x)
    // int clearance_cells = (int)std::ceil(frontier_clearance_m_ / map_.info.resolution);
    // if (isFrontierTooCloseToObstacle(g, obsRaw, clearance_cells)) { c_clearance++; continue; }
    
    // goal이 unknown, obs, laser인지 확인
    if (blockedMask[IDX(g.x,g.y,(int)map_.info.width)]) { c_blocked++; continue; }
    
    // 로봇이 goal에 도달했는지 확인
    double min_goal_dist_m = 0.6;  
    double d0 = std::hypot(
      (g.x - robot_g.x) * map_.info.resolution,
      (g.y - robot_g.y) * map_.info.resolution
    );
    if (d0 < min_goal_dist_m) { c_too_close_robot++; continue;}

    // ⭐ [수정 포인트] A* 돌리기 전에 좌표 변환부터 먼저 합니다!
    auto [wx, wy] = gridToWorld(g.x, g.y);
    double gx, gy;
    if (!toGlobal(wx, wy, gx, gy)) { c_toglobal_fail++; continue; }

    // ⭐ [수정 포인트] 남이 예약했는지 확인해서, 예약된 곳이면 아예 여기서 컷! (연산 절약)
    double rp = reservePenaltyGlobal(gx, gy);
    if (rp > 0.5) { 
        // 예약된 곳이면 굳이 A* 경로를 안 짜고 다음 프론티어 후보로 넘어갑니다.
        // 만약 페널티만 주고 탐사를 허용하고 싶다면 continue 대신 점수 깎는 로직을 쓰면 됩니다.
        continue; 
    }

    // 통과한 안전한(예약 안 된) 목표점만 A* 로 경로 생성
    auto astarMask = obsInfl;
    applyKeepOpen(astarMask, robot_g);
  
    auto p = astar(robot_g, g, astarMask);
    if (p.empty()) { c_astar_empty++; continue; }

    // 여기까지 왔다는 건 점수 계산까지 간 후보
    c_scored++;

    // utility 활용을 위한 penalty 계산
    double path_len = (double)p.size() * map_.info.resolution;
    double ig = infoGainAround(g, ig_cells);

    // rp 계산은 위에서 이미 했으므로 아래 코드는 수정
    double score = alpha_ * ig - beta_ * path_len; // delta_ * rp는 제외해도 됨 (위에서 이미 컷 했으므로)
    
    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    //   "[%s] cand ig=%.3f path_m=%.2f vp=%.2f rp=%.2f score=%.2f",
    //   robot_id_.c_str(), ig, path_len, vp, rp, score);

    if (!std::isfinite(score) || !std::isfinite(ig) || !std::isfinite(path_len) ||
       !std::isfinite(rp)) { //!std::isfinite(vp) ||
        c_bad_score++;
  //   RCLCPP_WARN(this->get_logger(),
  //     "[%s] BAD score: score=%f ig=%f path=%f vp=%f rp=%f a=%f b=%f g=%f d=%f",
  //     robot_id_.c_str(), score, ig, path_len, vp, rp, alpha_, beta_, gamma_, delta_);
        continue;
    }

    out_dbg.push_back(CandDbg{g, score, ig, rp, path_len});

    // 계산한 score를 기반으로 가장 높은 score를 가지는 goal 값을 선택하여 생성된 경로로 따라가기
    if (score > bestScore) {
      bestScore = score;
      out_goal = g;
      out_path = std::move(p);
      //best_key = rep.key;
      found = true;
      c_selected++;
    }
  }

  RCLCPP_WARN(this->get_logger(),
  "[%s] pickBest END: found=%d bestScore=%.3f out_path=%zu out_goal=(%d,%d)",
  robot_id_.c_str(), (int)found, bestScore, out_path.size(), out_goal.x, out_goal.y);

  RCLCPP_WARN(this->get_logger(),
  "[%s] cand total=%d scored=%d selected=%d badscore=%d | drop: trav=%d clear=%d block=%d close=%d astar=%d tail=%d toG=%d",
  robot_id_.c_str(),
  c_total, c_scored, c_selected, c_bad_score,
  c_not_trav, c_clearance, c_blocked, c_too_close_robot, c_astar_empty, c_tail_close, c_toglobal_fail
  );

  // if (found) {
  //   out_cluster_key = best_key;   // ⭐ 여기
  // }
    return found;
  }
  

  // ---- Motion ----
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

  void publishStop(const char* reason) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_pub_->publish(cmd);

    RCLCPP_ERROR(get_logger(),  "[%s] stop reason=%s", 
        robot_id_.c_str(), reason);
  }

  void publishAvoidCmd() {
    // 경로 추종 중 장애물이 있을 때 회피 로직
    geometry_msgs::msg::Twist cmd;

    double front = minRange(-0.3, 0.3);
    double left  = minRange(0.3, 1.0);
    double right = minRange(-1.0, -0.3);

    if (front < avoid_enter_dist_) {
      cmd.linear.x = -0.05;
      cmd.angular.z = (left > right) ? 0.4 : -0.4;
    } else {
      cmd.linear.x = 0.15;
      cmd.angular.z = 0.0;
    }
    cmd_pub_->publish(cmd);
  }

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
      if (d2 < best_d2) { best_d2 = d2; best_i = i; }
    }
    return best_i;
  }

  void followPathStep() {

    if (path_.empty()) return;

    double side = std::min(minRange(0.7, 1.4), minRange(-1.4, -0.7)); // 좌/우 측면
    double front = minRange(-0.3, 0.3);

    // // 벽 너무 가까우면 경로 추종 중단하고 replanning 유도
    // if (has_scan_ && (side < 0.18 || front < 0.22)) {
    //   path_.clear();
    //   wp_idx_ = 0;
    //   publishStop();
    //   return;
    // }

    wp_idx_ = findNearestIndexOnPath(path_, wp_idx_, 25);
    int target_idx = std::min(wp_idx_ + 3, (int)path_.size() - 1);

    const auto& target = path_[target_idx];
    auto [tx, ty] = gridToWorld(target.x, target.y);

    double dxw = tx - robot_.x;
    double dyw = ty - robot_.y;

    double dist = std::hypot(dxw, dyw);
    double ang_err = normAngle(std::atan2(dyw, dxw) - robot_.yaw);

    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 500,
    "[%s] 경로 추종 중: 목표 지점 (%f, %f)", robot_id_.c_str(), tx, ty);

    if (dist < reach_dist_) {
      wp_idx_ = target_idx + 1;
      if (wp_idx_ >= (int)path_.size()) 
      {
        RCLCPP_WARN(get_logger(), "[%s] clear path!", robot_id_.c_str()); 
        path_.clear();
        publishStop("path clear from path step");
        return;
      }
    }

    geometry_msgs::msg::Twist cmd;
    if (std::abs(ang_err) > 0.6) {
      cmd.linear.x  = 0.02;
      cmd.angular.z = clampd(2.5 * ang_err, -max_ang_vel_, max_ang_vel_);
    } else {
      cmd.linear.x  = clampd(0.4 * dist, 0.03, max_lin_vel_);
      cmd.angular.z = clampd(2.0 * ang_err, -max_ang_vel_, max_ang_vel_);
    }

    if (has_scan_) {
      double k = clampd((side - 0.18) / (0.35 - 0.18), 0.0, 1.0); // 0~1
      cmd.linear.x = std::max(cmd.linear.x * k, 0.05);
    }

    cmd_pub_->publish(cmd);
  }

  // 웜업
  struct LocalMapStats { int known{0}; int occ{0}; int minv{101}; int maxv{-1}; };

  LocalMapStats computeLocalMapStats(const GridPose& center_g, double radius_m) const {
    LocalMapStats s;
    if (!has_map_) return s;

    const int W = (int)map_.info.width;
    const double res = map_.info.resolution;

    int r = (int)std::ceil(radius_m / res);
    int x0 = std::max(0, center_g.x - r);
    int x1 = std::min(W-1, center_g.x + r);
    int y0 = std::max(0, center_g.y - r);
    int y1 = std::min((int)map_.info.height - 1, center_g.y + r);

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
    cmd.linear.x  = warmup_lin_vel_;
    cmd.angular.z = warmup_yaw_rate_;
    cmd_pub_->publish(cmd);
  }

  // 경로, 프론티어, 인플레이션 마커 시각화
  void publishPathMarker(const std::vector<GridPose>& path) {
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

  void publishFrontierMarkers(const std::vector<GridPose>& frontiers) {
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

  void publishCandidateTextMarkers(const std::vector<CandDbg>& dbg_sorted, int topN)
  {
    if (!enable_viz_ || !cand_text_pub_ || !has_map_) return;

    visualization_msgs::msg::MarkerArray arr;

    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);

    int N = std::min(topN, (int)dbg_sorted.size());
    for (int i = 0; i < N; ++i) {
      const auto& c = dbg_sorted[i];
      auto [wx, wy] = gridToWorld(c.g.x, c.g.y);

      visualization_msgs::msg::Marker m;
      m.header.stamp = this->now();
      m.header.frame_id = map_frame_;
      m.ns = robot_id_ + "_cand_text";
      m.id = i;

      m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      m.action = visualization_msgs::msg::Marker::ADD;

      m.pose.position.x = wx;
      m.pose.position.y = wy;
      m.pose.position.z = 0.25;
      m.pose.orientation.w = 1.0;

      m.scale.z = 0.18;
      m.color.a = 1.0;
      m.color.r = 0.5;
      m.color.g = 0.5;
      m.color.b = 0.5;

      char buf[160];
      std::snprintf(buf, sizeof(buf),
        "S:%.2f IG:%.2f RP:%.2f L:%.0f",
        c.score, c.ig, c.rp, c.path_len
      );
      m.text = buf;

      arr.markers.push_back(m);
    }

    cand_text_pub_->publish(arr);
  }

  // inflation marker: obsInfl를 큐브 리스트로 간단히 시각화
  void publishInflationMaskMarker(const std::vector<uint8_t>& obsInfl, const GridPose& center_g) {
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

  void publishClusterRings(const std::vector<GridPose>& pts,
                         const std::vector<int>& labels,
                         const std::vector<GridPose>& representatives)
{
  if (!enable_viz_ || !has_map_) return;

  visualization_msgs::msg::MarkerArray marker_array;
    int max_id = -1;
    for (int l : labels) if (l > max_id) max_id = l;
    if (max_id < 0) return;

    std::vector<std::vector<int>> clusters(max_id + 1);
    for (int i = 0; i < (int)pts.size(); ++i) {
        if (labels[i] >= 0) clusters[labels[i]].push_back(i);
    }

    double res = map_.info.resolution;
    auto now = rclcpp::Clock().now();

    for (int i = 0; i < (int)clusters.size(); ++i) {
        const auto& idxs = clusters[i];
        
        // 로봇의 판단 기준과 시각화 기준을 일치시킴
        if ((int)idxs.size() < dbscan_min_pts_) continue;

        double sx = 0.0, sy = 0.0;
        for (int id : idxs) { sx += pts[id].x; sy += pts[id].y; }
        double cx = sx / idxs.size();
        double cy = sy / idxs.size();

        double max_dist_sq = 0.0;
        for (int id : idxs) {
            double dx = (pts[id].x - cx) * res;
            double dy = (pts[id].y - cy) * res;
            double d2 = dx*dx + dy*dy;
            if (d2 > max_dist_sq) max_dist_sq = d2;
        }
        double radius = std::sqrt(max_dist_sq);
        
        // 반지름 필터링도 동일하게 적용
        if (radius < (dbscan_eps_m_ * 0.5)) continue;

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = now;
        marker.ns = "frontier_clusters";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = map_.info.origin.position.x + cx * res;
        marker.pose.position.y = map_.info.origin.position.y + cy * res;
        marker.pose.position.z = 0.1;

        marker.scale.x = radius * 2.0;
        marker.scale.y = radius * 2.0;
        marker.scale.z = 0.05;

        // 알록달록 색상 입히기
        marker.color.r = (float)((i * 70) % 255) / 255.0f;
        marker.color.g = (float)((i * 150) % 255) / 255.0f;
        marker.color.b = (float)((i * 210) % 255) / 255.0f;
        marker.color.a = 0.4f;

        marker_array.markers.push_back(marker);
    }
    cluster_marker_pub_->publish(marker_array); 
}

  bool toGlobal(double x_local, double y_local, double& x_g, double& y_g) {
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

  

  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    map_ = *msg;
    has_map_ = true;
    //map_frame_ = msg->header.frame_id;
  }

  bool shouldReplanByIG() {
    if (!has_goal_ || !has_map_) return false;
    if (path_.empty()) return false;

    auto now = this->now();

    // 너무 자주 체크하지 않기
    if ((now - last_replan_check_).seconds() < replan_check_period_s_) return false;
    last_replan_check_ = now;

    // 목표를 잡은지 너무 짧으면 안 바꾸기(덜덜 떨림 방지)
    if ((now - goal_commit_start_).seconds() < min_commit_time_s_) return false;

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

  void onTimer() {
    if (!has_map_) return;

    if (!updateRobotPoseFromTF()) {
      publishStop("no tf!");
      RCLCPP_INFO_THROTTLE(get_logger(),*this->get_clock(), 1000, "no TF!!");
      return;
    }


    //publishVisitedPointIfDue();


    GridPose robot_g = worldToGrid(robot_.x, robot_.y);
    
  
    // warmup
    if (warmup_enable_ && !warmup_done_) {
      auto st = computeLocalMapStats(robot_g, warmup_radius_m_);
      double t = (this->now() - warmup_state_start_).seconds();

      bool ready = (st.known >= warmup_min_known_cells_) && (st.occ >= warmup_min_occ_cells_);
      bool timeout = (t >= warmup_max_time_s_);

      RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000,
        "[%s] warmup known=%d occ=%d t=%.1f ready=%d",
        robot_id_.c_str(), st.known, st.occ, t, (int)ready);

      if (!ready && !timeout) {
        path_.clear();
        wp_idx_ = 0;
        avoiding_ = false;
        progress_inited_ = false;
        publishWarmupMotion();
        return;
      }
      warmup_done_ = true;
      publishStop("warmup");
      return;
    }

    double front = minRange(-0.3, 0.3);
    if (!avoiding_ && has_scan_ && front < avoid_enter_dist_) {
      avoiding_ = true;
      path_.clear();
      wp_idx_ = 0;
      progress_inited_ = false;
    }
    if (avoiding_) {
      static int clear_cnt = 0;
      if (has_scan_ && front > avoid_exit_dist_) {
        avoiding_ = false;
        path_.clear();
        wp_idx_ = 0;
        clear_cnt++;
        if (clear_cnt > 5){
          avoiding_ = false;
          clear_cnt = 0;
        }
      } else {
        publishAvoidCmd();
        return;
      }
    }

    // masks
    auto obsInfl = buildObstacleInflatedMask(); // A*/ reachable 통과 불가
    auto obsRaw  = buildObstacleRawMask(); // path tail/ frontier clearance
    auto blockedMask = buildBlockedMask(); // goal 후보 제외 (unknown도 막기)


    // inflation viz 로봇 주변만 시각화
    if (enable_viz_) publishInflationMaskMarker(obsInfl, robot_g);

    // If path exists -> follow + stuck check
    if (!path_.empty()) {
      // ✅ NEW: IG 떨어지면 갈아타기 (path가 있어도 재계획)
    if (shouldReplanByIG()) {
      path_.clear();
      wp_idx_ = 0;
      progress_inited_ = false;
      publishStop("replan by IG");
      // return 하지 말고 아래로 내려가서 새 goal 뽑게 만들기
    } else {
      // ✅ 기존 follow + stuck 체크 로직을 여기서 그대로 수행
      int new_wp = findNearestIndexOnPath(path_, wp_idx_, 25);

      if (!progress_inited_) {
        last_progress_time_ = this->now();
        last_progress_x_ = robot_.x;
        last_progress_y_ = robot_.y;
        progress_inited_ = true;
      }

      double moved = std::hypot(robot_.x - last_progress_x_, robot_.y - last_progress_y_);
      bool progressed = (moved > stuck_min_move_m_);

      if (progressed) {
        last_progress_time_ = this->now();
        last_progress_x_ = robot_.x;
        last_progress_y_ = robot_.y;
      } else {
        double dt = (this->now() - last_progress_time_).seconds();
        if (dt > stuck_timeout_s_) {
          RCLCPP_WARN(get_logger(), "[%s] stuck %.2fs -> replan", robot_id_.c_str(), dt);
          path_.clear();
          wp_idx_ = 0;
          progress_inited_ = false;
          publishStop("stuck");
          return;
        }
      }

      wp_idx_ = new_wp;
      followPathStep();
      return;
    }
    }

    // 1) detect frontiers
    auto frontiers = detectFrontiers(robot_g);
    if (frontiers.empty()) {
      publishStop("no frontiers");
      return;
    }

    // 2) clearance filter
    int clearance_cells = (int)std::ceil(frontier_clearance_m_ / map_.info.resolution);
    frontiers.erase(
      std::remove_if(frontiers.begin(), frontiers.end(),
        [&](const GridPose& f){
          return isFrontierTooCloseToObstacle(f, obsRaw, clearance_cells);
        }),
      frontiers.end()
    );
    if (frontiers.empty()) {
      publishStop("no frontiers before reachable");
      return;
    }

    // 3) reachable filter (obsInfl 기반으로 BFS)
    auto reachMask = obsInfl;
    applyKeepOpen(reachMask, robot_g);

    auto reachable = buildReachableMaskFromStart(robot_g, reachMask);
    filterFrontiersByReachable(frontiers, reachable);
    if (frontiers.empty()) {
      publishStop("no reachable frontiers");
      return;
    }

    // 4) DBSCAN reps
    // std::vector<GridPose> reps;
    // if (use_dbscan_) {
    //   auto labels = dbscanCluster(frontiers, dbscan_eps_m_, dbscan_min_pts_);
    //   reps = computeClusterRepresentatives(frontiers, labels);
    //   if (reps.empty()) reps = frontiers;
    // } else {
    //   reps = frontiers;
    // }

    //std::vector<FrontierRep> reps;
    // 4) DBSCAN reps
    // ⭐ 복잡했던 FrontierRep, labels, makeClusterKeyFromCellCentroid 다 지우고 
    //    가장 심플한 GridPose 리스트로 복원합니다.
    std::vector<GridPose> reps;
    std::vector<int> labels;

    if (use_dbscan_) {
      labels = dbscanCluster(frontiers, dbscan_eps_m_, dbscan_min_pts_);
      reps = computeClusterRepresentatives(frontiers, labels); // 함수 이름 원래대로 (WithKey 아님)
      if (reps.empty()) reps = frontiers;
    } else {
      reps = frontiers;
    }

    if (enable_viz_) {
      publishClusterRings(frontiers, labels, reps); // rep_pts 변환 과정 생략 가능
      publishFrontierMarkers(reps);
    }

    // 5) pick goal by utility + plan
    GridPose goal;
    std::vector<GridPose> new_path;
    std::vector<CandDbg> dbg;
    //int64_t goal_cluster_key = 0;   // ⭐ 추가

    bool planned = pickBestFrontierByUtility(robot_g, reps, blockedMask, obsInfl, obsRaw, goal, new_path, dbg);
    if (!planned) {
      publishStop("no plan");
      RCLCPP_WARN(get_logger(), "[%s] no plan", robot_id_.c_str());
      return;
    }

    // dbg sort
    std::sort(dbg.begin(), dbg.end(),
      [](const CandDbg& a, const CandDbg& b){ return a.score > b.score; }
    );
    publishCandidateTextMarkers(dbg, 10);

    // reservation publish
    publishReservationGlobal(goal);
    //publishClusterReservationKey(goal_cluster_key);  // ⭐ 추가

    path_ = std::move(new_path);
    wp_idx_ = 0;
    progress_inited_ = false;

    current_goal_ = goal;
    has_goal_ = true;
    goal_commit_start_ = this->now();   // 커밋 시작 시각 리셋

    if (enable_viz_) publishPathMarker(path_);

    followPathStep();

    RCLCPP_WARN(get_logger(), "[%s] goal=(%d,%d) pathN=%zu", robot_id_.c_str(), goal.x, goal.y, path_.size());
  }
};

// ---- main ----
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FrontierExplorerMulti>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
