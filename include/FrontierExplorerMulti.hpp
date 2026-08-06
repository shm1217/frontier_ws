#ifndef FRONTIER_EXPLORER_MULTI_HPP_
#define FRONTIER_EXPLORER_MULTI_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
#include <std_msgs/msg/bool.hpp>
#include <unordered_set>
#include <random>

#include "frontier_ws/msg/dynamic_obstacle.hpp"
#include "controller.hpp"

static constexpr int UNKNOWN = -1;

struct GridPose { int x{0}, y{0}; };
struct WorldPose { double x{0}, y{0}, yaw{0}; };

struct BlacklistedGoal {
    GridPose g;
    rclcpp::Time stamp;
};

static inline int IDX(int x, int y, int w) { return y * w + x; }
static inline double clampd(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

static inline const int dx8[8] = { 1, 1, 0,-1,-1,-1, 0, 1};
static inline const int dy8[8] = { 0, 1, 1, 1, 0,-1,-1,-1};
static inline const int dx4[4] = { 1,-1, 0, 0};
static inline const int dy4[4] = { 0, 0, 1,-1};

class FrontierExplorerMulti : public rclcpp::Node {
public:
    FrontierExplorerMulti(); // 생성자

private:
    // ---------------------------------------------------------
    // 1. 초기화 관련 함수
    // ---------------------------------------------------------
    void declare_params();
    void setup_ros_interfaces();

    // ---------------------------------------------------------
    // 2. 콜백 함수들
    // ---------------------------------------------------------
    // (1) 좌표 변환
    bool inBounds(int x, int y) const;
    GridPose worldToGrid(double wx, double wy) const;
    std::pair<double,double> gridToWorld(int gx, int gy) const;
    bool toGlobal(double x_local, double y_local, double& x_g, double& y_g);
    bool toLocal(double x_g, double y_g, double& x_local, double& y_local);


    // (2) robot pose
    bool updateRobotPoseFromTF();

    // (3) frontier 탐색 및 셀 검사, 로봇 발 밑 열어두는 함수
    bool isTraversable(int x, int y) const;
    bool isFrontierCell(int x, int y) const;
    std::vector<GridPose> detectFrontiers(const GridPose &robot_g) const;
    void applyKeepOpen(std::vector<uint8_t>& mask, const GridPose& robot_g) const;
    void applyGoalKeepOpen(std::vector<uint8_t>& mask, const GridPose& goal_g, const std::vector<uint8_t>& obsRaw) const;
    bool isFrontierTooCloseToObstacle(const GridPose& f,
                                    const std::vector<uint8_t>& obsRaw,
                                    int radius_cells) const;
    std::vector<uint8_t> buildReachableMaskFromStart(const GridPose& start,
                                                   const std::vector<uint8_t>& reachmask) const;
    void filterFrontiersByReachable(std::vector<GridPose>& frontiers,
                                  const std::vector<uint8_t>& reachable) const;

    // (4) 장애물 마스크
    std::vector<uint8_t> buildObstacleInflatedMask() const;
    std::vector<uint8_t> buildObstacleRawMask() const;
    std::vector<uint8_t> buildBlockedMask() const;
    void applyOtherRobotFootprints(std::vector<uint8_t>& mask);
    bool hasOtherRobotOnCurrentPath();

    // (5) 경로 계획 및 dbscan clustering
    std::vector<GridPose> astar(const GridPose &start, const GridPose &goal,
                             const std::vector<uint8_t> &astarMask) const;
    double distMeters(const GridPose& a, const GridPose& b) const;
    std::vector<int> regionQuery(const std::vector<GridPose>& pts, int idx, double eps_m) const;
    std::vector<int> dbscanCluster(const std::vector<GridPose>& pts, double eps_m, int min_pts) const;
    std::vector<GridPose> computeClusterRepresentatives(const std::vector<GridPose>& pts,
                                                    const std::vector<int>& labels) const;

    // (6) utility
    double infoGainAround(const GridPose& g, int radius_cells) const;
    void publishReservationGlobal(const GridPose& goal_local_g);
    void onReservePoint(const geometry_msgs::msg::PoseStamped& msg, const std::string& sender_id);
    void publishRobotPositionGlobal();
    void onRobotPosition(const geometry_msgs::msg::PoseStamped& msg, const std::string& sender_id);
    double reservePenaltyGlobal(double goal_x_g, double goal_y_g);
    bool hasHigherPriorityReservation(double goal_x_g, double goal_y_g);

    bool pickBestFrontierByUtility(
    const GridPose &robot_g,
    const std::vector<GridPose> &reps,
    const std::vector<uint8_t> &blockedMask,
    const std::vector<uint8_t> &obsInfl,
    const std::vector<uint8_t> &obsRaw,
    GridPose &out_goal,
    std::vector<GridPose> &out_path
    );


    void publishStop(const char* reason);
    void followPathStep();
    bool updateDynamicController();
    double minRange(double a_min, double a_max) const;
    geometry_msgs::msg::Twist applyDynamicSafetyFilter(geometry_msgs::msg::Twist cmd) const;
    void onDwbCmd(const geometry_msgs::msg::Twist::SharedPtr msg);
    void obsCallback(const frontier_ws::msg::DynamicObstacle::SharedPtr msg);
    void cancelDwbGoal();
    void clearPathAndCancel();
    nav_msgs::msg::Path makeNavPath() const;

    bool isRobotStuck();
    void resetStuckCheck();

    // (7) marker
    void publishPathMarker(const std::vector<GridPose>& path);
    void publishFrontierMarkers(const std::vector<GridPose>& frontiers);
    void publishInflationMaskMarker(const std::vector<uint8_t>& obsInfl, const GridPose& center_g);
    void publishClusterRings(const std::vector<GridPose>& pts,
                         const std::vector<int>& labels,
                         const std::vector<GridPose>& representatives);

    void publishMapDelta();
    void onGateGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    bool shouldReplanByIG();

    void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void onTimer();

    void addToBlacklist(const GridPose& g);
    bool isBlacklisted(const GridPose& g) const;

    // ---------------------------------------------------------
    // 3. 멤버 변수들
    // ---------------------------------------------------------
    // ---------------------------------------------------------
    // ---------- ROS interfaces ----------
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr dwb_cmd_sub_;
    rclcpp::Subscription<frontier_ws::msg::DynamicObstacle>::SharedPtr obs_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr dynamic_cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    using FollowPath = nav2_msgs::action::FollowPath;
    rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_;
    rclcpp_action::ClientGoalHandle<FollowPath>::SharedPtr follow_path_goal_handle_;
    std::string follow_path_action_name_;
    bool path_sent_to_dwb_ = false;
    uint64_t active_path_id_ = 0;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr frontier_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr infl_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_marker_pub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::vector<BlacklistedGoal> blacklisted_goals_;
    double blacklist_ttl_s_ = 20.0;       // <- blacklist_ttl_s (cpp 동일)
    double blacklist_radius_m_ = 1.5;     // <- 파라미터 미등록 (내부 상수)

    // ---------- Params / topics ----------
    std::string robot_id_;
    std::string map_topic_, scan_topic_, cmd_topic_, dwb_cmd_topic_, dynamic_cmd_topic_;
    std::string map_frame_, base_frame_;
    std::string global_frame_;
    double tf_timeout_s_ = 0.10;          // <- tf_timeout_s

    std::string path_marker_topic_, frontier_marker_topic_, infl_marker_topic_, cluster_marker_topic_;
    bool enable_viz_ = true;              // <- enable_viz

    int obstacle_threshold_ = 60;         // <- obstacle_threshold
    int free_threshold_ = 50;             // <- free_threshold
    double inflation_radius_m_ = 0.25;    // <- inflation_radius_m
    double frontier_search_radius_m_ = 6.0; // <- frontier_search_radius_m

    double frontier_clearance_m_ = 0.25;  // <- frontier_clearance_m
    double path_clearance_m_ = 0.15;      // <- path_clearance_m

    int keep_open_cells_ = 2;             // <- keep_open_cells

    // DBSCAN
    double dbscan_eps_m_ = 0.25;          // <- dbscan_eps_m
    int dbscan_min_pts_ = 10;             // <- dbscan_min_pts
    bool use_dbscan_ = true;              // <- use_dbscan

    // Laser mask
    sensor_msgs::msg::LaserScan last_scan_;
    bool has_scan_ = false;               // 내부 상태 (초기화 필수)
    std::vector<uint8_t> laser_blocked_;
    std::chrono::steady_clock::time_point last_laser_update_;
    double laser_block_ttl_ = 1.0;        // <- laser_block_ttl
    double laser_inflation_radius_m_ = 0.12; // <- laser_inflation_radius_m
    double laser_obstacle_max_range_ = 0.40;

    std::shared_ptr<Controller> dynamic_controller_;
    geometry_msgs::msg::Twist last_dwb_cmd_;
    bool dynamic_override_active_ = false;
    double dynamic_max_linear_speed_ = 0.08;
    double dynamic_max_angular_speed_ = 0.8;
    double dynamic_stop_distance_ = 0.32;
    double dynamic_slow_distance_ = 0.55;

    // Stuck
    double stuck_timeout_s_ = 3.0;        // <- stuck_timeout_s
    double stuck_min_move_m_ = 0.05;      // <- stuck_min_move_m
    rclcpp::Time last_progress_time_{0, 0, RCL_ROS_TIME};
    double last_progress_x_ = 0.0;        // 내부 상태 (초기화 필수)
    double last_progress_y_ = 0.0;        // 내부 상태 (초기화 필수)
    bool progress_inited_ = false;        // 내부 상태 (초기화 필수)

    // Map / pose / path
    nav_msgs::msg::OccupancyGrid map_;
    bool has_map_ = false;                // 내부 상태 (초기화 필수)

    WorldPose robot_;
    bool has_pose_ = false;                // 내부 상태 (초기화 필수)

    std::vector<GridPose> path_;
    int wp_idx_ = 0;                       // 내부 상태 (초기화 필수)

    // utility
    double utility_radius_m_ = 1.00;       // <- utility_radius_m (※ 현재 코드 로직 내 미사용, 정리 대상)
    double info_gain_radius_m_ = 1.50;     // <- info_gain_radius_m

    double alpha_ = 10.0, beta_ = 2.0, delta_ = 15.0; // <- alpha_info_gain, beta_path_len, delta_reserve

    double reserve_exclusion_radius_m_ = 2.0; // <- reserve_exclusion_radius_m
    double reserve_ttl_s_ = 6.0;              // <- reserve_ttl_s
    double reserve_refresh_period_s_ = 1.0;

    // reserve point topics
    std::string reserve_out_topic_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr reserve_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr reserve_sub_;
    std::string robot_position_topic_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_position_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_position_sub_;

    struct ReservedGoal {
        std::string src;
        double x{0}, y{0};
        rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    };

    std::unordered_map<std::string, ReservedGoal> reservations_;
    rclcpp::Time last_reservation_pub_{0, 0, RCL_ROS_TIME};
    std::unordered_map<std::string, ReservedGoal> other_robot_positions_;
    rclcpp::Time last_robot_position_pub_{0, 0, RCL_ROS_TIME};
    double robot_position_ttl_s_ = 1.0;
    double robot_position_period_s_ = 0.2;
    double other_robot_radius_m_ = 0.32;

    GridPose current_goal_;
    bool has_goal_ = false;                // 내부 상태 (초기화 필수)

    rclcpp::Time last_replan_check_{0,0,RCL_ROS_TIME};
    double replan_check_period_s_ = 0.5;   // <- replan_check_period_s
    rclcpp::Time last_plan_attempt_{0,0,RCL_ROS_TIME};
    double plan_retry_period_s_ = 0.75;

    rclcpp::Time goal_commit_start_{0,0,RCL_ROS_TIME};
    double min_commit_time_s_ = 2.0;       // <- min_commit_time_s
    rclcpp::Time replan_hold_until_{0,0,RCL_ROS_TIME};
    double replan_stop_hold_s_ = 0.4;

    double ig_drop_thresh_ = 0.10;         // <- ig_drop_thresh
    double goal_initial_ig_ = 0.0;
    double ig_drop_ratio_ = 0.55;          // 선정 시점 IG 대비 이 비율 이하이면 목표 교체
    double ig_drop_baseline_min_ = 0.20;   // 초기 IG가 너무 작은 목표는 비율 비교 제외
    double ig_replan_min_age_s_ = 0.75;    // 새 목표 직후의 불필요한 흔들림 방지

    // ---- Gate 관련 ----
    std::string gate_goal_topic_;          // <- gate_goal_topic
    std::string map_delta_topic_;          // <- map_delta_topic
    double gate_timeout_s_ = 3.0;          // <- gate_timeout_s
    double map_delta_period_s_ = 1.0;      // <- map_delta_period_s

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_delta_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gate_goal_sub_;

    geometry_msgs::msg::PoseStamped gate_goal_;
    bool has_gate_goal_ = false;            // 내부 상태
    bool new_gate_goal_ = false;            // 내부 상태
    rclcpp::Time last_gate_goal_time_{0,0,RCL_ROS_TIME};
    rclcpp::Time last_delta_pub_time_{0,0,RCL_ROS_TIME};

    std::vector<int8_t> prev_map_data_;     // delta 계산용 이전 맵

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr explore_done_sub_;
    bool exploration_done_ = false;         // 내부 상태

    // 추가 | A* 연속 실패 카운트용
    int gate_plan_fail_count_ = 0;          // 내부 상태
    int gate_plan_fail_max_ = 3;            // <- gate_plan_fail_max

    // 로컬 맵 폴백용
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_map_sub_;
    nav_msgs::msg::OccupancyGrid merge_map_;
    nav_msgs::msg::OccupancyGrid local_map_;
    bool has_merge_map_ = false;            // 내부 상태
    bool has_local_map_ = false;            // 내부 상태
    rclcpp::Time last_merge_map_time_{0,0,RCL_ROS_TIME};
    rclcpp::Time last_local_map_time_{0,0,RCL_ROS_TIME};
    std::string local_map_topic_;           // <- local_map_topic
    double merge_map_stale_s_ = 5.0;        // <- merge_map_stale_s
    bool using_local_map_ = false;          // 내부 상태

    void onLocalMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void selectActiveMap();

};

#endif // FRONTIER_EXPLORER_MULTI_HPP_
