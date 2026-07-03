#ifndef FRONTIER_EXPLORER_MULTI_HPP_
#define FRONTIER_EXPLORER_MULTI_HPP_

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
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

static constexpr int UNKNOWN = -1;

struct GridPose { int x{0}, y{0}; };
struct WorldPose { double x{0}, y{0}, yaw{0}; };

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

    // (2) robot pose
    bool updateRobotPoseFromTF();

    // (3) frontier 탐색 및 셀 검사, 로봇 발 밑 열어두는 함수
    bool isTraversable(int x, int y) const;
    bool isFrontierCell(int x, int y) const;
    std::vector<GridPose> detectFrontiers(const GridPose &robot_g) const;
    void applyKeepOpen(std::vector<uint8_t>& mask, const GridPose& robot_g) const;
    void applyGoalKeepOpen(std::vector<uint8_t>& mask, const GridPose& goal_g) const;
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
    double reservePenaltyGlobal(double goal_x_g, double goal_y_g);

    bool pickBestFrontierByUtility(
    const GridPose &robot_g,
    const std::vector<GridPose> &reps,
    const std::vector<uint8_t> &blockedMask,
    const std::vector<uint8_t> &obsInfl,
    const std::vector<uint8_t> &obsRaw,
    GridPose &out_goal,
    std::vector<GridPose> &out_path
    );


    static double normAngle(double a);
    double minRange(double a_min, double a_max);
    void publishStop(const char* reason);
    void publishAvoidCmd();
    int findNearestIndexOnPath(const std::vector<GridPose>& path, int start_idx, int window);
    void followPathStep();

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

    // ---------------------------------------------------------
    // 3. 멤버 변수들 
    // ---------------------------------------------------------
    // ---------- ROS interfaces ----------
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr frontier_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr infl_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_marker_pub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    GridPose blacklisted_goal_ = {-1, -1};
    double blacklist_radius_m_ = 1.5;

    // ---------- Params / topics ----------
    std::string robot_id_;
    std::string map_topic_, cmd_topic_, scan_topic_;
    std::string map_frame_, base_frame_;
    std::string global_frame_;
    double tf_timeout_s_ = 0.1;

    std::string path_marker_topic_, frontier_marker_topic_, infl_marker_topic_, cluster_marker_topic_;
    bool enable_viz_ = true;

    int obstacle_threshold_ = 60;
    int free_threshold_ = 50;
    double inflation_radius_m_ = 0.2;
    double frontier_search_radius_m_ = 8.0;

    double max_lin_vel_ = 0.2;
    double max_ang_vel_ = 0.8;
    double reach_dist_ = 0.3;

    bool avoiding_ = false;           // 초기화 필수
    double avoid_enter_dist_ = 0.3;
    double avoid_exit_dist_ = 0.45;

    double frontier_clearance_m_ = 0.15;
    double path_clearance_m_ = 0.1;

    int keep_open_cells_ = 2;

    double e_prev_ = 0.0;             // 초기화 필수
    double Kp = 0.8, Kd = 0.1;

    // DBSCAN
    double dbscan_eps_m_ = 0.3;
    int dbscan_min_pts_ = 5;
    bool use_dbscan_ = true;          // 초기화 필수

    // Laser mask
    sensor_msgs::msg::LaserScan last_scan_;
    bool has_scan_ = false;           // 초기화 필수
    std::vector<uint8_t> laser_blocked_;
    std::chrono::steady_clock::time_point last_laser_update_;
    double laser_block_ttl_ = 1.0;
    double laser_inflation_radius_m_ = 0.15;

    // Stuck
    double stuck_timeout_s_{5.0};
    double stuck_min_move_m_{0.02};
    rclcpp::Time last_progress_time_{0, 0, RCL_ROS_TIME};
    double last_progress_x_ = 0.0;    // 초기화 필수
    double last_progress_y_ = 0.0;    // 초기화 필수
    bool progress_inited_ = false;    // 초기화 필수

    // Map / pose / path
    nav_msgs::msg::OccupancyGrid map_;
    bool has_map_ = false;            //

    WorldPose robot_;
    bool has_pose_ = false;           // 초기화 필수

    std::vector<GridPose> path_;
    int wp_idx_ = 0;                  // 초기화 필수

    // utility
    double utility_radius_m_ = 1.0;
    double info_gain_radius_m_ = 1.5;

    double alpha_ = 1.0, beta_ = 1.0, delta_ = 1.0;

    double reserve_exclusion_radius_m_ = 1.5;
    double reserve_ttl_s_ = 5.0;

    // reserve point topics
    std::string reserve_out_topic_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr reserve_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr reserve_sub_;

    struct ReservedGoal {
        std::string src; 
        double x{0}, y{0};
        rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    };
    
    std::unordered_map<std::string, ReservedGoal> reservations_;

    GridPose current_goal_;
    bool has_goal_ = false;           // 초기화 필수

    rclcpp::Time last_replan_check_{0,0,RCL_ROS_TIME};
    double replan_check_period_s_ = 2.0;  

    rclcpp::Time goal_commit_start_{0,0,RCL_ROS_TIME};
    double min_commit_time_s_ = 3.0;       

    double ig_drop_thresh_ = 0.5;    

    // ---- Gate 관련 ----
    std::string gate_goal_topic_;
    std::string map_delta_topic_;
    double gate_timeout_s_{10.0};
    double map_delta_period_s_{1.0};

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_delta_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gate_goal_sub_;

    geometry_msgs::msg::PoseStamped gate_goal_;
    bool has_gate_goal_{false};
    bool new_gate_goal_{false};
    rclcpp::Time last_gate_goal_time_{0,0,RCL_ROS_TIME};
    rclcpp::Time last_delta_pub_time_{0,0,RCL_ROS_TIME};

    std::vector<int8_t> prev_map_data_; // delta 계산용 이전 맵

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr explore_done_sub_;
    bool exploration_done_{false};

};

#endif // FRONTIER_EXPLORER_MULTI_HPP_