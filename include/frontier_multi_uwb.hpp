#ifndef FRONTIER_MULTI_UWB_HPP_
#define FRONTIER_MULTI_UWB_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
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

#include "frontier_ws/msg/dynamic_obstacle.hpp"
#include "controller.hpp"

static constexpr int UNKNOWN = -1;

struct GridPose { int x{0}, y{0}; };
struct WorldPose { double x{0}, y{0}, yaw{0}; };

struct BlacklistedGoal {
    GridPose g;
    rclcpp::Time stamp;
    double ttl_s{8.0};
    double radius_m{0.60};
};

static inline int IDX(int x, int y, int w) { return y * w + x; }
static inline double clampd(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

static inline const int dx8[8] = { 1, 1, 0,-1,-1,-1, 0, 1};
static inline const int dy8[8] = { 0, 1, 1, 1, 0,-1,-1,-1};
static inline const int dx4[4] = { 1,-1, 0, 0};
static inline const int dy4[4] = { 0, 0, 1,-1};

class FrontierExplorerMulti : public rclcpp::Node {
public:
    FrontierExplorerMulti(); 

private:
    // ---------------------------------------------------------
    // 초기화 관련 함수 
    // ---------------------------------------------------------
    void declare_params();
    void setup_ros_interfaces();

    // ---------------------------------------------------------
    // 콜백 함수
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
    std::vector<GridPose> detectFrontiers(
        const GridPose &robot_g, double search_radius_m) const;
    void applyKeepOpen(std::vector<uint8_t>& mask, const GridPose& robot_g) const;
    bool isFrontierTooCloseToObstacle(const GridPose& f,
                                    const std::vector<uint8_t>& obsRaw,
                                    int radius_cells) const;
    std::vector<uint8_t> buildReachableMaskFromStart(const GridPose& start,
                                                   const std::vector<uint8_t>& reachmask) const;
    void filterFrontiersByReachable(std::vector<GridPose>& frontiers,
                                  const std::vector<uint8_t>& reachable) const;

    // (4) 장애물 마스크
    std::vector<uint8_t> buildObstacleInflatedMask(bool include_laser = true) const;
    std::vector<uint8_t> buildObstacleRawMask() const;
    std::vector<int> buildClearanceCostMap(
        const std::vector<uint8_t>& obsRaw) const;
    void applyOtherRobotFootprints(std::vector<uint8_t>& mask);
    bool hasOtherRobotOnCurrentPath();
    bool isCurrentPathBlocked(const std::vector<uint8_t>& obstacle_mask);

    // (5) 경로 계획 및 dbscan clustering 
    std::vector<GridPose> astar(const GridPose &start, const GridPose &goal,
                             const std::vector<uint8_t> &astarMask) const;
    std::vector<GridPose> simplifyPath(
        const std::vector<GridPose>& path,
        const std::vector<uint8_t>& obstacle_mask) const;
    bool lineOfSightCost(
        const GridPose& from, const GridPose& to,
        const std::vector<uint8_t>& obstacle_mask, int& cost) const;
    double distMeters(const GridPose& a, const GridPose& b) const;
    std::vector<int> regionQuery(
        const std::vector<GridPose>& pts,
        const std::vector<int>& frontier_index,
        int idx, double eps_m) const;
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

    bool pickBestFrontierByUtility(
    const GridPose &robot_g,
    const std::vector<GridPose> &reps,
    const std::vector<uint8_t> &obsInfl,
    GridPose &out_goal,
    std::vector<GridPose> &out_path
    );


    double minRange(double a_min, double a_max) const;
    void publishStop(const char* reason);
    int findNearestIndexOnPath(const std::vector<GridPose>& path, int start_idx, int window);
    void followPathStep();
    bool updateDynamicController();
    geometry_msgs::msg::Twist applyDynamicSafetyFilter(geometry_msgs::msg::Twist cmd) const;
    void cancelDwbGoal();
    void clearPathAndCancel();
    bool requestLocalCostmapClear(const GridPose& failed_goal);
    nav_msgs::msg::Path makeNavPath() const;
    void onDwbCmd(const geometry_msgs::msg::Twist::SharedPtr msg);

    bool isRobotStuck();
    void resetStuckCheck();

    // (7) marker
    void publishPathMarker(const std::vector<GridPose>& path);
    void publishFrontierMarkers(const std::vector<GridPose>& frontiers);
    void publishInflationMaskMarker(const std::vector<uint8_t>& obsInfl, const GridPose& center_g);
    void publishClusterMarkers(const std::vector<GridPose>& pts,
                               const std::vector<int>& labels);
    void publishSelectedGoalMarker(const GridPose& goal);

    bool shouldReplanByIG();

    void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void onRendezvousAnchor(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void onTimer();

    void addToBlacklist(const GridPose& g, double ttl_s = -1.0,
                        double radius_m = -1.0);
    bool isBlacklisted(const GridPose& g) const;

    // ---------------------------------------------------------
    // 3. 멤버 변수들
    // ---------------------------------------------------------
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rendezvous_anchor_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr dwb_cmd_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr dynamic_cmd_pub_;
    using FollowPath = nav2_msgs::action::FollowPath;
    using FollowPathGoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;
    rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_;
    FollowPathGoalHandle::SharedPtr follow_path_goal_handle_;
    using ClearEntireCostmap = nav2_msgs::srv::ClearEntireCostmap;
    rclcpp::Client<ClearEntireCostmap>::SharedPtr clear_costmap_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr frontier_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr infl_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_marker_pub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::vector<BlacklistedGoal> blacklisted_goals_;
    double blacklist_ttl_s_{3.0};
    double blacklist_radius_m_ = 0.25;
    double dwb_failure_blacklist_ttl_s_{6.0};
    double dwb_failure_blacklist_radius_m_{0.35};
    std::string clear_costmap_service_ = "local_costmap/clear_entirely_local_costmap";
    double dwb_recovery_match_radius_m_ = 0.50;
    bool costmap_clear_in_progress_ = false;
    bool has_dwb_recovery_goal_ = false;
    GridPose dwb_recovery_goal_;

    std::string robot_id_;
    std::string map_topic_, cmd_topic_, dwb_cmd_topic_, dynamic_cmd_topic_, scan_topic_;
    std::string follow_path_action_name_;
    std::string map_frame_, base_frame_;
    std::string global_frame_;
    double tf_timeout_s_ = 0.1;
    bool path_sent_to_dwb_ = false;
    uint64_t active_path_id_ = 0;

    std::string path_marker_topic_, frontier_marker_topic_, infl_marker_topic_, cluster_marker_topic_;
    bool enable_viz_ = true;

    int obstacle_threshold_ = 60;
    int free_threshold_ = 50;
    double inflation_radius_m_ = 0.30;
    double frontier_search_radius_m_ = 8.0;
    double frontier_extended_search_radius_m_ = 12.0;
    bool frontier_full_map_fallback_ = true;

    double frontier_clearance_m_ = 0.4;
    double path_clearance_m_ = 0.55;
    int path_clearance_cost_weight_ = 25;
    std::vector<int> clearance_cost_map_;

    int keep_open_cells_ = 2;

    double dbscan_eps_m_ = 0.25;
    int dbscan_min_pts_ = 10;
    bool use_dbscan_ = true;          

    sensor_msgs::msg::LaserScan last_scan_;
    bool has_scan_ = false;        
    std::vector<uint8_t> laser_blocked_;
    std::chrono::steady_clock::time_point last_laser_update_;
    double laser_block_ttl_ = 1.0;
    double laser_inflation_radius_m_ = 0.25;
    double laser_obstacle_max_range_ = 3.0;

    std::shared_ptr<Controller> dynamic_controller_;
    geometry_msgs::msg::Twist last_dwb_cmd_;
    rclcpp::Time last_dwb_cmd_time_{0, 0, RCL_ROS_TIME};
    double dwb_cmd_timeout_s_ = 1.0;
    double dynamic_max_linear_speed_ = 0.08;
    double dynamic_max_angular_speed_ = 0.8;
    double dynamic_stop_distance_ = 0.32;
    double dynamic_slow_distance_ = 0.55;

    double stuck_timeout_s_{8.0};
    double stuck_min_move_m_{0.02};
    double stuck_grace_s_{8.0};
    bool dwb_motion_cmd_seen_ = false;
    double path_blocked_lookahead_m_ = 1.5;
    double path_blocked_confirm_s_ = 1.0;
    rclcpp::Time path_blocked_since_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_progress_time_{0, 0, RCL_ROS_TIME};
    double last_progress_x_ = 0.0;    
    double last_progress_y_ = 0.0;    
    bool progress_inited_ = false;    

    nav_msgs::msg::OccupancyGrid map_;
    bool has_map_ = false;            

    WorldPose robot_;
    bool has_pose_ = false;           

    std::vector<GridPose> path_;
    int wp_idx_ = 0;                 

    double info_gain_radius_m_ = 1.5;
    double alpha_ = 10.0, beta_ = 2.0, delta_ = 15.0;
    std::string rendezvous_anchor_topic_ = "rendezvous_anchor";
    double rendezvous_command_ttl_s_ = 3.0;
    double rendezvous_arrival_radius_m_ = 2.0;
    double rendezvous_direct_min_distance_m_ = 0.6;
    double rendezvous_utility_weight_ = 4.0;
    geometry_msgs::msg::PoseStamped rendezvous_anchor_;
    rclcpp::Time last_rendezvous_anchor_time_{0, 0, RCL_ROS_TIME};
    bool has_rendezvous_anchor_ = false;
    bool rendezvous_replan_requested_ = false;
    bool following_rendezvous_ = false;
    bool rendezvous_arrived_ = false;

    double reserve_exclusion_radius_m_ = 2.0;
    double reserve_ttl_s_ = 6.0;
    double reserve_refresh_period_s_ = 1.0;
    rclcpp::Time last_reservation_pub_{0, 0, RCL_ROS_TIME};

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
    std::unordered_map<std::string, ReservedGoal> other_robot_positions_;
    rclcpp::Time last_robot_position_pub_{0, 0, RCL_ROS_TIME};
    double robot_position_ttl_s_ = 1.0;
    double robot_position_period_s_ = 0.2;
    double other_robot_radius_m_ = 0.40;
    double other_robot_path_lookahead_m_ = 1.5;

    GridPose current_goal_;
    bool has_goal_ = false;          

    rclcpp::Time last_replan_check_{0,0,RCL_ROS_TIME};
    double replan_check_period_s_ = 0.5;
    rclcpp::Time last_plan_attempt_{0,0,RCL_ROS_TIME};
    double plan_retry_period_s_ = 0.75;

    rclcpp::Time goal_commit_start_{0,0,RCL_ROS_TIME};
    double min_commit_time_s_ = 10.0;

    double ig_drop_thresh_ = 0.05;
    double goal_initial_ig_ = 0.0;
    double ig_drop_ratio_ = 0.20;
    double ig_drop_baseline_min_ = 0.20;
    double ig_replan_min_age_s_ = 10.0;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_map_sub_;
    nav_msgs::msg::OccupancyGrid merge_map_;
    nav_msgs::msg::OccupancyGrid local_map_;
    bool has_merge_map_{false};
    bool has_local_map_{false};
    rclcpp::Time last_merge_map_time_;
    rclcpp::Time last_local_map_time_;
    std::string local_map_topic_;
    double merge_map_stale_s_{5.0};
    bool using_local_map_{false};

    void onLocalMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void selectActiveMap();

    rclcpp::Subscription<frontier_ws::msg::DynamicObstacle>::SharedPtr obs_sub_;
    void obsCallback(const frontier_ws::msg::DynamicObstacle::SharedPtr msg);

    int no_plan_count_ = 0;

};

#endif // FRONTIER_MULTI_UWB_HPP_
