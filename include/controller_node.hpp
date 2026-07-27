#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "frontier_ws/msg/dynamic_obstacle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <Eigen/Dense>
#include <array>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <random>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <unordered_map>
#include <vector>

struct obs
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double vz = 0.0;
    rclcpp::Time last_update;
};

struct ControlInput
{
    double v = 0.0;
    double w = 0.0;
};

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode();

private:
    double robot_x = 0.0;
    double robot_y = 0.0;
    double robot_z = 0.0;
    double robot_roll = 0.0;
    double robot_pitch = 0.0;
    double robot_yaw = 0.0;
    double robot_v = 0.0;
    double robot_w = 0.0;
    double goal_x = 5.0;
    double goal_y = 0.0;
    bool goal_update = true;
    bool pose_update = false;

    int obs_N = 20;   // 몇 번 예측할건지
    int robot_N = 40; // 몇 번 예측할건지
    double dt = 0.1;  // 몇 초 간격으로 예측할건지
    double robot_radius = 0.2;
    double obstacle_radius = 0.2;
    double max_v = 0.2;
    double max_w = 2.5;
    double goal_tolerance = 0.2;
    double obstacle_timeout = 1.0;
    int num_control_sequences = 500; // 후보 개수
    int control_hold_steps = 3;
    double acc_v_weight = 0.2;
    double acc_w_weight = 0.05;

    std::unordered_map<int, obs> dynamic_obs;
    std::vector<ControlInput> previous_best_sequence;
    std::mt19937 rng;

    void obs_callback(const frontier_ws::msg::DynamicObstacle::SharedPtr msg);
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void timer_tf_callback();
    void timer_control_callback();
    void timer_obs_callback();
    void remove_stale_obstacles();
    std::vector<std::vector<ControlInput>> sample_control_sequences();
    std::vector<ControlInput> make_goal_tracking_sequence();
    void predict_trajectories(const std::vector<ControlInput> &control_sequence, std::vector<Eigen::Vector2d> &robot_traj, std::unordered_map<int, std::vector<Eigen::Vector2d>> &obs_traj);
    double evaluate_control_sequence(const std::vector<ControlInput> &control_sequence);
    double normalize_angle(double angle);
    void visualize_trajectory(const std::vector<Eigen::Vector2d> &robot_traj, const std::unordered_map<int, std::vector<Eigen::Vector2d>> &obs_traj);
    void visualize_obs(const std::unordered_map<int, obs> dy_obs);

    rclcpp::TimerBase::SharedPtr timer_cmd, timer_tf, timer_obs;
    rclcpp::Subscription<frontier_ws::msg::DynamicObstacle>::SharedPtr sub_obs;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_obs;

    std::string robot_id_;
    std::string cmd_vel_topic_;
    std::string trajectory_topic_;
    std::string dy_obs_topic_;
    std::string obs_speed_topic_;
    std::string goal_pose_topic_;
    std::string world_frame;
    std::string base_link_frame;


    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
};

#endif



