#ifndef PLANNER_H
#define PLANNER_H

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

class Controller
{
public:
    explicit Controller(rclcpp::Clock::SharedPtr clock);    
    void goal_update(std::pair<double,double> goal);
    void pose_update(double x, double y, double yaw);
    void obs_update(const frontier_ws::msg::DynamicObstacle::SharedPtr msg);
    geometry_msgs::msg::Twist control_cmd_update();

private:
    double robot_x = 0.0;
    double robot_y = 0.0;
    double robot_z = 0.0;
    double robot_roll = 0.0;
    double robot_pitch = 0.0;
    double robot_yaw = 0.0;
    double robot_v = 0.0;
    double robot_w = 0.0;
    double goal_x = 0.0;
    double goal_y = 0.0;
    bool has_goal = false;
    bool has_pose = false;

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
    rclcpp::Clock::SharedPtr clock_;

    void remove_stale_obstacles();
    std::vector<std::vector<ControlInput>> sample_control_sequences();
    std::vector<ControlInput> make_goal_tracking_sequence();
    void predict_trajectories(const std::vector<ControlInput> &control_sequence, std::vector<Eigen::Vector2d> &robot_traj, std::unordered_map<int, std::vector<Eigen::Vector2d>> &obs_traj);
    double evaluate_control_sequence(const std::vector<ControlInput> &control_sequence);
    double normalize_angle(double angle);
};

#endif
