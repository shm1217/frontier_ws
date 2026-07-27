#include "controller_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <vector>

using namespace std::chrono_literals;

namespace
{
std::string scoped_topic(const std::string &robot_id, const std::string &topic)
{
    std::string clean = topic;
    while (!clean.empty() && clean.front() == '/')
    {
        clean.erase(clean.begin());
    }
    if (robot_id.empty())
    {
        return "/" + clean;
    }
    return "/" + robot_id + "/" + clean;
}
std::string scoped_frame(const std::string &robot_id, const std::string &frame)
{
    if (robot_id.empty())
    {
        return frame;
    }
    return robot_id + "/" + frame;
}
} // namespace

ControllerNode::ControllerNode() : Node("controller_node"), tf_buffer(this->get_clock()), tf_listener(tf_buffer)
{
    robot_id_ = this->declare_parameter<std::string>("robot_id", "robot1");
    cmd_vel_topic_ = this->declare_parameter<std::string>(
        "cmd_vel_topic", scoped_topic(robot_id_, "cmd_vel"));
    trajectory_topic_ = this->declare_parameter<std::string>(
        "trajectory_topic", scoped_topic(robot_id_, "trajectory"));
    dy_obs_topic_ = this->declare_parameter<std::string>(
        "dy_obs_topic", scoped_topic(robot_id_, "dy_obs"));
    obs_speed_topic_ = this->declare_parameter<std::string>(
        "obs_speed_topic", scoped_topic(robot_id_, "obs_speed"));
    goal_pose_topic_ = this->declare_parameter<std::string>(
        "goal_pose_topic", scoped_topic(robot_id_, "goal_pose"));

    world_frame = this->declare_parameter<std::string>(
        "world_frame", "world");
    base_link_frame = this->declare_parameter<std::string>(
        "base_link_frame", scoped_frame(robot_id_, "base_link"));


    timer_tf = this->create_wall_timer(100ms, std::bind(&ControllerNode::timer_tf_callback, this));
    timer_cmd = this->create_wall_timer(100ms, std::bind(&ControllerNode::timer_control_callback, this));
    timer_obs = this->create_wall_timer(100ms, std::bind(&ControllerNode::timer_obs_callback, this));
    rng.seed(std::random_device{}());

    sub_obs = this->create_subscription<frontier_ws::msg::DynamicObstacle>(
        obs_speed_topic_, 10, std::bind(&ControllerNode::obs_callback, this, std::placeholders::_1));
    sub_goal = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_pose_topic_, 10,
        std::bind(&ControllerNode::goal_callback, this, std::placeholders::_1));

    pub_cmd = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    pub_marker = this->create_publisher<visualization_msgs::msg::MarkerArray>(trajectory_topic_, 10);
    pub_obs = this->create_publisher<visualization_msgs::msg::Marker>(dy_obs_topic_, 10);
}

void ControllerNode::timer_obs_callback()
{
    const auto now = this->get_clock()->now();
    const double t = now.seconds();

    auto set_patrol_obstacle = [&](int id, double x0, double y0, double x1, double y1, double speed, double phase)
    {
        const double dx = x1 - x0;
        const double dy = y1 - y0;
        const double length = std::hypot(dx, dy);

        if (length < 1e-6 || speed <= 0.0)
        {
            auto &obs_state = dynamic_obs[id];
            obs_state.x = x0;
            obs_state.y = y0;
            obs_state.z = 0.0;
            obs_state.vx = 0.0;
            obs_state.vy = 0.0;
            obs_state.vz = 0.0;
            obs_state.last_update = now;
            return;
        }

        const double period_distance = 2.0 * length;
        double s = std::fmod(speed * t + phase, period_distance);
        if (s < 0.0)
        {
            s += period_distance;
        }

        double direction = 1.0;
        if (s > length)
        {
            s = period_distance - s;
            direction = -1.0;
        }

        const double ux = dx / length;
        const double uy = dy / length;

        auto &obs_state = dynamic_obs[id];
        obs_state.x = x0 + ux * s;
        obs_state.y = y0 + uy * s;
        obs_state.z = 0.0;
        obs_state.vx = direction * speed * ux;
        obs_state.vy = direction * speed * uy;
        obs_state.vz = 0.0;
        obs_state.last_update = now;
    };

    set_patrol_obstacle(0, 1.8, 1.4, 1.8, -1.4, 0.1, 0.0);
    set_patrol_obstacle(1, -1.4, -1.2, 1.4, -1.2, 0.2, 0.7);
    set_patrol_obstacle(2, 2.2, -1.6, -0.6, 1.2, 0.2, 1.4);
    set_patrol_obstacle(3, -2.4, 1.5, 0.8, 1.5, 0.2, 2.1);
    set_patrol_obstacle(4, -0.7, -2.0, -0.7, 1.6, 0.1, 2.8);

    visualize_obs(dynamic_obs);
}

void ControllerNode::obs_callback(const frontier_ws::msg::DynamicObstacle::SharedPtr msg) 
{
    auto &obs_state = dynamic_obs[msg->track_id];
    obs_state.x = msg->x;
    obs_state.y = msg->y;
    obs_state.z = 0.0;
    obs_state.vx = msg->vx;
    obs_state.vy = msg->vy;
    obs_state.vz = msg->vz;
    obs_state.last_update = this->get_clock()->now();
}

void ControllerNode::goal_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    goal_x = msg->pose.position.x;
    goal_y = msg->pose.position.y;
    goal_update = true;
}

void ControllerNode::timer_tf_callback()
{
    geometry_msgs::msg::TransformStamped t;
    try
    {
        t = tf_buffer.lookupTransform(world_frame, base_link_frame, tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "Could not transform world to base_link");
        return;
    }

    robot_x = t.transform.translation.x;
    robot_y = t.transform.translation.y;
    robot_z = t.transform.translation.z;
    pose_update = true;

    tf2::Quaternion q(
        t.transform.rotation.x,
        t.transform.rotation.y,
        t.transform.rotation.z,
        t.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(robot_roll, robot_pitch, robot_yaw);
}

void ControllerNode::timer_control_callback()
{
    geometry_msgs::msg::Twist cmd;
    remove_stale_obstacles();

    if (!goal_update || !pose_update)
    {
        pub_cmd->publish(cmd);
        return;
    }

    double goal_dist = std::hypot(goal_x - robot_x, goal_y - robot_y);
    if (goal_dist < goal_tolerance)
    {
        robot_v = 0.0;
        robot_w = 0.0;
        previous_best_sequence.clear();
        pub_cmd->publish(cmd);
        return;
    }
    // else if (goal_dist < 1.0) // 목적지 부근
    // {
    //     max_v = 0.1;
    // }
    // else
    // {
    //     max_v = 0.2;
    // }

    double best_cost = std::numeric_limits<double>::infinity();
    std::vector<ControlInput> best_sequence(robot_N);
    auto candidate_sequences = sample_control_sequences();

    for (const auto &control_sequence : candidate_sequences)
    {
        double cost = evaluate_control_sequence(control_sequence);

        if (cost < best_cost)
        {
            best_cost = cost;
            best_sequence = control_sequence;
        }
    }

    if (best_sequence.empty())
    {
        pub_cmd->publish(cmd);
        return;
    }

    cmd.linear.x = best_sequence.front().v;
    cmd.angular.z = best_sequence.front().w;
    pub_cmd->publish(cmd);

    robot_v = best_sequence.front().v;
    robot_w = best_sequence.front().w;
    previous_best_sequence = best_sequence;

    std::vector<Eigen::Vector2d> robot_traj;
    std::unordered_map<int, std::vector<Eigen::Vector2d>> obs_traj;
    predict_trajectories(best_sequence, robot_traj, obs_traj);

    visualize_trajectory(robot_traj, obs_traj);
}

void ControllerNode::remove_stale_obstacles()
{
    const auto now = this->get_clock()->now();
    for (auto it = dynamic_obs.begin(); it != dynamic_obs.end();)
    {
        if ((now - it->second.last_update).seconds() > obstacle_timeout)
        {
            it = dynamic_obs.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

std::vector<std::vector<ControlInput>> ControllerNode::sample_control_sequences() // 후보 제어 시퀀스를 여러개 만드는 함수
{
    std::vector<std::vector<ControlInput>> sequences;
    sequences.reserve(num_control_sequences + 4);

    // sequences.push_back(std::vector<ControlInput>(robot_N, { 0.0, 0.0 }));         // 정지 시퀀스
    sequences.push_back(std::vector<ControlInput>(robot_N, { robot_v, robot_w })); // 현재 속도 유지 시퀀스
    sequences.push_back(make_goal_tracking_sequence());                            // 목표 방향 추종 시퀀스

    if (previous_best_sequence.size() == static_cast<size_t>(robot_N)) // 이전 주기에 선택된 시퀀스
    {
        std::vector<ControlInput> shifted_sequence;
        shifted_sequence.reserve(robot_N);
        for (int i = 1; i < robot_N; ++i)
        {
            shifted_sequence.push_back(previous_best_sequence[i]);
        }
        shifted_sequence.push_back(previous_best_sequence.back());
        sequences.push_back(shifted_sequence);
    }

    auto make_bypass_sequence = [&](double direction, double turn_rate)
    {
        std::vector<ControlInput> control_sequence;
        control_sequence.reserve(robot_N);

        double x_next = robot_x;
        double y_next = robot_y;
        double yaw_next = robot_yaw;

        for (int step = 0; step < robot_N; ++step)
        {
            double v = 0.0;
            double w = 0.0;

            if (step < robot_N * 0.3)
            {
                v = 0.7 * max_v;
                w = direction * turn_rate;
            }
            else if (step < robot_N * 0.6)
            {
                v = 0.85 * max_v;
                w = -direction * 0.35 * turn_rate;
            }
            else
            {
                const double target_yaw = std::atan2(goal_y - y_next, goal_x - x_next);
                const double heading_error = normalize_angle(target_yaw - yaw_next);
                v = 0.75 * max_v;
                w = 1.2 * heading_error;
            }

            v = std::clamp(v, 0.0, max_v);
            w = std::clamp(w, -max_w, max_w);
            control_sequence.push_back({ v, w });

            x_next += v * std::cos(yaw_next) * dt;
            y_next += v * std::sin(yaw_next) * dt;
            yaw_next = normalize_angle(yaw_next + w * dt);
        }

        return control_sequence;
    };

    bool has_bypass_direction = true;
    for (double direction : { -1.0, 1.0 })
    {
        sequences.push_back(make_bypass_sequence(direction, 0.6));
        sequences.push_back(make_bypass_sequence(direction, 0.7));
        sequences.push_back(make_bypass_sequence(direction, 0.8));
        sequences.push_back(make_bypass_sequence(direction, 0.9));
        sequences.push_back(make_bypass_sequence(direction, 1.0));
        sequences.push_back(make_bypass_sequence(direction, 1.1));
        sequences.push_back(make_bypass_sequence(direction, 1.2));
    }

    std::normal_distribution<double> noise_v(0.0, 0.05); // 기존 후보 주변에 노이즈 주기 위함
    std::normal_distribution<double> noise_w(0.0, 0.45);
    std::uniform_real_distribution<double> bypass_turn_rate(0.5, 2.0);

    while (static_cast<int>(sequences.size()) < num_control_sequences)
    {
        std::vector<ControlInput> control_sequence;
        control_sequence.reserve(robot_N);

        double v = 0.0;
        double w = 0.0;
        const int sequence_index = static_cast<int>(sequences.size());
        const double turn_rate = bypass_turn_rate(rng);
        const bool warm_start = previous_best_sequence.size() == static_cast<size_t>(robot_N) &&
                                sequence_index < num_control_sequences / 3;
        const bool bypass_guided = has_bypass_direction &&
                                   sequence_index >= num_control_sequences / 3 &&
                                   sequence_index < 2 * num_control_sequences / 3;

        const double bypass_direction = (sequence_index % 2 == 0) ? -1.0 : 1.0; // 추가

        double x_next = robot_x;
        double y_next = robot_y;
        double yaw_next = robot_yaw;

        for (int step = 0; step < robot_N; ++step)
        {
            if (warm_start)
            {
                const int warm_index = std::min(step + 1, robot_N - 1);
                v = previous_best_sequence[warm_index].v + noise_v(rng);
                w = previous_best_sequence[warm_index].w + noise_w(rng);
            }
            else if (bypass_guided)
            {
                if (step < robot_N * 0.3)
                {
                    v = 0.7 * max_v + noise_v(rng);
                    w = bypass_direction * turn_rate + noise_w(rng);
                }
                else if (step < robot_N * 0.6)
                {
                    v = 0.85 * max_v + noise_v(rng);
                    w = -bypass_direction * 0.35 * turn_rate + noise_w(rng);
                }
                else
                {
                    const double target_yaw = std::atan2(goal_y - y_next, goal_x - x_next);
                    const double heading_error = normalize_angle(target_yaw - yaw_next);
                    v = 0.75 * max_v + noise_v(rng);
                    w = 1.2 * heading_error + noise_w(rng);
                }
            }
            else
            {
                const double target_yaw = std::atan2(goal_y - y_next, goal_x - x_next);
                const double heading_error = normalize_angle(target_yaw - yaw_next);
                v = max_v * std::max(0.2, std::cos(heading_error)) + noise_v(rng);
                w = 1.5 * heading_error + noise_w(rng);
            }
            // else if (step % control_hold_steps == 0)
            // {
            //     v = uniform_v(rng);
            //     w = uniform_w(rng);
            // }

            v = std::clamp(v, 0.0, max_v);
            w = std::clamp(w, -max_w, max_w);
            control_sequence.push_back({ v, w });

            x_next += v * std::cos(yaw_next) * dt;
            y_next += v * std::sin(yaw_next) * dt;
            yaw_next = normalize_angle(yaw_next + w * dt);
        }

        sequences.push_back(control_sequence);
    }

    return sequences;
}

std::vector<ControlInput> ControllerNode::make_goal_tracking_sequence() // 목표 방향으로 가는 기본 제어 시퀀스 만듬
{
    std::vector<ControlInput> control_sequence;
    control_sequence.reserve(robot_N);

    double x_next = robot_x;
    double y_next = robot_y;
    double yaw_next = robot_yaw;

    for (int step = 0; step < robot_N; ++step)
    {
        const double target_yaw = std::atan2(goal_y - y_next, goal_x - x_next);
        const double heading_error = normalize_angle(target_yaw - yaw_next);
        const double v = std::clamp(max_v * std::max(0.2, std::cos(heading_error)), 0.0, max_v);
        const double w = std::clamp(1.5 * heading_error, -max_w, max_w);

        control_sequence.push_back({ v, w });

        x_next += v * std::cos(yaw_next) * dt;
        y_next += v * std::sin(yaw_next) * dt;
        yaw_next = normalize_angle(yaw_next + w * dt);
    }

    return control_sequence;
}

void ControllerNode::predict_trajectories(
    const std::vector<ControlInput> &control_sequence,
    std::vector<Eigen::Vector2d> &robot_traj,
    std::unordered_map<int, std::vector<Eigen::Vector2d>> &obs_traj)
{
    double x_next = robot_x;
    double y_next = robot_y;
    double yaw_next = robot_yaw;

    robot_traj.clear();
    obs_traj.clear();
    robot_traj.reserve(control_sequence.size());

    for (size_t i = 0; i < control_sequence.size(); ++i)
    {
        x_next += control_sequence[i].v * std::cos(yaw_next) * dt;
        y_next += control_sequence[i].v * std::sin(yaw_next) * dt;
        yaw_next = normalize_angle(yaw_next + control_sequence[i].w * dt);

        robot_traj.push_back({ x_next, y_next });

        if (i >= static_cast<size_t>(obs_N))
        {
            continue;
        }

        for (auto &[id, o] : dynamic_obs)
        {
            const double prediction_time = static_cast<double>(i + 1) * dt;
            double ox_next = o.x + o.vx * prediction_time;
            double oy_next = o.y + o.vy * prediction_time;

            auto &traj = obs_traj[id];
            traj.push_back({ ox_next, oy_next });
        }
    }
}

double ControllerNode::evaluate_control_sequence(const std::vector<ControlInput> &control_sequence) // cost 계산
{
    std::vector<Eigen::Vector2d> robot_traj;
    std::unordered_map<int, std::vector<Eigen::Vector2d>> obs_traj;
    predict_trajectories(control_sequence, robot_traj, obs_traj);

    if (robot_traj.empty())
    {
        return std::numeric_limits<double>::infinity();
    }

    double obstacle_cost = 0.0;
    int obstacle_cost_count = 0;
    double min_obstacle_dist = std::numeric_limits<double>::infinity();

    for (const auto &[id, traj] : obs_traj)
    {
        (void)id;
        for (size_t i = 0; i < traj.size() && i < robot_traj.size(); ++i)
        {
            double dist = std::hypot(robot_traj[i].x() - traj[i].x(), robot_traj[i].y() - traj[i].y());
            double safe_dist = robot_radius + obstacle_radius;
            double inflation_dist = safe_dist + 0.5;

            min_obstacle_dist = std::min(min_obstacle_dist, dist);
            ++obstacle_cost_count;

            if (dist < safe_dist)
            {
                obstacle_cost += 100.0 + 100.0 * (safe_dist - dist);
            }
            else if (dist < inflation_dist)
            {
                double ratio = (inflation_dist - dist) / (inflation_dist - safe_dist);
                obstacle_cost += 100.0 * ratio * ratio;
            }
        }
    }

    if (obstacle_cost_count > 0)
    {
        obstacle_cost /= static_cast<double>(obstacle_cost_count);
    }

    if (min_obstacle_dist < robot_radius + obstacle_radius)
    {
        obstacle_cost += 10000.0;
    }

    const auto &last_robot_point = robot_traj.back();
    double x_next = last_robot_point.x();
    double y_next = last_robot_point.y();

    double yaw_next = robot_yaw;
    for (const auto &u : control_sequence)
    {
        yaw_next = normalize_angle(yaw_next + u.w * dt);
    }

    double goal_cost = std::hypot(goal_x - x_next, goal_y - y_next);

    double target_yaw = std::atan2(goal_y - y_next, goal_x - x_next);
    double heading_weight = std::min(1.0, goal_cost / 0.5);
    double heading_cost = heading_weight * std::fabs(normalize_angle(target_yaw - yaw_next));

    double path_goal_cost = 0.0;
    double path_heading_cost = 0.0;
    yaw_next = robot_yaw;
    for (size_t i = 0; i < robot_traj.size() && i < control_sequence.size(); ++i)
    {
        yaw_next = normalize_angle(yaw_next + control_sequence[i].w * dt);

        const double x = robot_traj[i].x();
        const double y = robot_traj[i].y();
        const double dist_to_goal = std::hypot(goal_x - x, goal_y - y);
        path_goal_cost += dist_to_goal;

        const double path_heading_weight = std::min(1.0, dist_to_goal / 0.5);
        if (path_heading_weight > 0.0)
        {
            const double path_target_yaw = std::atan2(goal_y - y, goal_x - x);
            const double path_heading_error = std::fabs(normalize_angle(path_target_yaw - yaw_next));
            path_heading_cost += path_heading_weight * path_heading_error;
        }
    }
    path_goal_cost /= static_cast<double>(robot_traj.size());
    path_heading_cost /= static_cast<double>(robot_traj.size());

    double speed_cost = 0.0;
    double turn_cost = 0.0;       // 각속도 갑자기 커지는 현상 억제
    double rotation_cost = 0.0;   // 제자리 회전 억제
    double smoothness_cost = 0.0; // 속도 부드럽게 변하기 위함
    double prev_v = robot_v;
    double prev_w = robot_w;
    for (const auto &u : control_sequence)
    {
        speed_cost += max_v - u.v;

        if (u.v < 0.05 && std::abs(u.w) > 0.1)
        {
            rotation_cost += 100.0;
        }

        turn_cost += std::fabs(u.w);
        smoothness_cost += acc_v_weight * std::fabs(u.v - prev_v) + acc_w_weight * std::fabs(u.w - prev_w);
        prev_v = u.v;
        prev_w = u.w;
    }
    speed_cost /= static_cast<double>(control_sequence.size());
    turn_cost /= static_cast<double>(control_sequence.size());

    double total_cost =
        1.5 * goal_cost +      // 끝점   1.5
        0.8 * path_goal_cost + // 전체 경로에 대해서
        0.3 * obstacle_cost +  // 0.5
        0.7 * heading_cost +   // 1.3
        0.8 * path_heading_cost +
        0.5 * speed_cost +
        0.1 * turn_cost +     // 0.1
        1.0 * rotation_cost + // 1.5
        1.0 * smoothness_cost;

    return total_cost;
}

void ControllerNode::visualize_trajectory(
    const std::vector<Eigen::Vector2d> &robot_traj,
    const std::unordered_map<int, std::vector<Eigen::Vector2d>> &obs_traj)
{
    visualization_msgs::msg::MarkerArray marker_array;
    const auto stamp = this->get_clock()->now();

    auto make_line_marker = [&](const std::vector<Eigen::Vector2d> &traj, const std::array<float, 3> &color, int id, const std::string &name)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "camera_init";
        marker.header.stamp = stamp;
        marker.ns = name;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.lifetime = rclcpp::Duration::from_seconds(1.0);
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05;
        marker.color.r = color[0];
        marker.color.g = color[1];
        marker.color.b = color[2];
        marker.color.a = 1.0;

        for (const auto &t : traj)
        {
            geometry_msgs::msg::Point p;
            p.x = t.x();
            p.y = t.y();
            p.z = 0.0;
            marker.points.push_back(p);
        }

        return marker;
    };

    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = "camera_init";
    delete_marker.header.stamp = stamp;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    if (!robot_traj.empty())
    {
        marker_array.markers.push_back(make_line_marker(robot_traj, { 0.0, 1.0, 0.0 }, 100, "robot"));
    }

    for (const auto &[id, traj] : obs_traj)
    {
        if (!traj.empty())
        {
            marker_array.markers.push_back(make_line_marker(traj, { 1.0, 0.0, 0.0 }, id, "obs"));
        }
    }

    pub_marker->publish(marker_array);
}

void ControllerNode::visualize_obs(const std::unordered_map<int, obs> dy_obs)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "camera_init";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "dy_obs";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    marker.pose.orientation.w = 1.0;
    marker.scale.x = obstacle_radius;
    marker.scale.y = obstacle_radius;
    marker.scale.z = 0.2;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    for (const auto &[id, t] : dy_obs)
    {
        geometry_msgs::msg::Point p;
        p.x = t.x;
        p.y = t.y;
        p.z = 0.0;
        marker.points.push_back(p);
    }
    pub_obs->publish(marker);
}

double ControllerNode::normalize_angle(double angle)
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}

int main(int argc, const char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}
