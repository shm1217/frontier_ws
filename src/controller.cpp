#include "controller.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <utility>
#include <vector>

Controller::Controller(rclcpp::Clock::SharedPtr clock)
: clock_(std::move(clock))
{
    
}

void Controller::obs_update(const frontier_ws::msg::DynamicObstacle::SharedPtr msg) 
{
    auto &obs_state = dynamic_obs[msg->track_id];
    obs_state.x = msg->x;
    obs_state.y = msg->y;
    obs_state.z = 0.0;
    obs_state.vx = msg->vx;
    obs_state.vy = msg->vy;
    obs_state.vz = msg->vz;
    obs_state.last_update = clock_->now();
}

void Controller::goal_update(std::pair<double,double> goal)
{
    goal_x = goal.first;
    goal_y = goal.second;
    has_goal = true;
}

void Controller::pose_update(double x, double y, double yaw)
{
    // geometry_msgs::msg::TransformStamped t;
    // try
    // {
    //     t = tf_buffer.lookupTransform("camera_init", "base_scan", tf2::TimePointZero);
    // }
    // catch (const tf2::TransformException &ex)
    // {
    //     RCLCPP_INFO_ONCE(this->get_logger(), "Could not transform camera_init to base_scan");
    //     return;
    // }

    robot_x = x;
    robot_y = y;
    robot_z = 0.0;
    robot_yaw = yaw;

    has_pose = true;
}

geometry_msgs::msg::Twist Controller::control_cmd_update()
{
    geometry_msgs::msg::Twist cmd;
    remove_stale_obstacles();

    if (!has_goal || !has_pose)
    {
        return cmd;
    }

    double goal_dist = std::hypot(goal_x - robot_x, goal_y - robot_y);
    if (goal_dist < goal_tolerance)
    {
        robot_v = 0.0;
        robot_w = 0.0;
        previous_best_sequence.clear();
        return cmd;
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
        return cmd;
    }

    cmd.linear.x = best_sequence.front().v;
    cmd.angular.z = best_sequence.front().w;
    
    robot_v = best_sequence.front().v;
    robot_w = best_sequence.front().w;
    previous_best_sequence = best_sequence;

    std::vector<Eigen::Vector2d> robot_traj;
    std::unordered_map<int, std::vector<Eigen::Vector2d>> obs_traj;
    predict_trajectories(best_sequence, robot_traj, obs_traj);

    return cmd;

    // visualize_trajectory(robot_traj, obs_traj);
}

void Controller::remove_stale_obstacles()
{
    if (!clock_)
    {
        return;
    }

    const auto now = clock_->now();
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

std::vector<std::vector<ControlInput>> Controller::sample_control_sequences() // 후보 제어 시퀀스를 여러개 만드는 함수
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

std::vector<ControlInput> Controller::make_goal_tracking_sequence() // 목표 방향으로 가는 기본 제어 시퀀스 만듬
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

void Controller::predict_trajectories(
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

double Controller::evaluate_control_sequence(const std::vector<ControlInput> &control_sequence) // cost 계산
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

double Controller::normalize_angle(double angle)
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}
