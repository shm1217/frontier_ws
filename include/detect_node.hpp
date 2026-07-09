#ifndef DETECT_NODE_H
#define DETECT_NODE_H

#include "frontier_ws/msg/dynamic_obstacle.hpp"
#include "frontier_ws/msg/bbox_img.hpp"
#include "frontier_ws/msg/emb_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "yolo_msgs/msg/detection_array.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix36d = Eigen::Matrix<double, 3, 6>;
using Matrix63d = Eigen::Matrix<double, 6, 3>;

struct obj
{
    double x_ = 0;
    double y_ = 0;
    double z_ = 0;
    int id = -1;
    rclcpp::Time stamp;
};

struct vel
{
    double vx_ = 0;
    double vy_ = 0;
    double vz_ = 0;
};

struct cloud_track
{
    bool has_prev = false;
    obj curr;
    obj prev;
    rclcpp::Time last_seen;
    Vector6d cluster_X = Vector6d::Zero();
    Matrix6d cluster_P = Matrix6d::Identity();
    bool kf_init = false;
};

struct yolo_track
{
    obj curr, prev;
    Eigen::Vector3d boxsize;
    Eigen::Vector3d prev_boxsize;
    double cost = 1e18;
    int miss = 0;
    Vector6d obj_X = Vector6d::Zero();
    Matrix6d obj_P = Matrix6d::Identity();
    sensor_msgs::msg::Image imgs;
    Eigen::VectorXf emb;
    Eigen::VectorXf emb_ref;
    bool has_emb_ref = false;
    bool has_prev = false;
};

struct save_track // appearance cost 포함해서 tracking 결과 확인하기 위해
{
    double cost = 0.0;
    double base_cost = 0.0;
    double app_cost = -1.0;
    int frame_id = -1;
    int det_id = -1;
    int track_id = -1;
    Eigen::VectorXf track_emb;
    Eigen::VectorXf rep_emb;
    Eigen::VectorXf det_emb;
    bool best_pair = false;
    double curr_x = 0.0;
    double curr_size = 0.0; // bbox size
    obj pose;
    Eigen::Vector3d boxsize;
    sensor_msgs::msg::Image det_img;
    sensor_msgs::msg::Image track_img;
    sensor_msgs::msg::Image rep_img;
};

struct prev
{
    Eigen::VectorXf emb;
    sensor_msgs::msg::Image img;
};

struct best
{
    int det_id = -1;
    int track_id = -1;
    double cost = 1e18;
};

struct Represent
{
    int id;
    std::string path;
    Eigen::VectorXf emb;
    sensor_msgs::msg::Image img;
};

struct pub
{
    int id = -1;
    std::unordered_map<int, yolo_track> track;
    std::unordered_map<int, yolo_track> det;
};

class DetectNode : public rclcpp::Node
{
public:
    DetectNode();
    std::unordered_map<int, cloud_track> tracks_;    // id 정해진 후 id별로 KF 적용하기 위함
    std::unordered_map<int, yolo_track> yolo_tracks; // id 정하기 위해 최종 저장 본
    std::unordered_map<int, yolo_track> det_track;
    std::unordered_map<int, yolo_track> kal_track;
    std::vector<std::vector<save_track>> retrack;
    sensor_msgs::msg::Image::ConstSharedPtr imgs;
    image_geometry::PinholeCameraModel cam_model;
    std::vector<Represent> represent;
    std::unordered_map<int, int> track_to_rep;
    std::ofstream outFile;
    pub save_pub;
    int tracking_id = 0;
    int detection_id = 0;
    int frame_id = 0;
    int used_frame = -1;
    int last_calculated_frame_id_ = -1;
    bool img_ready = false;
    bool emb_ready = false;
    bool cam_ready = false;
    bool rep_ready = false;
    bool match_ready = false;

    std::unordered_map<int, prev> prev_track_emb_map_;
    int prev_track_emb_frame_ = -1;

    rclcpp::Time last_process_time_;
    double process_period_sec_ = 0.5; // 2Hz
    bool has_prev = false;
    bool kalman = false;

    double Q_sigma = 0.1;
    double R_sigma = 0.15;
    double P_pos0 = 1.5;
    double P_vel0 = 1.0;

    Eigen::Matrix3d Q = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d Y = Eigen::Vector3d::Zero();

    Vector6d X = Vector6d::Zero();
    Matrix6d P = Matrix6d::Identity();
    Matrix6d A = Matrix6d::Identity();
    Matrix36d C = Matrix36d::Zero();
    Matrix63d G = Matrix63d::Zero();
    Matrix63d K = Matrix63d::Zero();

    void yolo_callback(const yolo_msgs::msg::DetectionArray::SharedPtr msg);
    void emb_callback(const frontier_ws::msg::EmbArray::SharedPtr msg);
    void img_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void camera_callback(const sensor_msgs::msg::CameraInfo::SharedPtr camera_msg);
    void visualize_obj(const obj &obj, const std::array<float, 3> &color, const std::string &ns, int trac_id);
    void calculate_vel(const obj &prev, const obj &curr, vel &v);
    bool project_pixel(const geometry_msgs::msg::Point &meter_pt, Eigen::Vector2d &pixel_pt);
    void calculate_Kalman(Vector6d &X, Matrix6d &P, const obj &prev, const obj &curr, bool vis, obj &res, Eigen::Matrix3d &S);
    void visualize_vel(const obj &obj, const vel &obj_vels, const std::array<float, 3> &color, const std::string &ns, int trac_id);
    void visualize_box(const obj &obj, const Eigen::Vector3d &box, const std::array<float, 3> &color, int id, const std::string &ns);
    void tracking(std::vector<yolo_track> &track);
    double calculate_iou(const Eigen::Vector3d &s, const Eigen::Vector3d &prev_s, const obj &c_kf, const obj &p_kf);
    void pub_bbox_img(int frame_id, std::unordered_map<int, yolo_track> &det_map, std::unordered_map<int, yolo_track> &track_map);
    void calculate_cost();
    double cosine_similarity(const Eigen::VectorXf &a, const Eigen::VectorXf &b);
    std::array<float, 3> get_color(int id);
    sensor_msgs::msg::Image::SharedPtr pngToRosImage(const std::string &path);
    void timer_callback();
    Eigen::VectorXf loadEmbedding(const std::string &path);

private:
    rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr sub_yolo;
    rclcpp::Subscription<frontier_ws::msg::EmbArray>::SharedPtr sub_emb;
    // rclcpp::Subscription<frontier_ws::msg::EmbArray>::SharedPtr sub_rep_emb;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obj_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vel_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr box_pub;
    rclcpp::Publisher<frontier_ws::msg::BboxImg>::SharedPtr img_pub;
    rclcpp::Publisher<frontier_ws::msg::BboxImg>::SharedPtr app_pub;
    rclcpp::Publisher<frontier_ws::msg::DynamicObstacle>::SharedPtr obs_pub;

    std::string robot_id_;
    std::string yolo_detections_topic_;
    std::string embedding_topic_;
    std::string image_topic_;
    std::string camera_info_topic_;
    std::string detect_obj_topic_;
    std::string object_vel_topic_;
    std::string box3d_topic_;
    std::string bbox_image_topic_;
    std::string obs_speed_topic_;
    std::string camera_link_frame_;
    std::string camera_init_frame_;
    std::string obstacle_frame_;
    std::string camera_optical_frame_;
    std::string camera_debug_frame_;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    rclcpp::TimerBase::SharedPtr timer;
    // std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    // std::unique_ptr<tf2_ros::Buffer> tf_buffer;
};

#endif // DETECT_NODE_H
