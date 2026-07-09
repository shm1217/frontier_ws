#include "detect_node.hpp"
#include "rclcpp/rclcpp.hpp"

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

DetectNode::DetectNode() : Node("detect_node"), tf_buffer(this->get_clock()), tf_listener(tf_buffer)
{
    robot_id_ = this->declare_parameter<std::string>("robot_id", "robot1");

    yolo_detections_topic_ = this->declare_parameter<std::string>(
        "yolo_detections_topic", scoped_topic(robot_id_, "yolo/detections_3d"));
    embedding_topic_ = this->declare_parameter<std::string>(
        "embedding_topic", scoped_topic(robot_id_, "embedding"));

    // 실물
    image_topic_ = this->declare_parameter<std::string>(
        "image_topic", scoped_topic(robot_id_, "camera/camera/color/image_raw"));
    camera_info_topic_ = this->declare_parameter<std::string>(
        "camera_info_topic", scoped_topic(robot_id_, "camera/camera/color/camera_info"));

    // gazebo 환경 시
    // image_topic_ = this->declare_parameter<std::string>(
    //     "image_topic", scoped_topic(robot_id_, "camera/camera/image_raw"));
    // camera_info_topic_ = this->declare_parameter<std::string>(
    //     "camera_info_topic", scoped_topic(robot_id_, "camera/camera/camera_info"));

    detect_obj_topic_ = this->declare_parameter<std::string>(
        "detect_obj_topic", scoped_topic(robot_id_, "detect_obj"));
    object_vel_topic_ = this->declare_parameter<std::string>(
        "object_vel_topic", scoped_topic(robot_id_, "object_vel"));
    box3d_topic_ = this->declare_parameter<std::string>(
        "box3d_topic", scoped_topic(robot_id_, "box3d"));
    bbox_image_topic_ = this->declare_parameter<std::string>(
        "bbox_image_topic", scoped_topic(robot_id_, "bbox_image"));
    obs_speed_topic_ = this->declare_parameter<std::string>(
        "obs_speed_topic", scoped_topic(robot_id_, "obs_speed"));

    camera_link_frame_ = this->declare_parameter<std::string>(
        "camera_link_frame", scoped_frame(robot_id_, "camera_link"));
    camera_init_frame_ = this->declare_parameter<std::string>(
        "camera_init_frame", scoped_frame(robot_id_, "camera_init"));
    obstacle_frame_ = this->declare_parameter<std::string>(
        "obstacle_frame", "world");
    camera_optical_frame_ = this->declare_parameter<std::string>(
        "camera_optical_frame", scoped_frame(robot_id_, "camera_color_optical_frame"));
    camera_debug_frame_ = this->declare_parameter<std::string>(
        "camera_debug_frame", scoped_frame(robot_id_, "camera_frame"));

    sub_yolo = this->create_subscription<yolo_msgs::msg::DetectionArray>(
        yolo_detections_topic_, 10, std::bind(&DetectNode::yolo_callback, this, std::placeholders::_1));
    sub_emb = this->create_subscription<frontier_ws::msg::EmbArray>(
        embedding_topic_, 1000, std::bind(&DetectNode::emb_callback, this, std::placeholders::_1));

    // 실물 depth camera 사용 시
    sub_img = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic_, rclcpp::SensorDataQoS(), std::bind(&DetectNode::img_callback, this, std::placeholders::_1));
    sub_camera = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, rclcpp::SensorDataQoS(), std::bind(&DetectNode::camera_callback, this, std::placeholders::_1));

    // gazebo 환경 시
    // sub_img = this->create_subscription<sensor_msgs::msg::Image>(
    //     image_topic_, rclcpp::SensorDataQoS(), std::bind(&DetectNode::img_callback, this, std::placeholders::_1));
    // sub_camera = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    //     camera_info_topic_, rclcpp::SensorDataQoS(), std::bind(&DetectNode::camera_callback, this, std::placeholders::_1));

    obj_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(detect_obj_topic_, 10);
    vel_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(object_vel_topic_, 10);
    box_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(box3d_topic_, 10);
    img_pub = this->create_publisher<frontier_ws::msg::BboxImg>(bbox_image_topic_, 10);
    obs_pub = this->create_publisher<frontier_ws::msg::DynamicObstacle>(obs_speed_topic_, 10);

    RCLCPP_INFO(
        this->get_logger(),
        "[%s] detect_node topics: yolo=%s image=%s camera_info=%s obs=%s obstacle_frame=%s",
        robot_id_.c_str(),
        yolo_detections_topic_.c_str(),
        image_topic_.c_str(),
        camera_info_topic_.c_str(),
        obs_speed_topic_.c_str(),
        obstacle_frame_.c_str());

    timer = this->create_wall_timer(100ms, std::bind(&DetectNode::timer_callback, this));

    P(0, 0) = P(1, 1) = P(2, 2) = P_pos0 * P_pos0;
    P(3, 3) = P(4, 4) = P(5, 5) = P_vel0 * P_vel0;
    C(0, 0) = C(1, 1) = C(2, 2) = 1.0;
    G(3, 0) = G(4, 1) = G(5, 2) = 1.0;

    Q(0, 0) = Q_sigma * Q_sigma;
    Q(1, 1) = Q_sigma * Q_sigma;
    Q(2, 2) = Q_sigma * Q_sigma;

    R(0, 0) = R_sigma * R_sigma;
    R(1, 1) = R_sigma * R_sigma;
    R(2, 2) = R_sigma * R_sigma;

    std::string best1_path = "/home/seohyeongmi/ros2_ws/src/frontier_ws/appearance_file/female.png";
    std::string best2_path = "/home/seohyeongmi/ros2_ws/src/frontier_ws/appearance_file/male.png";
    represent.push_back({ 0, best1_path, Eigen::VectorXf(), sensor_msgs::msg::Image() });
    represent.push_back({ 1, best2_path, Eigen::VectorXf(), sensor_msgs::msg::Image() });
}

Eigen::VectorXf DetectNode::loadEmbedding(const std::string &path)
{
    std::ifstream file(path);

    std::vector<float> values;
    float v;

    while (file >> v)
    {
        values.push_back(v);
    }

    Eigen::VectorXf emb(values.size());

    for (size_t i = 0; i < values.size(); ++i)
    {
        emb[i] = values[i];
    }

    return emb;
}

void DetectNode::timer_callback()
{
    if (save_pub.id != -1)
        pub_bbox_img(save_pub.id, save_pub.det, save_pub.track);
    if (emb_ready)
        calculate_cost();
}

void DetectNode::camera_callback(const sensor_msgs::msg::CameraInfo::SharedPtr camera_msg)
{
    cam_model.fromCameraInfo(*camera_msg);
    cam_ready = true;
}

void DetectNode::yolo_callback(const yolo_msgs::msg::DetectionArray::SharedPtr msg)
{
    std::vector<yolo_track> tracks;

    // 오래된 tracking_id 삭제
    const auto now = this->now();
    for (auto it = tracks_.begin(); it != tracks_.end();)
    {
        rclcpp::Time last_seen(it->second.last_seen, RCL_ROS_TIME);
        if ((now - last_seen).seconds() > 10.0)
            it = tracks_.erase(it);
        else
            ++it;
    }

    for (size_t i = 0; i < msg->detections.size(); ++i)
    {
        const auto &det = msg->detections[i];
        const auto &pt3d = det.bbox3d.center.position;
        const auto &size = det.bbox3d.size;

        if (det.class_id == 0) // 0:person, 41:cup, 64:mouse, 67:cell phone
        {
            geometry_msgs::msg::PointStamped p_in, p_out;
            p_in.header.frame_id = camera_link_frame_;
            // p_in.header.stamp = msg->header.stamp;
            p_in.header.stamp.sec = 0;
            p_in.header.stamp.nanosec = 0;
            p_in.point = pt3d;

            p_out = tf_buffer.transform(p_in, obstacle_frame_);
            // try
            // {
            //     p_out = tf_buffer.transform(p_in, "camera_init");
            // }
            // catch (const tf2::TransformException &ex)
            // {
            //     RCLCPP_WARN_THROTTLE(
            //         this->get_logger(), *this->get_clock(), 1000,
            //         "Could not transform obstacle from camera_link to camera_init: %s", ex.what());
            //     continue;
            // }

            yolo_track t;
            t.curr.x_ = p_out.point.x;
            t.curr.y_ = p_out.point.y;
            t.curr.z_ = p_out.point.z;
            t.curr.stamp = rclcpp::Time(msg->header.stamp, RCL_ROS_TIME);

            t.boxsize.x() = size.x;
            t.boxsize.y() = size.y;
            t.boxsize.z() = size.z;

            tracks.push_back(t);
        }
    }
    tracking(tracks);
}

void DetectNode::img_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    imgs = msg;
    img_ready = true; // 필요성, 어차피 yolo가 된다는건 img 들어왔다는 거 아닌가
}

void DetectNode::emb_callback(const frontier_ws::msg::EmbArray::SharedPtr msg)
{
    if (!rep_ready)
    {
        if (msg->frame_id == -1)
        {
            for (size_t i = 0; i < msg->track_ids.size(); ++i)
            {
                Eigen::Map<const Eigen::VectorXf> emb(
                    msg->track_data.data() + i * msg->track_dim,
                    msg->track_dim);

                int rep_id = msg->track_ids[i];

                for (auto &rep : represent)
                {
                    if (rep.id == rep_id)
                    {
                        rep.emb = emb;
                        RCLCPP_INFO(this->get_logger(), "rep emb 들어옴 id=%d", rep_id);
                        break;
                    }
                }
            }
            rep_ready = true;
            return;
        }
    }

    // if (det_track.empty())
    //     return;

    for (auto &r : retrack) // 프레임 당
    {
        for (auto &f : r) // f = save_track
        {
            if (msg->frame_id != f.frame_id)
                continue;
            for (size_t i = 0; i < msg->det_ids.size(); ++i)
            {
                Eigen::Map<const Eigen::VectorXf> emb(msg->det_data.data() + i * msg->det_dim, msg->det_dim);
                if (f.det_id == msg->det_ids[i])
                {
                    f.det_emb = emb;
                    // RCLCPP_INFO(this->get_logger(), "det emb 존재 frame: %d", msg->frame_id);
                }
            }
            for (size_t i = 0; i < msg->track_ids.size(); ++i)
            {
                Eigen::Map<const Eigen::VectorXf> emb(msg->track_data.data() + i * msg->track_dim, msg->track_dim);
                if (f.track_id == msg->track_ids[i])
                {
                    f.track_emb = emb;
                }
            }
        }
    }

    emb_ready = true;
}

bool DetectNode::project_pixel(const geometry_msgs::msg::Point &meter_pt, Eigen::Vector2d &pixel_pt)
{
    if (!cam_ready)
        return false;
    geometry_msgs::msg::PointStamped p_in, p_out;
    p_in.header.frame_id = obstacle_frame_;
    // p_in.header.stamp = imgs->header.stamp;
    p_in.header.stamp.sec = 0;
    p_in.header.stamp.nanosec = 0;
    p_in.point = meter_pt;

    tf_buffer.transform(p_in, p_out, camera_optical_frame_);
    // try
    // {
    //     tf_buffer.transform(p_in, p_out, "camera_color_optical_frame");
    // }
    // catch (const tf2::TransformException &ex)
    // {
    //     return false;
    // }

    const double X = p_out.point.x;
    const double Y = p_out.point.y;
    const double Z = p_out.point.z;

    if (Z <= 0.1)
        return false;

    cv::Point2d uv = cam_model.project3dToPixel(cv::Point3d(X, Y, Z));
    pixel_pt.x() = uv.x;
    pixel_pt.y() = uv.y;
    return true;
}

void DetectNode::pub_bbox_img(int frame_id, std::unordered_map<int, yolo_track> &det_map, std::unordered_map<int, yolo_track> &track_map)
{
    if (!imgs)
    {
        RCLCPP_WARN(this->get_logger(), "imgs is null");
        return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(imgs, "bgr8");
    }
    catch (const cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat img = cv_ptr->image;
    frontier_ws::msg::BboxImg boximg;
    boximg.frame_id = frame_id;

    auto crop_one = [&](yolo_track &obj_track, sensor_msgs::msg::Image &out_img) -> bool
    {
        geometry_msgs::msg::Point top_left;
        top_left.x = obj_track.curr.x_;
        top_left.y = obj_track.curr.y_ + obj_track.boxsize.y() / 2.0;
        top_left.z = obj_track.curr.z_ + obj_track.boxsize.z() / 2.0;

        geometry_msgs::msg::Point top_right;
        top_right.x = obj_track.curr.x_;
        top_right.y = obj_track.curr.y_ - obj_track.boxsize.y() / 2.0;
        top_right.z = obj_track.curr.z_ + obj_track.boxsize.z() / 2.0;

        geometry_msgs::msg::Point bottom_left;
        bottom_left.x = obj_track.curr.x_;
        bottom_left.y = obj_track.curr.y_ + obj_track.boxsize.y() / 2.0;
        bottom_left.z = obj_track.curr.z_ - obj_track.boxsize.z() / 2.0;

        geometry_msgs::msg::Point bottom_right;
        bottom_right.x = obj_track.curr.x_;
        bottom_right.y = obj_track.curr.y_ - obj_track.boxsize.y() / 2.0;
        bottom_right.z = obj_track.curr.z_ - obj_track.boxsize.z() / 2.0;

        Eigen::Vector2d pixel_tl, pixel_tr, pixel_bl, pixel_br;
        if (!(project_pixel(top_left, pixel_tl) &&
              project_pixel(top_right, pixel_tr) &&
              project_pixel(bottom_left, pixel_bl) &&
              project_pixel(bottom_right, pixel_br)))
        {
            return false;
        }

        int x_tl = static_cast<int>(std::round(pixel_tl.x()));
        int y_tl = static_cast<int>(std::round(pixel_tl.y()));

        int x_tr = static_cast<int>(std::round(pixel_tr.x()));
        int y_tr = static_cast<int>(std::round(pixel_tr.y()));

        int x_bl = static_cast<int>(std::round(pixel_bl.x()));
        int y_bl = static_cast<int>(std::round(pixel_bl.y()));

        int x_br = static_cast<int>(std::round(pixel_br.x()));
        int y_br = static_cast<int>(std::round(pixel_br.y()));

        int left = std::min({ x_tl, x_tr, x_bl, x_br });
        int right = std::max({ x_tl, x_tr, x_bl, x_br });
        int top = std::min({ y_tl, y_tr, y_bl, y_br });
        int bottom = std::max({ y_tl, y_tr, y_bl, y_br });

        left = std::max(0, left);
        top = std::max(0, top);
        right = std::min(img.cols, right);
        bottom = std::min(img.rows, bottom);

        int width = right - left;
        int height = bottom - top;

        if (width <= 0 || height <= 0)
        {
            // RCLCPP_WARN(this->get_logger(),
            //             "Invalid ROI: left=%d top=%d right=%d bottom=%d img=(%d,%d)",
            //             left, top, right, bottom, img.cols, img.rows);
            return false;
        }

        cv::Rect roi(left, top, width, height);
        cv::Mat crop = img(roi).clone();

        out_img = *cv_bridge::CvImage(imgs->header, "bgr8", crop).toImageMsg();
        return true;
    };

    // det crop들
    for (auto &[det_id, det_obj] : det_map)
    {
        sensor_msgs::msg::Image crop_msg;
        if (!crop_one(det_obj, crop_msg))
            continue;

        boximg.det_ids.push_back(det_id);
        boximg.det_crops.push_back(crop_msg);
        for (auto &re : retrack)
        {
            for (auto &f : re)
            {
                if (f.frame_id != frame_id)
                    continue;
                if (f.det_id != det_id)
                    continue;
                f.det_img = crop_msg;
            }
        }
    }

    // track crop들
    for (auto &[track_id, track_obj] : track_map)
    {
        sensor_msgs::msg::Image crop_msg;
        if (!crop_one(track_obj, crop_msg))
            continue;

        boximg.track_ids.push_back(track_id);
        boximg.track_crops.push_back(crop_msg);
        for (auto &re : retrack)
        {
            for (auto &f : re)
            {
                if (f.frame_id != frame_id)
                    continue;
                if (f.track_id != track_id)
                    continue;
                f.track_img = crop_msg;
            }
        }
    }

    if (boximg.det_ids.empty() && boximg.track_ids.empty())
    {
        RCLCPP_WARN(this->get_logger(), "No valid det/track crops to publish");
        return;
    }

    if (!rep_ready)
    {
        frontier_ws::msg::BboxImg repimg;
        repimg.frame_id = -1;
        for (auto &r : represent)
        {
            repimg.track_ids.push_back(r.id);
            r.img = *pngToRosImage(r.path);
            repimg.track_crops.push_back(r.img);
        }
        img_pub->publish(repimg);
        // return;
    }

    // RCLCPP_INFO(this->get_logger(), "pub frame id: %d", frame_id);
    img_pub->publish(boximg);
}

sensor_msgs::msg::Image::SharedPtr DetectNode::pngToRosImage(const std::string &path)
{
    cv::Mat img = cv::imread(path, cv::IMREAD_COLOR); // BGR

    if (img.empty())
    {
        throw std::runtime_error("Failed to load image: " + path);
    }

    std_msgs::msg::Header header;
    header.stamp = this->get_clock()->now();
    header.frame_id = camera_debug_frame_;

    auto msg = cv_bridge::CvImage(
                   header,
                   "bgr8",
                   img)
                   .toImageMsg();

    return msg;
}

void DetectNode::tracking(std::vector<yolo_track> &track) // track을 id 포함된 det_track으로
{
    det_track.clear();
    std::vector<save_track> cost_track;
    frame_id++;

    std::unordered_set<int> matched_id;
    if (track.empty())
        return;
    if (yolo_tracks.empty())
    {
        for (auto &c : track) // 트랙 만들기
        {
            int c_id = tracking_id++;
            auto &t = yolo_tracks[c_id];
            t = c; // curr, size, stamp
            t.prev = t.curr;
            t.prev_boxsize = t.boxsize;
            t.obj_P.diagonal() << P_pos0 * P_pos0, P_pos0 * P_pos0, P_pos0 * P_pos0, P_vel0 * P_vel0, P_vel0 * P_vel0, P_vel0 * P_vel0;
            t.obj_X << t.curr.x_, t.curr.y_, t.curr.z_, 0.0, 0.0, 0.0;
        }
        save_pub.id = frame_id;
        save_pub.det.clear();
        save_pub.track = yolo_tracks;
        return;
    }

    for (auto &[id_, k] : yolo_tracks)
        k.cost = 1e18;

    std::vector<best> best_;

    for (auto &d : track)
    {
        int de_id = detection_id++;
        det_track[de_id] = d;
    }

    for (auto &[d_id, o] : det_track)
    {
        if (!img_ready)
            continue;

        for (auto &[id, t] : yolo_tracks)
        {
            obj c, p;
            obj kalman;
            Eigen::Matrix3d s;
            c = t.curr;
            p = t.prev;
            Vector6d pred_X = t.obj_X;
            Matrix6d pred_P = t.obj_P;
            calculate_Kalman(pred_X, pred_P, p, c, false, kalman, s);

            // Mahalanobis dis
            Eigen::Vector3d det_pose, track_pose;
            det_pose << o.curr.x_, o.curr.y_, o.curr.z_;
            track_pose << kalman.x_, kalman.y_, kalman.z_;
            Eigen::Matrix3d S_safe = s + 1e-6 * Eigen::Matrix3d::Identity();
            double maha_dis = (det_pose - track_pose).dot(S_safe.ldlt().solve(det_pose - track_pose));
            maha_dis = std::max(0.0, maha_dis);
            double dis_n = 1.0 - std::exp(-0.5 * maha_dis);

            obj cc, pp;
            cc = o.curr;
            pp.x_ = t.obj_X[0];
            pp.y_ = t.obj_X[1];
            pp.z_ = t.obj_X[2];
            double iou = calculate_iou(o.boxsize, t.boxsize, cc, pp); // 많이 겹쳐있으면 1에 가까워짐 0~1
            double area_p = t.boxsize.x() * t.boxsize.y() * t.boxsize.z();
            double area_c = o.boxsize.x() * o.boxsize.y() * o.boxsize.z();
            double size_ratio = std::abs(area_p - area_c) / std::max(area_p, area_c); // 0~1
            double direct = 0.0;
            if (t.curr.stamp.get_clock_type() != t.prev.stamp.get_clock_type())
            {
                RCLCPP_WARN(this->get_logger(), "tracking - Time source mismatch: curr=%d prev=%d",
                            (int)t.curr.stamp.get_clock_type(),
                            (int)t.prev.stamp.get_clock_type());
                continue;
            }
            if (t.curr.stamp != t.prev.stamp)
            {
                obj o_curr = o.curr;
                vel o_vel;
                calculate_vel(p, o_curr, o_vel);
                Eigen::Vector3d vel_kf, vel_o;
                vel_kf.x() = t.obj_X[3]; // track의 예측 속도
                vel_kf.y() = t.obj_X[4];
                vel_kf.z() = t.obj_X[5];
                vel_o.x() = o_vel.vx_; // detection 예측 속도 (prev: track의 prev, curr: detection 현재 위치)
                vel_o.y() = o_vel.vy_;
                vel_o.z() = o_vel.vz_;

                if (vel_kf.norm() > 1e-3 && vel_o.norm() > 1e-3)
                {
                    direct = vel_kf.dot(vel_o) / (vel_kf.norm() * vel_o.norm());
                    direct = std::clamp(direct, -1.0, 1.0); // 부동소수점 주의
                    direct = 0.5 * (1.0 - direct);          // cos(-@)=cos(@), 0~180까지만 가정함
                }
            }
            if (!std::isfinite(direct))
                direct = 0.0;

            double cost = dis_n + (1 - iou) + size_ratio * direct; // cost 스케일 맞추기
            // if (!std::isfinite(cost))
            // {
            //     RCLCPP_WARN(this->get_logger(),
            //                 "NaN cost distance=%.3f size=%.3f iou=%.3f dir=%.3f total=%.3f",
            //                 dis_n, size_ratio, 1 - iou, direct, cost);
            // }

            best_.push_back({ d_id, id, cost });

            save_track save;
            save.base_cost = cost;
            save.frame_id = frame_id;
            save.det_id = d_id;
            save.track_id = id;
            save.curr_x = o.curr.x_;
            save.pose = o.curr;
            save.boxsize = o.boxsize;
            save.curr_size = area_c;
            cost_track.emplace_back(save);
            // RCLCPP_INFO(this->get_logger(), "det_id: %d, track_id: %d, cost: %.3f", d_id, id, cost);
        }
    }

    std::sort(best_.begin(), best_.end(), [](auto &a, auto &b)
              { return a.cost < b.cost; }); // 오름차순 정렬
    std::unordered_set<int> used_det;
    std::unordered_set<int> used_track;

    for (auto &b : best_)
    {
        if (used_det.count(b.det_id) || used_track.count(b.track_id))
            continue;
        used_det.insert(b.det_id);
        used_track.insert(b.track_id);

        auto dit = det_track.find(b.det_id);
        if (dit == det_track.end())
            continue;

        yolo_tracks[b.track_id].prev = yolo_tracks[b.track_id].curr;
        yolo_tracks[b.track_id].curr = dit->second.curr;
        yolo_tracks[b.track_id].prev_boxsize = yolo_tracks[b.track_id].boxsize;
        yolo_tracks[b.track_id].boxsize = dit->second.boxsize;
        // yolo_tracks[b.track_id].emb = dit->second.emb; // 과연 이거 전에 det_track.emb 들어와서 이게 잘 될지는 의문임
        matched_id.insert(b.track_id);

        for (auto &re : cost_track)
        {
            if (re.det_id != b.det_id || re.track_id != b.track_id)
                continue;
            re.best_pair = true;
        }
    }

    // bbox 확 커지는 거 보정 용: retrack에 저장하기 전에 yolo_tracks와 cost_track을 같이 보정
    if (retrack.size() >= 4)
    {
        std::unordered_map<int, std::vector<double>> x_history;
        std::unordered_map<int, std::vector<double>> size_history;

        for (auto &re : retrack)
        {
            for (auto &f : re)
            {
                if (!f.best_pair)
                    continue;
                x_history[f.track_id].push_back(f.curr_x);
                size_history[f.track_id].push_back(f.curr_size);
            }
        }

        auto getMedian = [](std::vector<double> v)
        {
            if (v.empty())
                return 0.0;
            std::sort(v.begin(), v.end());
            size_t n = v.size();
            if (n % 2 == 0)
                return (v[n / 2 - 1] + v[n / 2]) / 2.0;
            return v[n / 2];
        };

        for (auto &f : cost_track)
        {
            if (!f.best_pair)
                continue;

            auto x_it = x_history.find(f.track_id);
            auto size_it = size_history.find(f.track_id);
            if (x_it == x_history.end() || size_it == size_history.end())
                continue;

            auto &xs = x_it->second;
            auto &sizes = size_it->second;
            if (xs.size() < 4 || sizes.size() < 4)
                continue;

            double median_x = getMedian(xs);
            double median_size = getMedian(sizes);
            double prev_x = xs.back();
            double prev_size = sizes.back();
            double last_x = f.curr_x;
            double last_size = f.curr_size;

            bool jump_from_prev = std::abs(last_x - prev_x) > 1.5;
            bool far_from_history = std::abs(last_x - median_x) > 1.5;

            double size_ratio_prev = last_size / std::max(prev_size, 1e-6);
            double size_ratio_hist = last_size / std::max(median_size, 1e-6);
            bool size_jump_prev = (size_ratio_prev > 1.5 || size_ratio_prev < 0.5);
            bool size_far_hist = (size_ratio_hist > 1.5 || size_ratio_hist < 0.5);

            if ((jump_from_prev || far_from_history) &&
                (size_jump_prev || size_far_hist))
            {
                auto track_it = yolo_tracks.find(f.track_id);
                if (track_it == yolo_tracks.end())
                    continue;

                track_it->second.curr = track_it->second.prev;
                track_it->second.boxsize = track_it->second.prev_boxsize;

                f.pose = track_it->second.curr;
                f.boxsize = track_it->second.boxsize;
                f.curr_x = f.pose.x_;
                f.curr_size = f.boxsize.x() * f.boxsize.y() * f.boxsize.z();
            }
        }
    }

    retrack.emplace_back(cost_track);
    if (retrack.size() > 100)
        retrack.erase(retrack.begin(), retrack.end() - 100);

    save_pub.id = frame_id;
    save_pub.det = det_track;
    save_pub.track = yolo_tracks;

    // 새로 생긴 객체일 경우
    for (auto &[det_id, det] : det_track)
    {
        if (used_det.count(det_id))
            continue;

        int new_id = tracking_id++;

        yolo_track new_ = det;
        new_.obj_X << det.curr.x_, det.curr.y_, det.curr.z_, 0.0, 0.0, 0.0;
        new_.obj_P.diagonal() << P_pos0 * P_pos0, P_pos0 * P_pos0, P_pos0 * P_pos0, P_vel0 * P_vel0, P_vel0 * P_vel0, P_vel0 * P_vel0;
        new_.prev = new_.curr;
        new_.prev_boxsize = new_.boxsize;
        new_.emb = det.emb;

        yolo_tracks.emplace(new_id, new_);
        matched_id.insert(new_id);

        match_ready = false; // 대표 이미지랑 다시 매칭
        track_to_rep.clear();
        prev_track_emb_map_.clear();
    }

    // yolo_tracks[best_idx] 이외는 miss gap +1
    for (auto it = yolo_tracks.begin(); it != yolo_tracks.end();)
    {
        rclcpp::Time curr_stamp(it->second.curr.stamp, RCL_ROS_TIME);
        int id_ = it->first;
        if (matched_id.count(id_))
            it->second.miss = 0;
        else
            it->second.miss++;
        if (it->second.miss > 30 || (this->now() - curr_stamp).seconds() > 5)
        {
            match_ready = false;
            track_to_rep.clear();
            prev_track_emb_map_.clear();
            it = yolo_tracks.erase(it);
        }

        else
            ++it;
    }

    // 트래킹 된 애들 KF 적용
    for (auto &[id, t] : yolo_tracks)
    {
        auto &tr = tracks_[id];
        tr.last_seen = this->now();
        obj curr_o;
        curr_o = t.curr;
        curr_o.id = id;

        if (tr.has_prev)
        {
            tr.prev = tr.curr;
        }

        tr.curr = curr_o;
        tr.has_prev = true;

        if (!tr.kf_init)
        {
            tr.cluster_P.diagonal() << P_pos0 * P_pos0, P_pos0 * P_pos0, P_pos0 * P_pos0, P_vel0 * P_vel0, P_vel0 * P_vel0, P_vel0 * P_vel0;
            tr.cluster_X << tr.curr.x_, tr.curr.y_, tr.curr.z_, 0.0, 0.0, 0.0;
            tr.kf_init = true;
            continue;
        }
        if (!tr.has_prev)
        {
            continue;
        }
        // 외형 cost 안 쓸때
        // obj res;
        // Eigen::Matrix3d s;
        // calculate_Kalman(tr.cluster_X, tr.cluster_P, tr.prev, tr.curr, true, res, s); // res !=0 일 때만 visualize_box에서 res 사용하도록 하기
        // if (!kalman)
        //     continue;
        // auto color = get_color(id);
        // visualize_box(res, t.boxsize, color, id, "obj_box");
        // frontier_ws::msg::DynamicObstacle obs;
        // obs.track_id = id;
        // obs.x = res.x_;
        // obs.y = res.y_;
        // obs.z = res.z_;
        // obs.vx = tr.cluster_X[3];
        // obs.vy = tr.cluster_X[4];
        // obs.vz = tr.cluster_X[5];
        // obs_pub->publish(obs);
    }
}

void DetectNode::calculate_cost()
{
    if (retrack.empty() || retrack.size() < 20)
        return;

    if (!rep_ready)
        return;

    const int frame_num = 20; // 사용할 embedding 있는 프레임 개수
    int used_emb_count = 0;
    int used_match_count = 0;

    // for (auto &re : retrack)
    // {
    //     for (auto &t : re)
    //     {
    //         if (t.det_emb.size() == 0)
    //             continue;
    //         RCLCPP_INFO(this->get_logger(), "frame_id: %d, det_id: %d", t.frame_id, t.det_id);
    //     }
    // }

    // 대표 이미지 track이랑 매칭
    std::unordered_map<int, std::unordered_map<int, float>> sim_sum; // track_id -> (rep_id -> similarity 누적)
    std::unordered_map<int, int> count_map;                          // track_id -> count
    if (!match_ready)
    {
        int valid_track_emb = 0;
        used_emb_count = 0;

        for (int i = (int)retrack.size() - 1; i >= 0 && used_emb_count < frame_num; --i)
        {
            bool frame_has_emb = false;

            for (auto &t : retrack[i])
            {
                if (t.track_emb.size() == 0)
                    continue;

                valid_track_emb++;
                frame_has_emb = true;

                count_map[t.track_id]++;

                Eigen::VectorXf t_norm = t.track_emb.normalized();

                for (auto &r : represent)
                {
                    if (r.emb.size() == 0)
                        continue;

                    Eigen::VectorXf r_norm = r.emb.normalized();
                    float sim = t_norm.dot(r_norm);

                    sim_sum[t.track_id][r.id] += sim;
                    RCLCPP_INFO(this->get_logger(), "det: %d, rep: %d, sim: %f", t.track_id, r.id, sim);
                }
            }
            if (frame_has_emb)
                used_emb_count++;
        }

        if (valid_track_emb == 0 || sim_sum.empty())
        {
            RCLCPP_WARN(this->get_logger(),
                        "track_emb 아직 없음. match_ready 유지 false");
            return;
        }

        // for (auto &[track_id, rep_map] : sim_sum)
        // {
        //     int best_rep_id = -1;
        //     float best_avg = -1.0f;

        //     int cnt = count_map[track_id];
        //     if (cnt == 0)
        //         continue;

        //     for (auto &[rep_id, sum_sim] : rep_map)
        //     {
        //         float avg_sim = sum_sim / cnt;

        //         if (avg_sim > best_avg)
        //         {
        //             best_avg = avg_sim;
        //             best_rep_id = rep_id;
        //         }
        //     }

        //     if (best_rep_id != -1)
        //     {
        //         track_to_rep[track_id] = best_rep_id;

        //         RCLCPP_INFO(this->get_logger(),
        //                     "[AVG] track %d -> rep %d (avg sim: %.3f, count: %d)",
        //                     track_id, best_rep_id, best_avg, cnt);
        //     }
        // }

        ///// 일대일 대응 되도록
        std::vector<std::tuple<float, int, int, int>> candidates;

        for (auto &[track_id, rep_map] : sim_sum)
        {
            int cnt = count_map[track_id];
            if (cnt == 0)
                continue;

            for (auto &[rep_id, sum_sim] : rep_map)
            {
                float avg_sim = sum_sim / cnt;

                candidates.push_back(
                    std::make_tuple(avg_sim, track_id, rep_id, cnt));

                RCLCPP_INFO(this->get_logger(),
                            "[SIM] track %d - rep %d : %.3f",
                            track_id, rep_id, avg_sim);
            }
        }

        std::sort(candidates.begin(), candidates.end(),
                  [](const auto &a, const auto &b)
                  {
                      return std::get<0>(a) > std::get<0>(b);
                  });

        std::unordered_set<int> used_tracks;
        std::unordered_set<int> used_reps;

        track_to_rep.clear();

        for (const auto &c : candidates)
        {
            float avg_sim = std::get<0>(c);
            int track_id = std::get<1>(c);
            int rep_id = std::get<2>(c);
            int cnt = std::get<3>(c);

            if (used_tracks.count(track_id) > 0)
                continue;

            if (used_reps.count(rep_id) > 0)
                continue;

            track_to_rep[track_id] = rep_id;

            used_tracks.insert(track_id);
            used_reps.insert(rep_id);

            RCLCPP_INFO(this->get_logger(),
                        "[MATCH 1:1] track %d -> rep %d (avg sim: %.3f, count: %d)",
                        track_id, rep_id, avg_sim, cnt);
        }
        /////

        if (track_to_rep.size() < yolo_tracks.size())
        {
            RCLCPP_WARN(this->get_logger(),
                        "matched tracks not enough: %zu / %zu",
                        track_to_rep.size(), yolo_tracks.size());
            return;
        }
        if (track_to_rep.empty())
        {
            RCLCPP_WARN(this->get_logger(), "track_to_rep 비어있음. match_ready 유지 false");
            return;
        }

        match_ready = true;
    }

    std::vector<save_track> r;

    for (auto &re : retrack)
    {
        if (re.empty())
            continue;

        int frame_id = re.front().frame_id;

        if (frame_id <= last_calculated_frame_id_)
            continue;

        int has_emb = 0;

        for (auto &f : re)
        {
            auto it = track_to_rep.find(f.track_id);
            if (it == track_to_rep.end())
                continue;

            for (auto &rep : represent)
            {
                if (rep.id == it->second)
                {
                    f.rep_emb = rep.emb;
                    f.rep_img = rep.img;
                    break;
                }
            }

            if (f.rep_emb.size() > 0 && f.det_emb.size() > 0)
                has_emb++;
        }

        if (has_emb >= 1)
        {
            r = re;
            last_calculated_frame_id_ = frame_id;
            break;
        }
    }

    if (r.empty())
        return;

    int app_frame_id = r.front().frame_id;

    if (app_frame_id == used_frame)
    {
        return;
    }

    used_frame = app_frame_id;

    std::vector<std::pair<int, int>> best_pairs;

    // 0. 이전 프레임 track embedding map이 없으면 아직 비교 불가
    if (prev_track_emb_map_.empty())
    {
        RCLCPP_INFO(this->get_logger(),
                    "prev_track_emb_map_ is empty, skip appearance compare");

        // 이번 프레임의 track_emb를 다음 프레임 reference로 저장
        for (auto &f : r)
        {
            if (f.rep_emb.size() == 0 || f.rep_img.data.empty())
                continue;

            prev_track_emb_map_[f.track_id].emb = f.rep_emb;
            prev_track_emb_map_[f.track_id].img = f.rep_img;
            prev_track_emb_frame_ = f.frame_id;
        }
        return;
    }
    // 1. 이전 track emb와 현재 det emb로 appearance cost 계산
    bool has_valid_pair = false;

    for (auto &f : r)
    {
        if (f.det_emb.size() == 0)
            continue;

        auto prev_it = prev_track_emb_map_.find(f.track_id);
        if (prev_it == prev_track_emb_map_.end())
            continue;

        const Eigen::VectorXf &prev_track_emb = prev_it->second.emb;
        if (prev_track_emb.size() == 0)
            continue;
        if (f.det_emb.size() != prev_track_emb.size())
            continue;

        double appear = cosine_similarity(f.det_emb, prev_track_emb);
        double total_cost = f.base_cost + (1.0 - appear);

        f.app_cost = appear;
        f.cost = total_cost;

        has_valid_pair = true;
    }

    if (!has_valid_pair)
    {
        RCLCPP_INFO(this->get_logger(),
                    "No valid appearance pair in frame=%d", r.front().frame_id);

        // 그래도 다음 프레임을 위해 이번 frame rep_emb 저장
        prev_track_emb_map_.clear();
        for (auto &f : r)
        {
            if (f.rep_emb.size() == 0 || f.rep_img.data.empty())
                continue;

            prev_track_emb_map_[f.track_id].emb = f.rep_emb;
            prev_track_emb_map_[f.track_id].img = f.rep_img;
            prev_track_emb_frame_ = f.frame_id;
        }
        return;
    }

    // 2. appearance 포함 후 best pair 다시 계산
    std::sort(r.begin(), r.end(), [](const auto &a, const auto &b)
              { return a.cost < b.cost; });

    std::unordered_set<int> used_det;
    std::unordered_set<int> used_track;

    for (auto &f : r)
    {
        // appearance 계산이 실제로 된 pair만 사용
        if (f.app_cost < 0.0)
            continue;

        if (used_det.count(f.det_id) || used_track.count(f.track_id))
            continue;

        used_det.insert(f.det_id);
        used_track.insert(f.track_id);
        best_pairs.emplace_back(f.det_id, f.track_id);
    }

    // 그냥 여기서 best_pair에 맞게 시각화하기
    for (auto &[best_det_id, best_track_id] : best_pairs)
    {
        for (auto &f : r)
        {
            if (f.det_id != best_det_id || f.track_id != best_track_id)
                continue;

            obj res;
            Eigen::Matrix3d s;
            auto &k_track = kal_track[f.track_id];
            if (!k_track.has_prev)
            {
                k_track.obj_X << f.pose.x_, f.pose.y_, f.pose.z_, 0.0, 0.0, 0.0;
                k_track.obj_P = Matrix6d::Identity();
                k_track.obj_P.diagonal() << P_pos0 * P_pos0, P_pos0 * P_pos0, P_pos0 * P_pos0, P_vel0 * P_vel0, P_vel0 * P_vel0, P_vel0 * P_vel0;
                auto color = get_color(best_track_id);
                visualize_box(f.pose, f.boxsize, color, best_track_id, "obj_box");
                k_track.prev = f.pose;
                k_track.has_prev = true;
                continue;
            }
            calculate_Kalman(k_track.obj_X, k_track.obj_P, k_track.prev, f.pose, true, res, s); // 이전  frame id에서 가져오기??
            if (!kalman)
                continue;
            auto color = get_color(best_track_id);
            visualize_box(res, f.boxsize, color, best_track_id, "obj_box");
            k_track.prev = res;

            frontier_ws::msg::DynamicObstacle obs;
            obs.track_id = best_track_id;
            obs.x = res.x_;
            obs.y = res.y_;
            obs.z = res.z_;
            obs.vx = k_track.obj_X[3];
            obs.vy = k_track.obj_X[4];
            obs.vz = k_track.obj_X[5];
            obs_pub->publish(obs);

            break;
        }
    }

    // 3. 1차 best_pair와 appearance 포함 후 best 비교
    // for (auto &f : r)
    // {
    // if (!f.best_pair) // 1차 cost 계산에서 best_pair였는지 확인
    //     continue;
    // if (f.app_cost < 0.0)
    //     continue;

    // bool is_in_best = false;

    // for (auto &[best_det_id, best_track_id] : best_pairs)
    // {
    //     if (f.det_id == best_det_id && f.track_id == best_track_id)
    //     {
    //         is_in_best = true;
    //         break;
    //     }
    // }

    // if (!is_in_best) // id 재매칭 해야함 (track이랑 det 재매칭)
    // {
    //     RCLCPP_WARN(this->get_logger(),
    //                 "appearance would flip: frame=%d det=%d old_track=%d app=%.6f",
    //                 f.frame_id, f.det_id, f.track_id, f.app_cost);

    //     // // 실험용 임시 swap
    //     // int first_id = -1;
    //     // int second_id = -1;
    //     // bool init = false;

    //     // if (yolo_tracks.size() > 1)
    //     // {
    //     //     for (auto &[id, y] : yolo_tracks)
    //     //     {
    //     //         if (!init)
    //     //         {
    //     //             first_id = id;
    //     //             init = true;
    //     //             continue;
    //     //         }
    //     //         second_id = id;
    //     //         break;
    //     //     }

    //     //     if (first_id != -1 && second_id != -1)
    //     //     {
    //     //         std::swap(yolo_tracks[first_id], yolo_tracks[second_id]);
    //     //         RCLCPP_WARN(this->get_logger(),
    //     //                     "TEMP swap executed: %d <-> %d",
    //     //                     first_id, second_id);
    //     //     }
    //     // }
    // }
    // }

    // 4. 이번 프레임 track_emb를 다음 프레임 reference로 저장
    prev_track_emb_map_.clear();

    for (auto &f : r)
    {
        if (f.rep_emb.size() == 0 || f.rep_img.data.empty())
            continue;

        prev_track_emb_map_[f.track_id].emb = f.rep_emb;
        prev_track_emb_map_[f.track_id].img = f.rep_img;
        prev_track_emb_frame_ = f.frame_id;
    }
}

double DetectNode::cosine_similarity(const Eigen::VectorXf &a, const Eigen::VectorXf &b)
{
    if (a.size() == 0 || b.size() == 0)
        return -1.0;
    if (a.size() != b.size())
        return -1.0;

    float na = a.norm();
    float nb = b.norm();
    if (na < 1e-6 || nb < 1e-6)
        return -1.0;

    return a.dot(b) / (na * nb);
}

double DetectNode::calculate_iou(const Eigen::Vector3d &s, const Eigen::Vector3d &prev_s, const obj &c_kf, const obj &p_kf)
{
    double p_x1 = p_kf.x_ - (prev_s.x() / 2); // 사각형 왼쪽 아래 뒷쪽
    double p_y1 = p_kf.y_ - (prev_s.y() / 2);
    double p_z1 = p_kf.z_ - (prev_s.z() / 2);
    double p_x2 = p_kf.x_ + (prev_s.x() / 2); // 사각형 오른쪽 위 앞쪽
    double p_y2 = p_kf.y_ + (prev_s.y() / 2);
    double p_z2 = p_kf.z_ + (prev_s.z() / 2);

    double c_x1 = c_kf.x_ - (s.x() / 2);
    double c_y1 = c_kf.y_ - (s.y() / 2);
    double c_z1 = c_kf.z_ - (s.z() / 2);
    double c_x2 = c_kf.x_ + (s.x() / 2);
    double c_y2 = c_kf.y_ + (s.y() / 2);
    double c_z2 = c_kf.y_ + (s.z() / 2);

    double x1 = std::max(p_x1, c_x1);
    double y1 = std::max(p_y1, c_y1);
    double z1 = std::max(p_z1, c_z1);

    double x2 = std::min(p_x2, c_x2);
    double y2 = std::min(p_y2, c_y2);
    double z2 = std::min(p_z2, c_z2);

    double intersection = 0.0;

    if (x1 < x2 && y1 < y2 && z1 < z2)
    {
        intersection = (x2 - x1) * (y2 - y1) * (z2 - z1); // 교집합
    }

    double vol_c = s.x() * s.y() * s.z();
    double vol_p = prev_s.x() * prev_s.y() * prev_s.z();

    double union_ = vol_c + vol_p - intersection; // 합집합

    if (union_ <= 1e-9)
        return 0.0;

    return intersection / union_;
}

void DetectNode::calculate_Kalman(Vector6d &X, Matrix6d &P, const obj &prev, const obj &curr, bool vis, obj &res, Eigen::Matrix3d &S)
{
    kalman = false;
    // time source가 같은지 체크(디버그용)
    if (curr.stamp.get_clock_type() != prev.stamp.get_clock_type())
    {
        RCLCPP_WARN(this->get_logger(), "kal - Time source mismatch: curr=%d prev=%d",
                    (int)curr.stamp.get_clock_type(),
                    (int)prev.stamp.get_clock_type());
        return;
    }

    double dt = (curr.stamp - prev.stamp).seconds();
    if (dt < 1e-3)
        return;

    Y << curr.x_, curr.y_, curr.z_;
    A(0, 3) = A(1, 4) = A(2, 5) = dt;

    // dynamic update
    X = A * X;
    P = A * P * A.transpose() + G * Q * G.transpose();

    // kalman gain
    K = P * C.transpose() * (C * P * C.transpose() + R).inverse();

    // measurement update
    X = X + K * (Y - C * X);
    P = P - K * C * P;

    res.x_ = X[0];
    res.y_ = X[1];
    res.z_ = X[2];
    res.stamp = curr.stamp;

    vel kalman_vel;
    kalman_vel.vx_ = X[3];
    kalman_vel.vy_ = X[4];
    kalman_vel.vz_ = X[5];

    S = (C * P * C.transpose() + R);

    double speed = std::sqrt(kalman_vel.vx_ * kalman_vel.vx_ + kalman_vel.vy_ * kalman_vel.vy_ + kalman_vel.vz_ * kalman_vel.vz_);
    if (speed < 1e-3 || speed > 3.0)
        return;
    kalman = true;
    if (vis)
    {
        visualize_obj(res, { 0.0f, 1.0f, 0.0f }, "kalman_obj", curr.id);             // 보정된 좌표: green
        visualize_vel(res, kalman_vel, { 0.0f, 1.0f, 0.0f }, "kalman_vel", curr.id); // 보정된 속도: green
        // RCLCPP_INFO(this->get_logger(), "속도: %.2f", speed);
    }
}

void DetectNode::calculate_vel(const obj &prev, const obj &curr, vel &v)
{
    // time source가 같은지 체크(디버그용)
    if (curr.stamp.get_clock_type() != prev.stamp.get_clock_type())
    {
        RCLCPP_WARN(this->get_logger(), "vel - Time source mismatch: curr=%d prev=%d",
                    (int)curr.stamp.get_clock_type(),
                    (int)prev.stamp.get_clock_type());
        return;
    }

    double dt = (curr.stamp - prev.stamp).seconds();
    if (dt < 1e-3)
        return;

    double vx = (curr.x_ - prev.x_) / dt;
    double vy = (curr.y_ - prev.y_) / dt;
    double vz = (curr.z_ - prev.z_) / dt;
    double speed = std::sqrt(vx * vx + vy * vy + vz * vz);
    if (speed < 0.1 || speed > 3.0)
        return;

    obj c;
    c.x_ = curr.x_;
    c.y_ = curr.y_;
    c.z_ = curr.z_;

    v.vx_ = vx;
    v.vy_ = vy;
    v.vz_ = vz;
}

std::array<float, 3> DetectNode::get_color(int id)
{
    float h = (id * 37) % 360; // hue 분산
    float s = 0.8f;
    float v = 0.9f;

    float c = v * s;
    float x = c * (1 - fabs(fmod(h / 60.0, 2) - 1));
    float m = v - c;

    float r, g, b;

    if (h < 60)
    {
        r = c;
        g = x;
        b = 0;
    }
    else if (h < 120)
    {
        r = x;
        g = c;
        b = 0;
    }
    else if (h < 180)
    {
        r = 0;
        g = c;
        b = x;
    }
    else if (h < 240)
    {
        r = 0;
        g = x;
        b = c;
    }
    else if (h < 300)
    {
        r = x;
        g = 0;
        b = c;
    }
    else
    {
        r = c;
        g = 0;
        b = x;
    }

    return { r + m, g + m, b + m };
}

void DetectNode::visualize_box(const obj &obj, const Eigen::Vector3d &box, const std::array<float, 3> &color, int id, const std::string &ns)
{
    visualization_msgs::msg::Marker marker;
    visualization_msgs::msg::MarkerArray arr;
    marker.header.frame_id = obstacle_frame_;
    marker.header.stamp = rclcpp::Time(obj.stamp, RCL_ROS_TIME);
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    marker.pose.position.x = obj.x_;
    marker.pose.position.y = obj.y_;
    marker.pose.position.z = obj.z_;
    marker.scale.x = box.x();
    marker.scale.y = box.y();
    marker.scale.z = box.z();
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = 0.3;

    arr.markers.push_back(marker);

    std::string str = std::to_string(id);

    visualization_msgs::msg::Marker text;
    text.header.frame_id = obstacle_frame_;
    text.header.stamp = rclcpp::Time(obj.stamp, RCL_ROS_TIME);
    text.ns = ns;
    text.id = id + 10;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.lifetime = rclcpp::Duration::from_seconds(0.5);
    text.text = str;

    text.pose.position.x = obj.x_ + box.x();
    text.pose.position.y = obj.y_ - box.y() / 2;
    text.pose.position.z = obj.z_ + box.z() / 2 + 0.05;
    text.scale.z = 0.2; // 글자크기

    text.color.r = color[0];
    text.color.g = color[1];
    text.color.b = color[2];
    text.color.a = 1.0;
    arr.markers.push_back(text);

    box_pub->publish(arr);
}

void DetectNode::visualize_vel(const obj &obj, const vel &obj_vels, const std::array<float, 3> &color, const std::string &ns, int trac_id)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = obstacle_frame_;
    marker.header.stamp = rclcpp::Time(obj.stamp, RCL_ROS_TIME);
    marker.ns = ns;
    marker.id = trac_id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(1.0);

    geometry_msgs::msg::Point start, end;
    start.x = obj.x_;
    start.y = obj.y_;
    start.z = obj.z_;

    const double scale_seconds = 0.4; // 화살표 길이

    end.x = start.x + obj_vels.vx_ * scale_seconds;
    end.y = start.y + obj_vels.vy_ * scale_seconds;
    end.z = start.z + obj_vels.vz_ * scale_seconds;

    marker.points.clear();
    marker.points.push_back(start);
    marker.points.push_back(end);
    marker.scale.x = 0.02; // shaft diameter
    marker.scale.y = 0.04; // head diameter
    marker.scale.z = 0.06; // head length
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = 1.0;

    marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    visualization_msgs::msg::MarkerArray arr;
    arr.markers.push_back(marker);
    vel_pub->publish(arr);
}

void DetectNode::visualize_obj(const obj &obj, const std::array<float, 3> &color, const std::string &ns, int trac_id)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = obstacle_frame_;
    marker.header.stamp = (obj.stamp);
    marker.ns = ns;
    marker.id = trac_id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    marker.pose.position.x = obj.x_;
    marker.pose.position.y = obj.y_;
    marker.pose.position.z = obj.z_;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = 1.0;

    visualization_msgs::msg::MarkerArray arr;
    arr.markers.push_back(marker);

    obj_pub->publish(arr);
}

int main(int argc, const char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectNode>());
    rclcpp::shutdown();
    return 0;
}
