#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # 실행 명령에서 받은 namespace 실제 문자열
    ns = LaunchConfiguration("robot_namespace").perform(context)

    use_sim_time = False

    pkg_dir = get_package_share_directory("frontier_ws")
    param_file = os.path.join(
        pkg_dir,
        "config",
        "params.yaml",
    )

    actions = []

    # =========================================================
    # 1) base_footprint -> base_scan static TF
    # =========================================================
    actions.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"{ns}_base_to_scan",
            output="screen",
            arguments=[
                "--x", "0.0",
                "--y", "0.0",
                "--z", "0.20",
                "--yaw", "0.0",
                "--pitch", "0.0",
                "--roll", "0.0",
                "--frame-id", f"{ns}/base_footprint",
                "--child-frame-id", f"{ns}/base_scan",
            ],
        )
    )

    # =========================================================
    # 2) SLAM Toolbox
    # =========================================================
    slam_params = {
        "use_sim_time": use_sim_time,

        # 프레임 설정
        "odom_frame": f"{ns}/odom",
        "map_frame": f"{ns}/map",
        "base_frame": f"{ns}/base_footprint",
        "scan_topic": f"/{ns}/scan",

        # 맵 해상도
        "resolution": 0.05,

        # 레이저 범위
        "max_laser_range": 8.0,
        "min_laser_range": 0.12,

        # TF 설정
        "transform_publish_period": 0.02,
        "tf_buffer_duration": 30.0,
        "transform_timeout": 0.2,

        # 맵 퍼블리시 주기
        "map_update_interval": 1.0,

        # 루프 클로저
        "do_loop_closing": True,
        "loop_search_maximum_distance": 3.0,
        "loop_match_minimum_chain_size": 10,
        "loop_match_maximum_variance_coarse": 3.0,

        # 스캔 매칭
        "use_scan_matching": True,
        "use_scan_barycenter": True,
        "minimum_travel_distance": 0.05,
        "minimum_travel_heading": 0.1,

        # Mapping 모드
        "mode": "mapping",
        "debug_logging": False,
    }

    actions.append(
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            namespace=ns,
            output="screen",
            parameters=[slam_params],
            remappings=[
                ("scan", f"/{ns}/scan"),
                ("odom", f"/{ns}/odom"),
                ("map", f"/{ns}/map"),
                ("map_metadata", f"/{ns}/map_metadata"),
            ],
        )
    )

    # =========================================================
    # 3) world -> namespace/map static TF
    # =========================================================
    actions.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"world_to_{ns}_map",
            output="screen",
            arguments=[
                "--x", "0.0",
                "--y", "0.0",
                "--z", "0.0",
                "--yaw", "0.0",
                "--pitch", "0.0",
                "--roll", "0.0",
                "--frame-id", "world",
                "--child-frame-id", f"{ns}/map",
            ],
        )
    )

    # =========================================================
    # 4) Frontier 노드
    # =========================================================
    frontier_node = Node(
        package="frontier_ws",
        executable="frontier_multi",
        name="frontier_multi",
        namespace=ns,
        output="screen",
        parameters=[
            param_file,
            {
                "use_sim_time": use_sim_time,
                "robot_id": ns,

                # 단일 로봇 map 사용
                "map_topic": f"/{ns}/map",
                "map_frame": f"{ns}/map",
                "base_frame": f"{ns}/base_footprint",
                "global_frame": f"{ns}/map",
            },
        ],
    )

    # =========================================================
    # 5) Detect 노드
    # =========================================================
    detect_node = Node(
        package="frontier_ws",
        executable="detect_node",
        name="detect_node",
        namespace=ns,
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_id": ns,

            "yolo_detections_topic":
                f"/{ns}/yolo/detections_3d",

            "embedding_topic":
                f"/{ns}/embedding",

            "image_topic":
                f"/{ns}/camera/camera/color/image_raw",

            "camera_info_topic":
                f"/{ns}/camera/camera/color/camera_info",

            "camera_link_frame":
                f"{ns}/camera_link",

            "obstacle_frame":
                "world",

            "camera_optical_frame":
                f"{ns}/camera_color_optical_frame",
        }],
    )

    # =========================================================
    # 6) YOLO launch
    # 상위 launch argument 전달 차단
    # =========================================================
    yolo_launch_file = os.path.join(
        get_package_share_directory("yolo_bringup"),
        "launch",
        "yolo.launch.py",
    )

    yolo_node = GroupAction(
        scoped=True,
        forwarding=False,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    yolo_launch_file
                ),
                launch_arguments={
                    "use_3d": "True",

                    "input_image_topic":
                        f"/{ns}/camera/camera/color/image_raw",

                    "target_frame":
                        f"{ns}/camera_link",

                    "input_depth_topic":
                        f"/{ns}/camera/camera/"
                        "aligned_depth_to_color/image_raw",

                    "input_depth_info_topic":
                        f"/{ns}/camera/camera/color/camera_info",

                    "use_sim_time":
                        str(use_sim_time).lower(),

                    "namespace":
                        f"{ns}/yolo",
                }.items(),
            )
        ],
    )

    # =========================================================
    # 7) RealSense align depth launch
    # rs_align_depth_launch.py 그대로 사용
    # =========================================================
    realsense_launch_file = os.path.join(
        get_package_share_directory("realsense2_camera"),
        "examples",
        "align_depth",
        "rs_align_depth_launch.py",
    )

    camera_node = GroupAction(
        # 별도의 launch configuration scope 생성
        scoped=True,

        # robot_namespace 등 상위 argument를 넘기지 않음
        forwarding=False,

        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    realsense_launch_file
                ),
                launch_arguments={
                    "camera_namespace": f"{ns}/camera",
                    "camera_name": "camera",
                    "tf_prefix": f"{ns}/",
                    "align_depth.enable": "true",
                    "rgb_camera.color_profile": "424x240x5",
                    "depth_module.depth_profile": "424x240x5",

                    # "enable_color": "true",
                    # "enable_depth": "true",
                    # "enable_infra1": "false",
                    # "enable_infra2": "false",
                    # "enable_gyro": "false",
                    # "enable_accel": "false",
                    # "pointcloud.enable": "false",
                }.items(),
            )
        ],
    )

    # =========================================================
    # 실행할 노드 선택
    # =========================================================

    actions.append(camera_node)
    actions.append(frontier_node)
    # actions.append(yolo_node)
    actions.append(detect_node)

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_namespace",
            default_value="tb3_0",
            description="Namespace of the robot.",
        ),

        OpaqueFunction(function=launch_setup),
    ])
