#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction  # 추가
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    use_sim_time = False
    pkg_dir = get_package_share_directory('frontier_ws')
    param_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    robots = [
        {"ns": "tb3_0"},
        {"ns": "tb3_1"},
        # {"ns": "tb3_2"}
    ]

    # =========================================================
    # 1) 로봇별 base -> scan static TF (카토그래퍼 때와 동일)
    # =========================================================
    for r in robots:
        ns = r["ns"]
        ld.add_action(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"{ns}_base_to_scan",
            output="screen",
            arguments=[
                "--x", "0.0", "--y", "0.0", "--z", "0.20",
                "--yaw", "0.0", "--pitch", "0.0", "--roll", "0.0",
                "--frame-id", f"{ns}/base_footprint",
                "--child-frame-id", f"{ns}/base_scan",
            ],
        ))

    # =========================================================
    # 2) SLAM Toolbox (로봇별 실행, online async 모드)
    # =========================================================
    for r in robots:
        ns = r["ns"]

        slam_params = {
            "use_sim_time": use_sim_time,

            # 프레임 설정
            "odom_frame":  f"{ns}/odom",
            "map_frame":   f"{ns}/map",
            "base_frame":  f"{ns}/base_footprint",
            "scan_topic":  f"/{ns}/scan",

            # 맵 해상도 (카토그래퍼와 동일하게 유지)
            "resolution": 0.05,

            # 레이저 범위
            "max_laser_range": 8.0,   # TB3 LDS 최대 범위
            "min_laser_range": 0.12,

            # TF 안정성 (튐 방지 핵심)
            "transform_publish_period": 0.02,  # 50Hz
            "tf_buffer_duration": 30.0,
            "transform_timeout": 0.2,

            # 맵 퍼블리시
            "map_update_interval": 1.0,

            # 루프클로저 (실내 환경에서 유용)
            "do_loop_closing": True,
            "loop_search_maximum_distance": 3.0,
            "loop_match_minimum_chain_size": 10,
            "loop_match_maximum_variance_coarse": 3.0,

            # 스캔 매칭
            "use_scan_matching": True,
            "use_scan_barycenter": True,
            "minimum_travel_distance": 0.05,
            "minimum_travel_heading": 0.1,

            # 모드: mapping
            "mode": "mapping",
            "debug_logging": False,
        }

        ld.add_action(Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace=ns,
            output='screen',
            parameters=[slam_params],
            remappings=[
                ('scan',  f'/{ns}/scan'),
                ('odom',  f'/{ns}/odom'),
                ('/map',  f'/{ns}/map'),
                ('/map_metadata', f'/{ns}/map_metadata'),
            ],
        ))

    # =========================================================
    # 3) world -> tb3_x/map static TF (카토그래퍼 때와 동일)
    # =========================================================
    ld.add_action(Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_tb3_0_map",
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", "0",
            "--yaw", "0", "--pitch", "0", "--roll", "0",
            "--frame-id", "world",
            "--child-frame-id", "tb3_0/map",
        ],
    ))
    ld.add_action(Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_tb3_1_map",
        arguments=[
            "--x", "1.0", "--y", "1.0", "--z", "0",
            "--yaw", "0", "--pitch", "0", "--roll", "0",
            "--frame-id", "world",
            "--child-frame-id", "tb3_1/map",
        ],
    ))

    # =========================================================
    # 4) multirobot_map_merge (SLAM 뜨고 나서 시작)
    # =========================================================
    # ld.add_action(TimerAction(
    #     period=7.0,
    #     actions=[Node(
    #         package='multirobot_map_merge',
    #         executable='map_merge',
    #         name='map_merge',
    #         output='screen',
    #         parameters=[{
    #             'use_sim_time': use_sim_time,
    #             'robot_namespace': 'tb3',
    #             'merged_map_topic': 'merge_map',
    #             'world_frame': 'world',
    #             'known_init_poses': False,
    #             'merging_rate': 2.0,
    #             'discovery_rate': 0.05,
    #             'expand_slam_maps_to_common_canvas': True,
    #             'expand_slam_maps_apply_init_pose': True,
    #             '/tb3_0/map_merge/init_pose_x': 0.0,
    #             '/tb3_0/map_merge/init_pose_y': 0.0,
    #             '/tb3_0/map_merge/init_pose_z': 0.0,
    #             '/tb3_0/map_merge/init_pose_yaw': 0.0,
    #             '/tb3_1/map_merge/init_pose_x': 1.0,
    #             '/tb3_1/map_merge/init_pose_y': 1.0,
    #             '/tb3_1/map_merge/init_pose_z': 0.0,
    #             '/tb3_1/map_merge/init_pose_yaw': 0.0,
    #         }],
    #     )]
    # ))


    # =========================================================
    # 4) frontier 노드 (기존과 동일)
    # =========================================================
    def frontier_node(ns: str):
        return Node(
            package="frontier_ws",
            executable="frontier_multi",
            name="frontier_multi",
            namespace=ns,
            output="screen",
            parameters=[param_file, {
                "use_sim_time": use_sim_time,
                "robot_id": ns,
                "map_topic": "/merge_map",
                "map_frame": "world",
                "base_frame": f"{ns}/base_footprint",
                "global_frame": "world",
            }],
        )
    
    def detect_node(ns: str):
        return Node(
        package= 'frontier_ws',
        namespace= ns,
        executable= 'detect_node',
        output='screen',
        parameters=[{
            "use_sim_time": True,
            "robot_id": ns,
            "yolo_detections_topic": f"/{ns}/yolo/detections_3d",
            "embedding_topic": f"/{ns}/embedding",
            "image_topic": f"/{ns}/camera/camera/color/image_raw",
            "camera_info_topic": f"/{ns}/camera/camera/color/camera_info",
            "camera_link_frame": f"{ns}/camera_link",
            "obstacle_frame": "world",
            "camera_optical_frame": f"{ns}/camera_color_optical_frame",
        }] # True: /clock 사용, ros bag 사용할 때, False: system time 사용
    )

    def yolo_node(ns: str):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('yolo_bringup'),
                    'launch',
                    'yolo.launch.py'
                )
            ),
            ## 실물에서
            launch_arguments={
                "use_3d": "True",
                "input_image_topic": f"/{ns}/camera/camera/color/image_raw",
                "target_frame": f"{ns}/camera_link",
                "input_depth_topic": f"/{ns}/camera/camera/aligned_depth_to_color/image_raw",
                "input_depth_info_topic": f"/{ns}/camera/camera/color/camera_info",
                "use_sim_time":"true",
                "namespace": f"{ns}/yolo", 
            }.items(),

            # ## gazebo에서
            # launch_arguments={
            #     "use_3d": "True",
            #     "input_image_topic": f"/{ns}/camera/camera/image_raw",
            #     "target_frame": f"{ns}/camera_link",
            #     "input_depth_topic": f"/{ns}/camera/camera/depth/image_raw",
            #     "input_depth_info_topic": f"/{ns}/camera/camera/camera_info",
            #     "use_sim_time":"true",
            #     "depth_image_units_divisor": "1",
            #     "namespace": f"{ns}/yolo",
            # }.items(),
        )
    
    def camera_node(ns: str):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('realsense2_camera'),
                    'examples',
                    'align_depth',
                    'rs_align_depth_launch.py'
                )
            ),
            launch_arguments={
                "camera_namespace": ns + "/camera",
                "camera_name": "camera",
            }.items()
    )

    for r in robots:
        ns = r["ns"]
        ld.add_action(frontier_node(ns))
        # ld.add_action(detect_node(ns))
        # ld.add_action(yolo_node(ns))
        # ld.add_action(camera_node(ns))

    return ld
