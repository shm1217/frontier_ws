#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction  # 추가
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    ld = LaunchDescription()

    use_sim_time = True
    use_sim_time_str = "True"
    pkg_dir = get_package_share_directory('frontier_ws')
    param_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    dwb_param_file = os.path.join(pkg_dir, 'config', 'dwb_controller.yaml')

    # TODO: 로봇 대수 설정 가능
    robots = [
        {"ns": "tb3_0"},
        {"ns": "tb3_1"},
        {"ns": "tb3_2"}
    ]

    # =========================================================
    # 로봇별 base -> scan static TF 
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
    # SLAM Toolbox
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

            # 맵 해상도 
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
                ('/scan',  f'/{ns}/scan'),
                ('/odom',  f'/{ns}/odom'),
                ('/map',  f'/{ns}/map'),
                ('/map_metadata', f'/{ns}/map_metadata'),
            ],
        ))

    # =========================================================
    # frontier 노드
    # =========================================================
    def frontier_node(ns: str):
        frontier_overrides = {
            "use_sim_time": use_sim_time,
            "robot_id": ns,
            "map_topic": "/merge_map",
            "map_frame": "world",
            "base_frame": f"{ns}/base_footprint",
            "global_frame": "world",
            "merge_map_stale_s": 5.0,
            "local_map_topic": "map",
        }
        return Node(
            package="frontier_ws",
            executable="frontier_multi_uwb", 
            name="frontier_multi_uwb",
            namespace=ns,
            output="screen",
            parameters=[param_file, frontier_overrides],
        )

    def dwb_nodes(ns: str):
        dwb_rewrites = {
            'use_sim_time': use_sim_time_str,
            'robot_base_frame': f'{ns}/base_footprint',
            'global_frame': f'{ns}/odom',
            'topic': f'/{ns}/scan',
        }
        configured_params = ParameterFile(
            RewrittenYaml(
                source_file=dwb_param_file,
                root_key=ns,
                param_rewrites=dwb_rewrites,
                convert_types=True),
            allow_substs=True)
        return [
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                namespace=ns,
                output='screen',
                parameters=[configured_params],
                remappings=[('cmd_vel', 'cmd_vel_dwb')]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_controller',
                namespace=ns,
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    'node_names': ['controller_server'],
                }]),
        ]
    
    def detect_node(ns: str):
        return Node(
        package= 'frontier_ws',
        namespace= ns,
        executable= 'detect_node',
        output='screen',
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_id": ns,
            "yolo_detections_topic": f"/{ns}/yolo/detections_3d",
            "embedding_topic": f"/{ns}/embedding",
            "image_topic": f"/{ns}/camera/camera/color/image_raw",
            "camera_info_topic": f"/{ns}/camera/camera/color/camera_info",
            "camera_link_frame": f"{ns}/camera_link",
            "obstacle_frame": f"{ns}/map",
            "camera_optical_frame": f"{ns}/camera_color_optical_frame",
        }] 
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
            launch_arguments={
                "use_3d": "True",
                "input_image_topic": f"/{ns}/camera/camera/image_raw",
                "target_frame": f"{ns}/camera_link",
                "input_depth_topic": f"/{ns}/camera/camera/depth/image_raw",
                "input_depth_info_topic": f"/{ns}/camera/camera/camera_info",
                "use_sim_time":"true",
                "depth_image_units_divisor": "1",
                "namespace": f"{ns}/yolo",
            }.items(),
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

        for node in dwb_nodes(ns):
            ld.add_action(node)
        ld.add_action(frontier_node(ns))
        ld.add_action(detect_node(ns))
        ld.add_action(yolo_node(ns))
        ld.add_action(camera_node(ns))

    return ld
