#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def launch_setup(context, *args, **kwargs):
    use_sim_time_str = LaunchConfiguration("use_sim_time").perform(context)
    use_sim_time = use_sim_time_str.lower() in ("true", "1", "yes")
    use_sim_time_str = "True" if use_sim_time else "False"

    pkg_dir = get_package_share_directory('frontier_ws')
    param_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    dwb_param_file = os.path.join(pkg_dir, 'config', 'dwb_controller.yaml')

    # 이 launch 파일을 실행할 때 지정하는 로봇 네임스페이스 (예: tb3_0)
    ns = LaunchConfiguration("robot_namespace").perform(context)

    actions = []

    # =========================================================
    # 1) base -> scan static TF
    # =========================================================
    #actions.append(Node(
    #    package="tf2_ros",
    #    executable="static_transform_publisher",
    #    name=f"{ns}_base_to_scan",
    #    output="screen",
    #    arguments=[
    #        "--x", "0.0", "--y", "0.0", "--z", "0.20",
    #        "--yaw", "0.0", "--pitch", "0.0", "--roll", "0.0",
    #        "--frame-id", f"{ns}/base_footprint",
    #        "--child-frame-id", f"{ns}/base_scan",
    #    ],
    #))

    # =========================================================
    # 2) SLAM Toolbox
    # =========================================================
    slam_params = {
        "use_sim_time": use_sim_time,

        "odom_frame":  f"{ns}/odom",
        "map_frame":   f"{ns}/map",
        "base_frame":  f"{ns}/base_footprint",
        "scan_topic":  f"/{ns}/scan",

        "resolution": 0.05,
        "scan_queue_size": 1,

        "max_laser_range": 8.0,
        "min_laser_range": 0.12,

        "transform_publish_period": 0.02,
        "tf_buffer_duration": 30.0,
        "transform_timeout": 0.2,

        "map_update_interval": 1.0,

        "do_loop_closing": True,
        "loop_search_maximum_distance": 3.0,
        "loop_match_minimum_chain_size": 10,
        "loop_match_maximum_variance_coarse": 3.0,

        "use_scan_matching": True,
        "use_scan_barycenter": True,
        "minimum_travel_distance": 0.05,
        "minimum_travel_heading": 0.1,

        "mode": "mapping",
        "debug_logging": False,
    }

    actions.append(Node(
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
    # 3) world -> {ns}/map static TF
    # =========================================================
    # actions.append(Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name=f"world_to_{ns}_map",
    #     output="screen",
    #     arguments=[
    #         "--x", "0.0", "--y", "0.0", "--z", "0",
    #         "--yaw", "0", "--pitch", "0", "--roll", "0",
    #         "--frame-id", "world",
    #         "--child-frame-id", f"{ns}/map",
    #     ],
    # ))

    # =========================================================
    # 4) DWB (controller_server + lifecycle_manager)
    # =========================================================
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=dwb_param_file,
            root_key=ns,
            param_rewrites={
                'use_sim_time': use_sim_time_str,
                'robot_base_frame': f'{ns}/base_footprint',
                'global_frame': f'{ns}/odom',
                'topic': f'/{ns}/scan',
            },
            convert_types=True),
        allow_substs=True)

    actions.append(Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace=ns,
        output='screen',
        parameters=[configured_params],
        remappings=[('cmd_vel', 'cmd_vel_dwb')],
    ))
    actions.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_controller',
        namespace=ns,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['controller_server'],
        }],
    ))

    # =========================================================
    # 노드 정의 (필요한 것만 골라서 아래에서 append)
    # =========================================================
    def frontier_node():
        return Node(
            package="frontier_ws",
            executable="frontier_multi_uwb",
            name="frontier_multi_uwb",
            namespace=ns,
            output="screen",
            parameters=[param_file, {
                "use_sim_time": use_sim_time,
                "robot_id": ns,
                "map_topic": "/merge_map",
                "map_frame": "world",
                "base_frame": f"{ns}/base_footprint",
                "global_frame": "world",
                "merge_map_stale_s": 5.0,
                "local_map_topic": "map",
            }],
        )

    def detect_node():
        return Node(
            package='frontier_ws',
            namespace=ns,
            executable='detect_node',
            name='detect_node',
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
            }],
        )

    def yolo_node():
        return GroupAction(
            scoped=True,
            forwarding=False,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('yolo_bringup'),
                            'launch',
                            'yolo.launch.py'
                        )
                    ),
                    launch_arguments={
                        "use_3d": "True",
                        "input_image_topic": f"/{ns}/camera/camera/color/image_raw",
                        "target_frame": f"{ns}/camera_link",
                        "input_depth_topic": f"/{ns}/camera/camera/aligned_depth_to_color/image_raw",
                        "input_depth_info_topic": f"/{ns}/camera/camera/color/camera_info",
                        "use_sim_time": use_sim_time_str,
                        "namespace": f"{ns}/yolo",
                        "device": "cuda:0",
                        "use_tracking": "False",
                        "use_debug": "False",
                        "imgsz_height": "192",
                        "imgsz_width": "320",
                        "max_det": "10",
                        "device": "cpu",
                        "model": "yolov8n.pt",
                    }.items(),
                )
            ],
        )

    def camera_node():
        return GroupAction(
            scoped=True,
            forwarding=False,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('realsense2_camera'),
                            'examples',
                            'align_depth',
                            'rs_align_depth_launch.py'
                        )
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
    # 실행할 노드 선택 (필요 없는 건 주석 처리)
    # =========================================================
    actions.append(camera_node())
    actions.append(frontier_node())
    actions.append(yolo_node())
    actions.append(detect_node())

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_namespace",
            default_value="tb3_0",
            description="이 launch를 실행할 로봇의 네임스페이스 (예: tb3_0, tb3_1)",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="하드웨어에서는 false, bag/시뮬레이션에서는 true",
        ),
        OpaqueFunction(function=launch_setup),
    ])
