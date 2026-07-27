#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit


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
    scripts_dir = os.path.join(pkg_dir, "scripts")

    # ---- UWB 관련 launch argument 실제 값 ----
    front_serial_port = LaunchConfiguration("front_serial_port").perform(context)
    back_serial_port = LaunchConfiguration("back_serial_port").perform(context)
    anchor_x_str = LaunchConfiguration("anchor_x").perform(context)
    anchor_y_str = LaunchConfiguration("anchor_y").perform(context)

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
    # 3-1) world -> namespace/map static TF
    # =========================================================
    # UWB 센서 위치 추정 사용하지 않을 시에 world -> map static TF 사용
    # actions.append(
    #     Node(
    #         package="tf2_ros",
    #         executable="static_transform_publisher",
    #         name=f"world_to_{ns}_map",
    #         output="screen",
    #         arguments=[
    #             "--x", "0.0",
    #             "--y", "0.0",
    #             "--z", "0.0",
    #             "--yaw", "0.0",
    #             "--pitch", "0.0",
    #             "--roll", "0.0",
    #             "--frame-id", "world",
    #             "--child-frame-id", f"{ns}/map",
    #         ],
    #     )
    # )

    

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
    # 3-2) UWB: front/back 태그 trilateration + heading
    # =========================================================
    # frontier, detect 노드 정의때문에 순서 변경 ㅠㅠ
    # UWB 센서 위치 추정 사용할 시에 동적으로 world -> map TF 계산
    trilateration_script = os.path.join(scripts_dir, "trilateration_node.py")
    heading_script = os.path.join(scripts_dir, "uwb_heading_node.py")
    
    actions.append(
        TimerAction(
            period=2.0,
            actions=[
                # front 태그
                ExecuteProcess(
                    cmd=[
                        "python3", trilateration_script, "--ros-args",
                        "-r", f"__node:=trilateration_{ns}_front",
                        "-p", f"serial_port:={front_serial_port}",
                        "-p", "baud_rate:=115200",
                        "-p", "frame_id:=world",
                        "-p", f"anchor_x:={anchor_x_str}",
                        "-p", f"anchor_y:={anchor_y_str}",
                        "--remap", f"/uwb/position:=/{ns}/uwb_front/position",
                        "--remap", f"/uwb/ranges:=/{ns}/uwb_front/ranges",
                    ],
                    output="screen",
                ),
                # back 태그
                ExecuteProcess(
                    cmd=[
                        "python3", trilateration_script, "--ros-args",
                        "-r", f"__node:=trilateration_{ns}_back",
                        "-p", f"serial_port:={back_serial_port}",
                        "-p", "baud_rate:=115200",
                        "-p", "frame_id:=world",
                        "-p", f"anchor_x:={anchor_x_str}",
                        "-p", f"anchor_y:={anchor_y_str}",
                        "--remap", f"/uwb/position:=/{ns}/uwb_back/position",
                        "--remap", f"/uwb/ranges:=/{ns}/uwb_back/ranges",
                    ],
                    output="screen",
                ),
            ],
        )
    )
    
    heading_process = ExecuteProcess(
        cmd=[
            "python3", heading_script, "--ros-args",
            "-r", f"__node:=uwb_heading_{ns}",
            "-p", f"namespace:={ns}",
            "-p", f"front_topic:=/{ns}/uwb_front/position",
            "-p", f"back_topic:=/{ns}/uwb_back/position",
            "-p", "world_frame:=world",
            "-p", "num_samples:=30",
            "-p", "keep_publishing:=false",
        ],
        output="screen",
    )
    actions.append(
        TimerAction(
            period=6.0,
            actions=[heading_process],
        )
    )
    # ---- world -> {ns}/map TF publish 완료 후 시작 ----
    actions.append(
        RegisterEventHandler(
            OnProcessExit(
                target_action=heading_process,
                on_exit=[
                    frontier_node,   # 탐사 시작
                    detect_node,     # 사람 감지 
                ],
            )
        )
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
        scoped=True,
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
                }.items(),
            )
        ],
    )

    # =========================================================
    # 실행할 노드 선택
    # camera, yolo는 TF 정렬과 무관하게 바로 시작
    # frontier_node, detect_node는 heading 프로세스 종료(=world->map TF 완료) 후
    # RegisterEventHandler를 통해 자동으로 시작됨 (아래 3-1 UWB 섹션 참고)
    # =========================================================

    actions.append(camera_node)
    actions.append(yolo_node)

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_namespace",
            default_value="tb3_0",
            description="Namespace of the robot.",
        ),
        DeclareLaunchArgument(
            "front_serial_port",
            default_value="/dev/ttyUSB_tb3_0_front",
            description="UWB front tag serial port.",
        ),
        DeclareLaunchArgument(
            "back_serial_port",
            default_value="/dev/ttyUSB_tb3_0_back",
            description="UWB back tag serial port.",
        ),
        DeclareLaunchArgument(
            "anchor_x",
            default_value="[0.0,1.6,1.6]",
            description="UWB anchor x coordinates (m), anchor_id order 0,1,2...",
        ),
        DeclareLaunchArgument(
            "anchor_y",
            default_value="[0.0,1.2,-1.2]",
            description="UWB anchor y coordinates (m), anchor_id order 0,1,2...",
        ),

        OpaqueFunction(function=launch_setup),
    ])