#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    ld = LaunchDescription()

    use_sim_time = True
    pkg_dir = get_package_share_directory('frontier_ws')
    param_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    cartographer_config_dir = os.path.join(get_package_share_directory('frontier_ws'), 'lua')
    
    robots = [
        {"ns": "tb3_0"},
        {"ns": "tb3_1"},
        #{"ns": "tb3_2"}
    ]

    # 1) 로봇별 base -> scan static TF 
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


   # 2) Cartographer SLAM 노드 (로봇별 실행)
    for r in robots:
        ns = r["ns"]

        lua_file = f"{ns}_turtlebot3_lds_2d.lua"

        # A. 메인 카토그래퍼 노드
        ld.add_action(Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            namespace=ns,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', lua_file,
            ],
            remappings=[
                ('scan', 'scan'),
                ('odom', 'odom'),
                ('imu', 'imu'),
            ]
        ))

        # B. Occupancy Grid Node (Submap -> Map 변환기)
        ld.add_action(Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            namespace=ns,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'resolution': 0.05,
                'publish_period_sec': 1.0
            }],
            remappings=[
                ('/map', f'/{ns}/map'), # 최종 지도를 로봇별 맵 토픽으로 매핑
            ]
        ))

    # 3) tb3_x/map을 world frame에 고정
    ld.add_action(Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="world_to_tb3_0_map",
    arguments=[
        "-0.978", "1.92", "0",
        "0", "0", "0",
        "world",
        "tb3_0/map",
    ],
    ))
    ld.add_action(Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_tb3_1_map",
        arguments=[
            "0.978", "1.92", "0",
            "0", "0", "0",
            "world",
            "tb3_1/map",
        ],
    ))

    # ld.add_action(Node(
    # package="tf2_ros",
    # executable="static_transform_publisher",
    # name="world_to_tb3_2_map",
    # arguments=[
    #     "-0.978", "-1.92", "0",
    #     "0", "0", "0",
    #     "world",
    #     "tb3_2/map",
    # ],
    # ))
    

    # 4) frontier 노드는 로봇별로 유지
    def frontier_node(ns: str):
        map_frame = f"{ns}/map"
        global_frame = "world"

        return Node(
            package="frontier_ws",
            executable="frontier_multi",
            name="frontier_multi",
            namespace=ns,
            output="screen",
            # yaml 파일과 개별 파라미터를 동시에 넘겨줍니다.
            parameters=[param_file, {
                "use_sim_time": use_sim_time,
                "robot_id": ns,
                "map_topic": "/merge_map",
                "map_frame": "world",
                "base_frame": f"{ns}/base_footprint",
                "global_frame": "world"
            }],
        )

    for r in robots:
        ld.add_action(frontier_node(r["ns"]))

    return ld
