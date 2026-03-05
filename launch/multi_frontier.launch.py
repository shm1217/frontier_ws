#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    use_sim_time = True

    # # slam_toolbox 기본 async mapper 파라미터 파일 
    slam_pkg = get_package_share_directory("slam_toolbox")
    default_async_yaml = os.path.join(slam_pkg, "config", "mapper_params_online_async.yaml")

    robots = [
        {"ns": "tb3_0"},
        {"ns": "tb3_1"},
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

    # 2) slam_toolbox async를 로봇별로 1개씩
    #
    # - map_frame/odom_frame/base_frame은 반드시 로봇별로 분리 (충돌 방지)
    # - scan topic도 로봇 네임스페이스 안에서 "scan"으로 받도록 (즉 /tb3_0/scan, /tb3_1/scan)
    #
    for r in robots:
        ns = r["ns"]

        ld.add_action(Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            namespace=ns,
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time,
                 },
                default_async_yaml,
                {
                    "map_update_interval": 4.0,
                    # 로봇별 프레임 분리
                    "map_frame": f"{ns}/map",
                    "odom_frame": f"{ns}/odom",
                    "base_frame": f"{ns}/base_footprint",

                    # scan 토픽: namespace 안의 "scan"을 구독
                    # (대부분 slam_toolbox는 scan_topic 파라미터 사용)
                    "scan_topic": "scan",

                    "mode": "mapping",
                    #"transform_publish_period": 0.05,
                    # scan 값 3분의 1로 받기
                    "throttle_scans": 3,

                },
            ],

            remappings=[
                ("/map", f"/{ns}/map"),

                ("/map_metadata", f"/{ns}/map_metadata"),
            ],
        ))

    # # 3) 전역 프레임을 tb3_0/map으로 쓰기 위한 map 간 TF
    # # 0번 좌표계에서 본 1번의 위치 및 자세 
    # ld.add_action(Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="map0_to_map1",
    #     output="screen",
    #     arguments=[
    #         "--x", "-2.5", "--y", "1.94", "--z", "0.0",
    #         "--yaw", "0.0", "--pitch", "0.0", "--roll", "0.0",
    #         "--frame-id", "tb3_0/map",
    #         "--child-frame-id", "tb3_1/map",
    #     ],
    # ))

    ld.add_action(Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="world_to_tb3_0_map",
    arguments=[
        "0", "0", "0",
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
            "0", "0", "0",
            "0", "0", "0",
            "world",
            "tb3_1/map",
        ],
    ))
    

    # 4) frontier 노드는 로봇별로 유지
    #
    # - tb3_0은 자기 map(tb3_0/map)을 그대로 global로 사용
    # - tb3_1은 map_frame은 tb3_1/map(OccupancyGrid frame과 맞춰야 안전),
    #   global_frame만 tb3_0/map으로 두고 TF를 통해 전역 좌표로 통일
    def frontier_node(ns: str):
        map_frame = f"{ns}/map"
        global_frame = "world"

        return Node(
            package="frontier_ws",
            executable="frontier_multi",
            name="frontier_multi",
            namespace=ns,
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "robot_id": ns,

                # slam_toolbox가 namespace 안에 map 토픽을 내는 구조(/tb3_x/map)가 일반적이라
                # 상대 토픽 "map" 사용
                "map_topic": "/merge_map", 
                "scan_topic": "scan",
                "cmd_topic": "cmd_vel",

                "map_frame": global_frame,
                "base_frame": f"{ns}/base_footprint",

                # 전역 프레임만 tb3_0/map으로 통일
                "global_frame": global_frame,

                # "visited_out_topic": "visited_point",
                # "visited_in_topics": ["/tb3_0/visited_point", "/tb3_1/visited_point"],
                "reserve_out_topic": "/global_goal_reservation",
                # "reserve_in_topics": ["/tb3_0/goal_reservation", "/tb3_1/goal_reservation"],
                # "cluster_reserve_out_topic": "cluster_reservation",
                # "cluster_reserve_in_topics": ["/tb3_0/cluster_reservation", "/tb3_1/cluster_reservation"],
                # "cluster_reserve_ttl_s": 6.0,
            }],
        )

    for r in robots:
        ld.add_action(frontier_node(r["ns"]))

    return ld
