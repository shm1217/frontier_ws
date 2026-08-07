#!/usr/bin/env python3
"""Start the UWB-assisted merger.

SLAM and frontier_multi must already be running per robot.  Before registration,
frontier_multi uses its existing local-map fallback.  After registration each
robot explores /merge_map independently and coordinates through goal
reservations.  Do not run manual world->tb3_x/map static publishers together
with this launch.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    scripts = os.path.join(get_package_share_directory("frontier_ws"), "scripts")
    merger = os.path.join(scripts, "merge_map_uwb.py")
    mock_ranger = os.path.join(scripts, "mock_uwb_range_uwb.py")
    robots = LaunchConfiguration("robot_namespaces")

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_namespaces", default_value="['tb3_0','tb3_1']"),
        DeclareLaunchArgument(
            "use_mock_uwb", default_value="false",
            description="하드웨어 UWB 사용 시 false, Gazebo에서는 true"),
        DeclareLaunchArgument(
            "tb3_0_front_serial_port", default_value="/dev/ttyUSB_tb3_0_front"),
        DeclareLaunchArgument(
            "tb3_0_back_serial_port", default_value="/dev/ttyUSB_tb3_0_back"),
        DeclareLaunchArgument(
            "tb3_1_front_serial_port", default_value="/dev/ttyUSB_tb3_1_front"),
        DeclareLaunchArgument(
            "tb3_1_back_serial_port", default_value="/dev/ttyUSB_tb3_1_back"),
        DeclareLaunchArgument("tag_offset_from_base_m", default_value="0.15"),
        DeclareLaunchArgument("anchor_x", default_value="-6.0"),
        DeclareLaunchArgument("anchor_y", default_value="0.0"),
        DeclareLaunchArgument("publish_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("noise_stddev_m", default_value="0.2"),
        DeclareLaunchArgument(
            "model_states_topic", default_value="/gazebo/model_states"),
        DeclareLaunchArgument(
            "initial_world_x", default_value="[0.0,0.0]"), ## 시뮬레이션 상에서 uwb 센서 재현하기 위해 
        DeclareLaunchArgument(
            "initial_world_y", default_value="[0.0,0.0]"),
        DeclareLaunchArgument(
            "initial_world_yaw", default_value="[0.0,0.0]"),

        ## 하드웨어 
        # 로봇마다 앞/뒤 태그용 시리얼 노드를 하나씩 실행
        # ExecuteProcess(
        #     cmd=[
        #         "python3", ranger, "--ros-args",
        #         "-r", "__node:=uwb_range_tb3_0_front",
        #         "-p", ["serial_port:=", LaunchConfiguration("tb3_0_front_serial_port")],
        #         "-p", "anchor_index:=0",
        #         "-p", "tag_name:=front",
        #         "--remap", "uwb/front/range:=/tb3_0/uwb/front/range",
        #     ],
        #     output="screen",
        # ),
        # ExecuteProcess(
        #     cmd=[
        #         "python3", ranger, "--ros-args",
        #         "-r", "__node:=uwb_range_tb3_0_back",
        #         "-p", ["serial_port:=", LaunchConfiguration("tb3_0_back_serial_port")],
        #         "-p", "anchor_index:=0",
        #         "-p", "tag_name:=back",
        #         "--remap", "uwb/back/range:=/tb3_0/uwb/back/range",
        #     ], output="screen"),
        # ExecuteProcess(
        #     cmd=["python3", ranger, "--ros-args",
        #         "-r", "__node:=uwb_range_tb3_1_front",
        #         "-p", ["serial_port:=", LaunchConfiguration("tb3_1_front_serial_port")],
        #         "-p", "anchor_index:=0", "-p", "tag_name:=front",
        #         "--remap", "uwb/front/range:=/tb3_1/uwb/front/range",
        #     ], output="screen"),
        # ExecuteProcess(
        #     cmd=["python3", ranger, "--ros-args",
        #         "-r", "__node:=uwb_range_tb3_1_back",
        #         "-p", ["serial_port:=", LaunchConfiguration("tb3_1_back_serial_port")],
        #         "-p", "anchor_index:=0", "-p", "tag_name:=back",
        #         "--remap", "uwb/back/range:=/tb3_1/uwb/back/range",
        #     ],
        #     output="screen",
        # ),

        ## 시뮬레이션
        ExecuteProcess(
            cmd=[
                "python3",
                mock_ranger,
                "--ros-args",
                "-p", ["robot_namespaces:=", robots],
                "-p", ["anchor_x:=", LaunchConfiguration("anchor_x")],
                "-p", ["anchor_y:=", LaunchConfiguration("anchor_y")],
                "-p", ["publish_rate_hz:=", LaunchConfiguration("publish_rate_hz")],
                "-p", ["noise_stddev_m:=", LaunchConfiguration("noise_stddev_m")],
                "-p", ["model_states_topic:=", LaunchConfiguration("model_states_topic")],
                "-p", ["initial_world_x:=", LaunchConfiguration("initial_world_x")],
                "-p", ["initial_world_y:=", LaunchConfiguration("initial_world_y")],
                "-p", ["initial_world_yaw:=", LaunchConfiguration("initial_world_yaw")],
                "-p", ["tag_offset_from_base_m:=", LaunchConfiguration("tag_offset_from_base_m")],
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_mock_uwb")),
        ),

        ExecuteProcess(
            cmd=[
                "python3", merger, "--ros-args",
                "-p", ["robot_namespaces:=", robots],
                "-p", ["tag_offset_from_base_m:=", LaunchConfiguration("tag_offset_from_base_m")],
            ],
            output="screen",
        ),
    ])
