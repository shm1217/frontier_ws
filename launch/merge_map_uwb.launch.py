#!/usr/bin/env python3
"""Start the separate UWB-assisted merger and gated global goal allocator.

SLAM and frontier_multi must already be running per robot.  Before registration,
frontier_multi uses its existing local-map fallback.  Do not run manual
world->tb3_x/map static publishers together with this launch.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    scripts = os.path.join(get_package_share_directory("frontier_ws"), "scripts")
    merger = os.path.join(scripts, "merge_map_uwb.py")
    gate = os.path.join(scripts, "gate_node_uwb.py")
    ranger = os.path.join(scripts, "uwb_range_node_uwb.py")
    mock_ranger = os.path.join(scripts, "mock_uwb_range_uwb.py")
    robots = LaunchConfiguration("robot_namespaces")

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_namespaces", default_value="['tb3_0','tb3_1']"),
        DeclareLaunchArgument(
            "tb3_0_serial_port", default_value="/dev/ttyUSB_tb3_0"),
        DeclareLaunchArgument(
            "tb3_1_serial_port", default_value="/dev/ttyUSB_tb3_1"),
        DeclareLaunchArgument("anchor_x", default_value="-3.0"),
        DeclareLaunchArgument("anchor_y", default_value="5.0"),
        DeclareLaunchArgument("publish_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("noise_stddev_m", default_value="0.0"),
        DeclareLaunchArgument(
            "model_states_topic", default_value="/gazebo/model_states"),
        DeclareLaunchArgument(
            "initial_world_x", default_value="[-1.0,-6.0]"), ## 시뮬레이션 상에서 uwb 센서 재현하기 위해 
        DeclareLaunchArgument(
            "initial_world_y", default_value="[4.0,2.0]"),
        DeclareLaunchArgument(
            "initial_world_yaw", default_value="[0.0,0.0]"),

        ## 하드웨어 
        # ExecuteProcess(
        #     cmd=[
        #         "python3", ranger, "--ros-args",
        #         "-r", "__node:=uwb_range_tb3_0_uwb",
        #         "-p", ["serial_port:=", LaunchConfiguration("tb3_0_serial_port")],
        #         "-p", "anchor_index:=0",
        #         "--remap", "uwb/range:=/tb3_0/uwb/range",
        #     ],
        #     output="screen",
        # ),
        # ExecuteProcess(
        #     cmd=[
        #         "python3", ranger, "--ros-args",
        #         "-r", "__node:=uwb_range_tb3_1_uwb",
        #         "-p", ["serial_port:=", LaunchConfiguration("tb3_1_serial_port")],
        #         "-p", "anchor_index:=0",
        #         "--remap", "uwb/range:=/tb3_1/uwb/range",
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
            ],
            output="screen",
        ),

        ExecuteProcess(
            cmd=[
                "python3", merger, "--ros-args",
                "-p", ["robot_namespaces:=", robots],
            ],
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                "python3", gate, "--ros-args",
                "-p", ["robot_namespaces:=", robots],
                "-p", "merge_map_topic:=/merge_map",
                "-p", "global_frame:=world",
            ],
            output="screen",
        ),
    ])
