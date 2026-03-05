#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # map → tb3_0/odom
    tf_tb3_0 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_tb3_0_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'tb3_0/odom'],
        output='screen'
    )

    # map → tb3_1/odom
    tf_tb3_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_tb3_1_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'tb3_1/odom'],
        output='screen'
    )

    return LaunchDescription([
        tf_tb3_0,
        tf_tb3_1,
    ])
