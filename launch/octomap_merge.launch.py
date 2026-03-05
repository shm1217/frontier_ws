#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory 
from launch.actions import ExecuteProcess 
import os

def generate_launch_description():

    cloud_merge = ExecuteProcess( 
        cmd=['ros2', 'run', 'frontier_ws', 'cloud_merge_to_octomap.py', 
             '--ros-args', '-p', 
             'global_frame:=tb3_0/map', '-p', 
             'output_topic:=/merged_cloud', '-p', 
             'publish_rate_hz:=5.0', '-p', 
             'max_points:=60000', '-p', 
             'tf_timeout_sec:=0.15', 
             '-p', 
             'use_sim_time:=True'], 
             output='screen' )  
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([cloud_merge, rviz])
