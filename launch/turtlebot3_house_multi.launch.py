#!/usr/bin/env python3
import os
import re
import tempfile

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_robots(yaml_path):
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)

    robots = [r for r in data["robots"] if r.get("enabled", True)]
    if len(robots) < 2:
        raise RuntimeError("robots.yaml needs at least two enabled robots.")
    return robots[:2]


def upsert_tag(body, tag, value):
    pat = rf"<{tag}>\s*.*?\s*</{tag}>"
    if re.search(pat, body, flags=re.DOTALL):
        return re.sub(pat, f"<{tag}>{value}</{tag}>", body, flags=re.DOTALL)
    return body + f"\n    <{tag}>{value}</{tag}>"


def patch_sdf_for_robot(sdf_path, name):
    with open(sdf_path, "r") as f:
        sdf = f.read()

    diff_pat = r'(<plugin\s+name="turtlebot3_diff_drive"\s+filename="libgazebo_ros_diff_drive\.so"\s*>)(.*?)(</plugin>)'
    match = re.search(diff_pat, sdf, flags=re.DOTALL)
    if not match:
        raise RuntimeError("Could not find turtlebot3_diff_drive plugin in model.sdf.")

    open_tag, body, close_tag = match.group(1), match.group(2), match.group(3)
    body = upsert_tag(body, "odometry_frame", f"{name}/odom")
    body = upsert_tag(body, "robot_base_frame", f"{name}/base_footprint")
    body = upsert_tag(body, "publish_odom_tf", "true")
    body = upsert_tag(body, "odometryFrame", f"{name}/odom")
    body = upsert_tag(body, "robotBaseFrame", f"{name}/base_footprint")
    sdf = sdf[:match.start()] + open_tag + body + close_tag + sdf[match.end():]

    ray_pat = r'(<plugin\s+name="turtlebot3_laserscan"\s+filename="libgazebo_ros_ray_sensor\.so"\s*>)(.*?)(</plugin>)'
    match = re.search(ray_pat, sdf, flags=re.DOTALL)
    if not match:
        raise RuntimeError("Could not find turtlebot3_laserscan plugin in model.sdf.")

    open_tag, body, close_tag = match.group(1), match.group(2), match.group(3)
    body = upsert_tag(body, "frame_name", f"{name}/base_scan")
    body = upsert_tag(body, "frameName", f"{name}/base_scan")
    sdf = sdf[:match.start()] + open_tag + body + close_tag + sdf[match.end():]

    out_path = os.path.join(tempfile.gettempdir(), f"{name}_tb3_patched.sdf")
    with open(out_path, "w") as f:
        f.write(sdf)
    return out_path


def first_existing(paths):
    for path in paths:
        if path and os.path.exists(path):
            return path
    return paths[0]


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    pkg_tb3 = get_package_share_directory("turtlebot3_gazebo")
    prefix_tb3 = get_package_prefix("turtlebot3_gazebo")
    pkg_this = get_package_share_directory("frontier_ws")

    gui = LaunchConfiguration("gui", default="true")
    world_arg = LaunchConfiguration("world")

    # TODO: 로봇 스폰 위치 조정 가능
    robots = load_robots(os.path.join(pkg_this, "config", "robots.yaml"))
    # TODO: 경로 설정
    default_world = first_existing([
    "/home/hdzggg/turtlebot3_simulations/install/turtlebot3_gazebo/share/turtlebot3_gazebo/worlds/turtlebot3_house.world",
    "/home/hdzggg/turtlebot3_simulations/turtlebot3_gazebo/worlds/turtlebot3_house.world",
    ])
    # default_world = first_existing([
    #     os.path.join(pkg_tb3, "worlds", "turtlebot3_random2.world"),
    #     os.path.join(prefix_tb3, "share", "turtlebot3_gazebo", "worlds", "turtlebot3_random2.world"),
    #     "/home/hdzggg/turtlebot3_simulations/install/turtlebot3_gazebo/share/turtlebot3_gazebo/worlds/turtlebot3_random2.world",
    #     "/home/hdzggg/turtlebot3_simulations/turtlebot3_gazebo/worlds/turtlebot3_random2.world",
    #     os.path.join(pkg_tb3, "worlds", "turtlebot3_house.world"),
    # ])

    tb3_model = os.environ.get("TURTLEBOT3_MODEL", "burger")
    sdf_path = os.path.join(pkg_tb3, "models", f"turtlebot3_{tb3_model}", "model.sdf")

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("gui", default_value="true"))
    ld.add_action(DeclareLaunchArgument("world", default_value=default_world))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")),
        launch_arguments={
            "world": world_arg,
            "pause": "true",
            "verbose": "true",
            "init": "true",
            "factory": "true",
            "force_system": "true",
        }.items(),
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")),
        condition=IfCondition(gui),
    ))

    delay = 8.0
    for robot in robots:
        name = robot["name"]
        patched_sdf = patch_sdf_for_robot(sdf_path, name)

        spawn = Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name=f"spawn_{name}",
            output="screen",
            arguments=[
                "-entity", name,
                "-file", patched_sdf,
                "-robot_namespace", f"/{name}",
                "-timeout", "120.0",
                "-x", str(robot.get("x", 0.0)),
                "-y", str(robot.get("y", 0.0)),
                "-z", str(robot.get("z", 0.0)),
                "-Y", str(robot.get("yaw", 0.0)),
            ],
        )
        ld.add_action(TimerAction(period=delay, actions=[spawn]))
        delay += 1.5

    ld.add_action(TimerAction(
        period=delay + 1.0,
        actions=[ExecuteProcess(
            cmd=["ros2", "service", "call", "/unpause_physics", "std_srvs/srv/Empty", "{}"],
            output="screen",
        )],
    ))

    return ld