import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = False
    ns = 'tb3_1' 

    cartographer_config_dir = os.path.join(get_package_share_directory('frontier_ws'), 'lua')
    lua_file = "tb3_1_turtlebot3_lds_2d.lua"

    # A. 메인 카토그래퍼 노드
    cartographer_node = Node(
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
            # ✅ 가제보 전역 토픽(/scan, /odom)을 tb3_0 네임스페이스 안으로 끌어옴
            ('scan', f'/{ns}/scan'),  # /tb3_0/scan 구조로 매핑
            ('odom', f'/{ns}/odom'),  # /tb3_0/odom 구조로 매핑
            ('imu', f'/{ns}/imu'),
        ]
    )
    
    # B. Occupancy Grid Node
    occupancy_grid_node = Node(
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
            ('/map', f'/{ns}/map'),
        ]
    )

    # 2. Static TF Publisher
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_scan",
        arguments=[
            "0", "0", "0.2", "0", "0", "0",
            "tb3_1/base_footprint", "tb3_1/base_scan"
        ]
    )

    world_to_map_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_tb3_0_map",
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", "0",
            "--yaw", "0", "--pitch", "0", "--roll", "0",
            "--frame-id", "world",
            "--child-frame-id", "tb3_1/map",
        ],
    )
    # 이것도 return LaunchDescription([ ... ]) 안에 넣어줍니다.

    # 3. Frontier Multi Node
    frontier_node = Node(
        package="frontier_ws",
        executable="frontier_multi",
        name="frontier_multi",
        namespace=ns,
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_id": ns,
            "map_topic": f"/{ns}/map",        
            "scan_topic": f"/{ns}/scan",     
            "cmd_topic": f"/{ns}/cmd_vel",   
            
            "map_frame": f"{ns}/map",
            "base_frame": f"{ns}/base_footprint", 
            "global_frame": "world",
        }]
    )

    return LaunchDescription([
        static_tf_node,
        world_to_map_node,
        cartographer_node,
        occupancy_grid_node,

        frontier_node
    ])