import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = True
    ns = 'tb3_0' 

    cartographer_config_dir = os.path.join(get_package_share_directory('frontier_ws'), 'lua')
    lua_file = "tb3_0_turtlebot3_lds_2d.lua"

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
            ('scan', '/scan'),
            ('odom', '/odom'),
            ('imu', '/imu'),
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
    # ✅ 가제보 로봇은 네임스페이스가 없으므로 base_footprint와 base_scan을 바로 연결
    # static_tf_node = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="base_to_scan",
    #     arguments=[
    #         "0", "0", "0.2", "0", "0", "0",
    #         "base_footprint", "base_scan"
    #     ]
    # )

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
            "map_topic": "map",        
            "scan_topic": "/scan",     
            "cmd_topic": "/cmd_vel",   # ✅ 전역 cmd_vel 사용
            "map_frame": f"{ns}/map",
            "base_frame": "base_footprint",
            "global_frame": f"{ns}/map",
        }]
    )

    return LaunchDescription([
        cartographer_node,
        occupancy_grid_node,

        frontier_node
    ])