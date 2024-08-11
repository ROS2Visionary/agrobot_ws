import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 定义静态变换发布器
    tf_base_link_to_liser_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_liser_link',
        output='screen',
        arguments=['0.25', '0.0', '0.06', '0.0', '0.0', '0.0', 'base_link', 'liser_link']
    )
    
    tf_base_footprint_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_base_link',
        output='screen',
        arguments=['0.0', '0.0', '0.05', '0.0', '0.0', '0.0', 'base_footprint', 'base_link']
    )
    
    tf_odom_to_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_odom',
        output='screen',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'odom', 'base_footprint']
    )
    
    tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map',
        output='screen',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom']
    )

    return LaunchDescription([
        tf_base_link_to_liser_link,
        tf_base_footprint_to_base_link,
        tf_odom_to_base_footprint,
        tf_map_to_odom,
    ])

