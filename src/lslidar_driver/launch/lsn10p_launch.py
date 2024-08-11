#!/usr/bin/python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

import lifecycle_msgs.msg
import os

def generate_launch_description():

    driver_dir = os.path.join(get_package_share_directory('lslidar_driver'), 'params','lidar_uart_ros2', 'lsn10p.yaml')
                     
    driver_node = LifecycleNode(package='lslidar_driver',
                                executable='lslidar_driver_node',
                                name='lslidar_driver_node',		#设置激光数据topic名称
                                output='screen',
                                emulate_tty=True,
                                namespace='',
                                parameters=[driver_dir],
                                )
    
    tf_liser_linnk_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_liser',
        output='screen',
        arguments=[
            '0.3', '0.0', '0.045',  # translation (x, y, z)
            '0.0', '0.0', '0.0', '1.0',  # rotation (x, y, z, w)
            'base_link',  # frame_id
            'liser_link'  # child_frame_id
        ]   
    )

    tf_base_footprint_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_base_link',
        output='screen',
        arguments=[
            '0.0', '0.0', '0.045',  # translation (x, y, z)
            '0.0', '0.0', '0.0', '1.0',  # rotation (x, y, z, w)
            'base_footprint',  # frame_id
            'base_link'  # child_frame_id
        ]    
    )
    
    tf_odom_to_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_odom',
        output='screen',
        arguments=[
            '0.0', '0.0', '0.0',  # translation (x, y, z)
            '0.0', '0.0', '0.0', '1.0',  # rotation (x, y, z, w)
            'odom',  # frame_id
            'base_footprint'  # child_frame_id
        ] 
    )
    
    tf_odom_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map',
        output='screen',
        arguments=[
            '0.0', '0.0', '0.0',  # translation (x, y, z)
            '0.0', '0.0', '0.0', '1.0',  # rotation (x, y, z, w)
            'map',  # frame_id
            'odom'  # child_frame_id
        ]  
    )


    return LaunchDescription([
        tf_liser_linnk_to_base_link,
        tf_base_footprint_to_base_link,
        tf_odom_to_base_footprint,
        tf_odom_to_map,
        driver_node,
    ])

