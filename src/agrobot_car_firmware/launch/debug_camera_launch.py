import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():


    camera_node = Node(
        package="agrobot_car_firmware",
        executable="camera_comm",
        name="debug_camera_node"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz_node"
    )

    return LaunchDescription([
        camera_node,
        rviz_node,
    ])