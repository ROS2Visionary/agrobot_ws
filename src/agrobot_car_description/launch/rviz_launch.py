
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
import xacro

def generate_launch_description():
    package_name = 'agrobot_car_description'
    urdf_name = "robot_display.urdf"

    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')

    # 加载xacro文件(机器人描述文件)
    robot_description_doc = xacro.process_file(urdf_model_path)
    robot_description_content = robot_description_doc.toprettyxml(indent='  ')

    use_sim_time = LaunchConfiguration("use_sim_time")
    arg_sim = DeclareLaunchArgument(
        'use_sim_time',
        default_value="false"
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="screen",
        parameters=[{
            "robot_description": robot_description_content,
            "use_sim_time":use_sim_time,
        }],
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output="screen",
        # arguments=["-d", os.path.join(pkg_share, "rviz", "display.rviz")],
    )

    return LaunchDescription([robot_state_publisher_node,
                              joint_state_publisher_node,
                              arg_sim,
                              rviz2_node])

