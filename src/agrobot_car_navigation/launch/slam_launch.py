

import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    
    pkg_share_dir = get_package_share_directory("agrobot_car_navigation")

    bringup_pkg_share_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_pkg_share_dir = get_package_share_directory("slam_toolbox")
        
    slam_params_file = os.path.join(pkg_share_dir,"config","mapper_params_online_async.yaml")
    nav2_param_path = os.path.join(pkg_share_dir,"config","navigation_params.yaml")
        
    use_sim_time = LaunchConfiguration("use_sim_time")
    arg_sim = DeclareLaunchArgument(
        'use_sim_time',
        default_value="false"
    )
    
    
    nav_launch =  IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [bringup_pkg_share_dir, '/launch', '/navigation_launch.py']),
            # 使用 Launch 参数替换原有参数
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path,
                }.items(),
    )
    
    slam_launch =  IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [slam_toolbox_pkg_share_dir, '/launch', '/online_async_launch.py']),
            # 使用 Launch 参数替换原有参数
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': slam_params_file,
                }.items(),
    )

    
    return launch.LaunchDescription([arg_sim,
                                     nav_launch,
                                     slam_launch,
                                     ])
