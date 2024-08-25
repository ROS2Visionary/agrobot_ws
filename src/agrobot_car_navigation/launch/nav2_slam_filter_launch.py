import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument,TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
import math


def generate_launch_description():
    
    # 获取功能包的共享目录路径
    pkg_share_dir = get_package_share_directory("agrobot_car_navigation")
    slam_toolbox_pkg_share_dir = get_package_share_directory("slam_toolbox")
        
    # 指定SLAM工具箱和导航的参数文件路径
    slam_params_file = os.path.join(pkg_share_dir,"config","mapper_params_localization.yaml")
    nav2_param_path = os.path.join(pkg_share_dir,"config","nav2_keepout_filter_params.yaml")
        
    # 创建一个参数LaunchConfiguration对象，允许在运行时设置使用仿真时间的配置
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    # 声明一个启动参数，用于设置是否使用仿真时间
    arg_sim = DeclareLaunchArgument(
        'use_sim_time',
        default_value="false"  # 默认使用仿真时间
    )
    
    # 导入Nav2的启动文件，并通过Launch参数替换其内部参数
    nav_launch =  IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('nav2_bringup'), '/launch', '/navigation_launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,  # 使用仿真时间
                'params_file': nav2_param_path,  # 导航参数文件的路径
                }.items(),
    )
    
    # 导入SLAM工具箱的启动文件，并通过Launch参数替换其内部参数
    slam_launch =  IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [slam_toolbox_pkg_share_dir, '/launch', '/localization_launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,  # 使用仿真时间
                'slam_params_file': slam_params_file,  # SLAM工具箱参数文件的路径
                }.items(),
    )

    # 导入Keepout过滤器的启动文件，并设置其mask_yaml_file参数
    keepout_filter_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("agrobot_car_navigation"),"/launch","/keepout_filter_launch.py"]),
        launch_arguments={
            "mask_yaml_file":os.path.join(get_package_share_directory("agrobot_car_mapping"),"maps","laboratory_mask.yaml")  # Keepout过滤器的遮罩文件路径
        }.items()
    )
    
    # 返回包含所有启动动作的LaunchDescription对象，启动时会依次执行这些动作
    return launch.LaunchDescription([arg_sim,  # 添加仿真时间参数
                                     slam_launch,  # 启动SLAM工具箱
                                     keepout_filter_node,  # 启动Keepout过滤器
                                     TimerAction(
                                         period=5.0,
                                         actions=[
                                            nav_launch,  # 启动Nav2导航
                                         ]
                                     )
                                     ])
