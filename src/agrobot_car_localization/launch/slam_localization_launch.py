import os
from launch import LaunchDescription  # 用于创建Launch文件的描述
from ament_index_python.packages import get_package_share_directory  # 用于获取ROS 2包的共享目录
from launch_ros.actions import Node, LifecycleNode  # 用于启动ROS 2节点和生命周期节点
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription  # 用于声明launch参数、定时器和包含其他Launch文件
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution  # 用于处理launch文件中的动态参数和路径拼接
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 用于加载其他Launch文件
from nav2_common.launch import RewrittenYaml  # 用于动态重写YAML配置文件中的参数

def generate_launch_description():

    # 创建一个LaunchConfiguration对象，用于获取传递给launch文件的参数'use_sim_time'
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 声明一个LaunchArgument，用于在运行时接受参数'use_sim_time'
    arg_sim = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='是否使用模拟时间'  # 该参数用于决定是否使用模拟时间
    )

    # * 不会修改地图，只会临时在地图上显示激光雷达扫到的新的障碍物，激光雷达扫描过后将会清除新的障碍物痕迹
    slam_config = os.path.join(get_package_share_directory("agrobot_car_localization"), "config", "temp.yaml") 
    # slam_config = os.path.join(get_package_share_directory("agrobot_car_localization"), "config", "mapper_params_localization.yaml") 
    # * 会修改地图，会在地图上显示激光雷达扫到的新的障碍物，激光雷达扫描过后不会清除新的障碍物痕迹
    # slam_config = os.path.join(get_package_share_directory("agrobot_car_localization"), "config", "mapper_params_online_async.yaml")

    # 包含SLAM工具箱的启动文件，用于启动SLAM节点
    localization_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("slam_toolbox"), "/launch", "/localization_launch.py"]),
        # PythonLaunchDescriptionSource([get_package_share_directory("slam_toolbox"), "/launch", "/online_async_launch.py"]),
        launch_arguments={
            "use_sim_time": use_sim_time,  # 传递是否使用仿真时间的参数给SLAM节点
            "slam_params_file": slam_config  # 传递SLAM配置文件的路径给SLAM节点
        }.items()  # 将参数以键值对的形式传递给包含的Launch文件
    )

    # 创建并返回LaunchDescription对象，包含启动所需的所有节点和参数声明
    return LaunchDescription([
        arg_sim,  # 声明使用仿真时间的参数
        localization_node  # 启动SLAM节点
    ])
