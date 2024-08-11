import os
from launch import LaunchDescription  # 导入LaunchDescription类，用于定义launch文件的描述
from ament_index_python.packages import get_package_share_directory  # 用于获取ROS 2包的共享目录
from launch_ros.actions import Node, LifecycleNode  # 导入Node和LifecycleNode类，用于启动ROS 2节点
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription  # 导入声明参数、定时器动作和包含其他launch文件的类
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution  # 导入LaunchConfiguration和PathJoinSubstitution类，用于参数替换
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 导入PythonLaunchDescriptionSource类，用于加载其他launch文件


def generate_launch_description():
    # 创建一个LaunchConfiguration对象，用于获取传递给launch文件的参数'use_sim_time'
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 声明一个LaunchArgument，用于在运行时接受参数'use_sim_time'
    arg_sim = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='是否使用模拟时间'  # 该参数用于决定是否使用模拟时间
    )

    # 获取slam_toolbox参数配置文件的路径
    slam_config = os.path.join(get_package_share_directory("agrobot_car_mapping"), "config", "mapper_params_online_async.yaml")

    # IncludeLaunchDescription用于包含另一个launch文件，PythonLaunchDescriptionSource指定包含的文件为slam_toolbox的online_async_launch.py
    async_slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("slam_toolbox"), "/launch", "/online_async_launch.py"]),
        launch_arguments={
            "use_sim_time": use_sim_time,  # 传递给slam_toolbox的参数，决定是否使用模拟时间
            "slam_params_file": slam_config  # 传递slam_toolbox的配置文件路径
        }.items()  # items()方法用于将字典转化为元组列表，以供launch_arguments参数使用
    )

    lidar_data_relay_node = Node(
        package="agrobot_car_mapping",  # 指定节点所在的包名
        executable="lidar_data_relay",  # 指定要执行的可执行文件
        name="lidar_data_relay_node",  # 为节点指定一个名称
        output="screen",  # 输出信息到屏幕
    )

    # 定义一个Node动作，用于启动自定义的sequence_map_saver节点
    sequence_map_saver_node = Node(
        package="agrobot_car_mapping",  # 指定节点所在的包名
        executable="sequence_map_saver",  # 指定要执行的可执行文件
        name="custom_sequence_map_saver",  # 为节点指定一个名称
        output="screen",  # 输出信息到屏幕
        parameters=[
            {"time_interval":10.0},  # 地图保存间隔，单位为秒
            {"save_dir_path":"/root/agrobot_ws/src/agrobot_car_mapping/sequence_maps_sw"},  # 保存地图的目录路径，确保该路径存在
        ]
    )

    # 定义另一个Node动作，用于启动自定义的map_saver节点
    map_saver_node = Node(
        package="agrobot_car_mapping",  # 指定节点所在的包名
        executable="map_saver",  # 指定要执行的可执行文件
        name="custom_map_saver",  # 为节点指定一个名称
        output="screen",  # 输出信息到屏幕
        parameters=[
            {"time_interval":10.0},  # 地图保存间隔，单位为秒
            {"save_dir_path":"/root/agrobot_ws/src/agrobot_car_mapping/maps_sw"},  # 保存地图的目录路径，确保该路径存在
        ]
    )

    # 返回一个LaunchDescription对象，其中包含了要执行的所有launch动作
    return LaunchDescription([
        arg_sim,  # 首先声明的参数
        lidar_data_relay_node,
        async_slam_toolbox,  # 然后包含slam_toolbox的launch文件
        TimerAction(
            period=40.0,  # 指定延迟
            actions=[map_saver_node, sequence_map_saver_node]  # 定时启动两个地图保存节点
        )
    ])
