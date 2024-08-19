

import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
import math


def generate_launch_description():
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    arg_sim = DeclareLaunchArgument(
        'use_sim_time',
        default_value="False"
    )
    
    nav2_param_path = os.path.join(get_package_share_directory("agrobot_car_navigation"),"config","navigation_params.yaml")
    nav_launch =  IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('nav2_bringup'), '/launch', '/navigation_launch.py']),
            # 使用 Launch 参数替换原有参数
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path,
                }.items(),
    )


    map_path = os.path.join(get_package_share_directory("agrobot_car_mapping"), "maps", "laboratory.yaml")
    # 定义地图服务器节点，用于提供已知地图的服务
    nav2_map_server_node = Node(
        package="nav2_map_server",  # 节点所在的功能包
        executable="map_server",  # 要执行的可执行文件名
        name="nav2_map_server_node",  # 节点的名称
        output="screen",  # 将节点的输出打印到屏幕
        parameters=[
            {"yaml_filename": map_path},  # 地图文件的路径
            {"use_sim_time": use_sim_time},  # 使用仿真时间
        ]
    )

    # 定义需要管理生命周期的节点列表，包括AMCL节点和地图服务器节点
    lifecycle_nodes = ["nav2_map_server_node"]
    # 定义生命周期管理器节点，用于管理其他节点的生命周期
    nav2_lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",  # 节点所在的功能包
        executable="lifecycle_manager",  # 要执行的可执行文件名
        name="lifecycle_manager",  # 节点的名称
        parameters=[
            {"node_names": lifecycle_nodes},  # 需要管理的节点列表
            {"use_sim_time": use_sim_time},  # 使用仿真时间
            {"autostart": True}  # 自动启动管理的节点
        ],
    )
    
    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("agrobot_car_bringup"),"/launch","/static_tf_launch.py"])
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_node111"
    )

    # 导入Keepout过滤器的启动文件，并设置其mask_yaml_file参数
    keepout_filter_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("agrobot_car_navigation"),"/launch","/keepout_filter_launch.py"]),
        launch_arguments={
            "mask_yaml_file":os.path.join(get_package_share_directory("agrobot_car_mapping"),"maps","laboratory_mask.yaml")  # Keepout过滤器的遮罩文件路径
        }.items()
    )

    return launch.LaunchDescription([arg_sim,
                                     rviz2_node,
                                     keepout_filter_node,
                                     static_tf_launch,
                                     nav2_map_server_node,
                                     nav_launch,
                                     nav2_lifecycle_manager_node,
                                     ])
