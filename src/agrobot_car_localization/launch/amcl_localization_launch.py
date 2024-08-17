import os
import launch
from launch import LaunchDescription  # 用于定义launch文件的描述
from launch.actions import DeclareLaunchArgument  # 用于声明launch参数
from ament_index_python.packages import get_package_share_directory  # 用于获取ROS 2包的共享目录
from launch_ros.actions import Node  # 用于启动ROS 2节点
from launch_ros.substitutions import FindPackageShare  # 用于找到功能包的共享目录（此处未使用）
from launch.substitutions import LaunchConfiguration  # 用于处理launch文件中的动态参数


def generate_launch_description():
    
    # 获取"agrobot_localization"包的共享目录路径
    pkg_share_dir = get_package_share_directory("agrobot_car_localization")
    
    # 构建AMCL（Adaptive Monte Carlo Localization）配置文件的完整路径
    amcl_config_path = os.path.join(pkg_share_dir, "config", "amcl.yaml")
    
    # 获取"agrobot_mapping"包中已保存地图文件的路径
    map_path = os.path.join(get_package_share_directory("agrobot_car_mapping"), "maps", "laboratory.yaml")
    
    
    # 定义一个launch参数"use_sim_time"，用于决定是否使用仿真时间
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    # 声明"use_sim_time"参数，默认值为"True"，表示使用仿真时间
    arg_sim = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false"
    )
    
    # 定义一个launch参数"amcl_config"，用于指定AMCL的配置文件路径
    amcl_config = LaunchConfiguration("amcl_config")
    
    # 声明"amcl_config"参数，默认值为前面定义的AMCL配置文件路径
    arg_amcl_config = DeclareLaunchArgument(
        "amcl_config",
        default_value=amcl_config_path
    )

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

    # 定义AMCL节点，主要用于在已知地图上进行机器人定位
    nav2_amcl_node = Node(
        package="nav2_amcl",  # 节点所在的功能包
        executable="amcl",  # 要执行的可执行文件名
        name="nav2_amcl_node",  # 节点的名称
        output="screen",  # 将节点的输出打印到屏幕
        emulate_tty=True,  # 模拟TTY，以确保输出格式正确
        parameters=[
            amcl_config,  # 使用AMCL配置文件
            {"use_sim_time": use_sim_time},  # 使用仿真时间
        ],
    )

    # 定义需要管理生命周期的节点列表，包括AMCL节点和地图服务器节点
    lifecycle_nodes = ["nav2_amcl_node", "nav2_map_server_node"]
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
    
    # 返回一个LaunchDescription对象，其中包含了要启动的所有launch动作
    return LaunchDescription([arg_sim,
                              arg_amcl_config,
                              nav2_map_server_node,
                              nav2_amcl_node,
                              nav2_lifecycle_manager_node])
