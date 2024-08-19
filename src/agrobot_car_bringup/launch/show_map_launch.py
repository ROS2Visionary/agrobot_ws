import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node,LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    

    # 获取"agrobot_mapping"包中已保存地图文件的路径
    map_path = os.path.join(get_package_share_directory("agrobot_car_mapping"), "maps", "laboratory.yaml")
    
    
    # 定义一个launch参数"use_sim_time"，用于决定是否使用仿真时间
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    # 声明"use_sim_time"参数，默认值为"True"，表示使用仿真时间
    arg_sim = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false"
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

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz_node"
    )
    
    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("agrobot_car_bringup"),"/launch","/static_tf_launch.py"])
    )

    # 返回一个LaunchDescription对象，其中包含了要启动的所有launch动作
    return LaunchDescription([arg_sim,
                              static_tf_launch,
                              nav2_map_server_node,
                              rviz_node,
                              nav2_lifecycle_manager_node])
