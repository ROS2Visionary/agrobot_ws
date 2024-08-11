import os
import launch
from launch.substitutions import LaunchConfiguration  # 用于处理launch文件中的动态参数
from launch import LaunchDescription  # 用于定义launch文件的描述
from launch.actions import DeclareLaunchArgument  # 用于声明launch参数
from ament_index_python.packages import get_package_share_directory  # 用于获取ROS 2包的共享目录
from launch_ros.actions import Node  # 用于启动ROS 2节点
from launch_ros.substitutions import FindPackageShare  # 用于找到功能包的共享目录（此处未使用）

def generate_launch_description():
    
    # 获取并处理launch文件中传入的参数"use_sim_time"，这个参数决定是否使用仿真时间
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    # 声明launch文件参数"use_sim_time"
    arg_sim = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false"
    )
    
    # 定义mapping节点，这个节点执行地图构建功能
    mapping_node = Node(
        package="agrobot_car_mapping",  # 指定节点所在的功能包
        executable="mapping",  # 要执行的可执行文件名
        name="mapping",  # 节点的名称
        parameters=[{'use_sim_time': use_sim_time}],  # 将"use_sim_time"参数传递给节点
    )
    
    # 定义地图保存节点，这个节点会周期性地保存生成的地图
    map_saver_node = Node(
        package="agrobot_car_mapping",  # 指定节点所在的功能包
        executable="map_saver",  # 要执行的可执行文件名
        name="custom_map_saver",  # 节点的名称
        output="screen",  # 将节点的输出打印到屏幕
        parameters=[
            {"time_interval": 10.0},  # 设置地图保存的时间间隔，单位为秒
            {"save_dir_path": "/root/agrobot_ws/src/agrobot_car_mapping/maps_sw"},  # 设置保存地图的目录路径，确保路径存在
        ]
    )
    
    # 返回一个LaunchDescription对象，其中包含了要启动的所有launch动作
    return LaunchDescription([arg_sim, mapping_node, map_saver_node])
