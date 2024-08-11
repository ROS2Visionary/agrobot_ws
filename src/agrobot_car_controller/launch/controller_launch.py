import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration



def generate_launch_description():
    # 获取包的共享目录路径
    pkg_share_dir = get_package_share_directory("agrobot_car_controller")
    # 设置配置文件路径
    twist_mux_config_path = os.path.join(pkg_share_dir, "config", "twist_mux.yaml")

    # 声明 use_sim_time 参数
    arg_sim = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulated time (true/false)"
    )

    # 定义 twist_mux 节点
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[{
            'config_file': twist_mux_config_path,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen',
    )

    # 返回启动描述
    return LaunchDescription([
        arg_sim,
        twist_mux_node
    ])


