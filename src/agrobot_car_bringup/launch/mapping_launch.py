import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
from launch_ros.actions import LifecycleNode
from launch.actions import GroupAction, TimerAction, OpaqueFunction, LogInfo, Shutdown


def generate_launch_description():

    # 雷达节点
    liser_params_path = os.path.join(get_package_share_directory('lslidar_driver'), 'params','lidar_uart_ros2', 'lsn10p.yaml')          
    liser_node = LifecycleNode(package='lslidar_driver',
                                executable='lslidar_driver_node',
                                name='lslidar_driver_node',		
                                output='screen',
                                emulate_tty=True,
                                namespace='',
                                parameters=[
                                    liser_params_path
                                ]
                                )


    # 电机控制(里程计在这个节点中发布)
    firmware_node = Node(
        package="agrobot_car_firmware",
        executable="serial_comm",
        output="screen",
    )

    # 建图可选方式，（slam、cartographer麻烦的地方在于参数的设置，建图的质量与参数相关，不正确的参数设置，可能会导致建图失败）
    mapping_launch_name = "/slam_toolbox_mapping_launch.py" # slam建图
    # mapping_launch_name = "/cartographer_mapping_launch.py" # cartographer建图
    # mapping_launch_name = "/custom_mapping_launch.py" # 自定义建图，仅为演示2D建图的流程，该建图方式仅供学习
    
    mapping_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("agrobot_car_mapping"),"/launch",mapping_launch_name]),
    )

    static_tf_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("agrobot_car_bringup"),"/launch","/static_tf_launch.py"]),
    )

    # 一、默认情况下launch的节点是并行启动，由于上位机(比如：树莓派)性能的原因，如果所有节点一起并行启动：
    # 1、启动时间过长：由于资源有限，同时启动多个节点可能会导致某些节点的启动时间过长。
    # 2、启动失败：资源争用可能导致某些节点无法成功启动。
    # 3、时间不同步：节点启动顺序不确定可能导致时间同步问题，尤其是在需要依赖其他节点的数据时（如传感器数据、TF变换）。
    # 二、TF节点必须优先启动
    return LaunchDescription([
            # 错开节点启动的时间间隔
            firmware_node,liser_node,
            static_tf_node,
            TimerAction(
                period=10.0,  
                actions=[mapping_node]
            )
        ])

