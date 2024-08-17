import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node,LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction

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
        emulate_tty=True,  # 模拟TTY以保留输出格式和颜色
    )

    # 多点导航
    multi_point_nav_node = Node(
        package="agrobot_car_navigation",
        executable="multi_point_nav",
        output="screen",
        emulate_tty=True,  # 模拟TTY以保留输出格式和颜色
    )
 
    
    nav_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("agrobot_car_navigation"),"/launch","/nav2_with_slam_launch.py"]),
    )


    static_tf_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("agrobot_car_bringup"),"/launch","/static_tf_launch.py"]),
    )

    lidar_data_relay_node = Node(
        package="agrobot_car_mapping",  # 指定节点所在的包名
        executable="lidar_data_relay",  # 指定要执行的可执行文件
        name="lidar_data_relay_node",  # 为节点指定一个名称
        output="screen",  # 输出信息到屏幕
    )

    return LaunchDescription([
            # 错开节点启动的时间间隔
            firmware_node,liser_node,
            static_tf_node,
            lidar_data_relay_node,
            TimerAction(
                period=10.0,
                actions=[nav_server_launch]
            )
            # TimerAction(
            #     period=60.0,
            #     actions=[multi_point_nav_node]
            # )
            ])
    
    
    