import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node,LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    arg_sim = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false"
    )

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

    lidar_data_relay_node = Node(
        package="agrobot_car_mapping",  # 指定节点所在的包名
        executable="lidar_data_relay",  # 指定要执行的可执行文件
        name="lidar_data_relay_node",  # 为节点指定一个名称
        output="screen",  # 输出信息到屏幕
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("agrobot_car_navigation"),"/launch","/nav2_slam_filter_launch.py"]),
        launch_arguments={
            "use_sim_time":use_sim_time,
            }.items()
    )

    
    tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("agrobot_car_bringup"),"/launch","/static_tf_launch.py"]),
    )

    return LaunchDescription([
        arg_sim,
        tf_launch,
        liser_node,
        firmware_node,
        lidar_data_relay_node,
        TimerAction(
            period=15.0,
            actions=[nav_launch]
        )
    ])
    
    
    