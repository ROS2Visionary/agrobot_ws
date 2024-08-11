import os
import launch
from launch import LaunchDescription  
from launch.actions import DeclareLaunchArgument,TimerAction  
from ament_index_python.packages import get_package_share_directory  
from launch_ros.actions import Node  
from launch_ros.substitutions import FindPackageShare  
from launch.substitutions import LaunchConfiguration  

def generate_launch_description():
    
    # 定位到功能包的共享目录，用于找到配置文件等资源
    pkg_share = get_package_share_directory("agrobot_car_mapping")
    
    # 声明一个LaunchConfiguration对象，用于获取传递给launch文件的参数'use_sim_time'
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    # 声明launch参数'use_sim_time'，允许用户在运行时传入是否使用模拟时间
    arg_sim = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false" 
    )
    
    # 定义其他LaunchConfiguration对象，用于获取配置文件中的分辨率和发布周期
    resolution = LaunchConfiguration('resolution', default='0.05')  # 地图的分辨率，默认为0.05米/像素
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='2.0')  # 地图的发布周期，默认为2秒
    configuration_directory = LaunchConfiguration('configuration_directory', default=os.path.join(pkg_share, 'config'))  # 配置文件夹路径，默认为功能包的'config'文件夹
    configuration_basename = LaunchConfiguration('configuration_basename', default='cartographer_2d.lua')  # 配置文件名，默认为'cartographer_2d.lua'
  
    # 创建Cartographer节点，用于运行SLAM算法
    cartographer_node = Node(
        package='cartographer_ros',  # 该节点所在的功能包
        executable='cartographer_node',  # 要执行的可执行文件
        name='cartographer_node',  # 节点的名称
        output='screen',  # 将节点输出打印到屏幕
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename],  # 传递配置文件的路径和文件名作为参数
        # 可选的重映射配置，用于重映射传感器话题
        # remappings=[
                # ('/imu', '/imu/data'),  # 重映射IMU话题
                # ('/scan', '/scan/data'),  # 重映射激光雷达话题
                # ('/odom', '/odom/data')  # 重映射里程计话题
            # ]
        parameters=[{"use_sim_time":use_sim_time}]  # 使用传递的'sim_time'参数，决定是否使用模拟时间
    )

    # 创建OccupancyGrid节点，用于将SLAM生成的地图转换为占据栅格地图
    occupancy_grid_node = Node(
        package='cartographer_ros',  # 该节点所在的功能包
        executable='cartographer_occupancy_grid_node',  # 要执行的可执行文件
        name='occupancy_grid_node',  # 节点的名称
        output='screen',  # 将节点输出打印到屏幕
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec],  # 传递分辨率和发布周期作为参数
        parameters=[{"use_sim_time":use_sim_time}]  # 使用传递的'sim_time'参数，决定是否使用模拟时间
    )
    
    # 创建自定义的Map Saver节点，用于定期保存生成的地图
    map_saver_node = Node(
        package="agrobot_car_mapping",  # 该节点所在的功能包
        executable="map_saver",  # 要执行的可执行文件
        name="custom_map_saver",  # 节点的名称
        output="screen",  # 将节点输出打印到屏幕
        parameters=[
            {"time_interval":10.0},  # 设置地图保存的时间间隔，单位为秒
            {"save_dir_path":"/root/agrobot_ws/src/agrobot_car_mapping/maps_sw"}  # 设置保存地图的目录路径，确保路径存在
        ]
    )
    
    # 返回LaunchDescription对象，其中包含了要启动的所有launch动作
    return LaunchDescription([
        arg_sim, 
        cartographer_node, 
        occupancy_grid_node, 
        TimerAction(
            period=120.0,
            actions=[map_saver_node]
        )
    ])
