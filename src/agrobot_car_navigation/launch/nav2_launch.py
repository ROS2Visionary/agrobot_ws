import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    pkg_share_dir = get_package_share_directory("agrobot_car_navigation")

    nav2_pkg_share_dir = get_package_share_directory('nav2_bringup')
    mapping_pkg_share_dir = get_package_share_directory("agrobot_car_mapping")
    
    map_yaml_path = os.path.join(mapping_pkg_share_dir,"maps","laboratory_map.yaml")
    nav2_param_path = os.path.join(pkg_share_dir,"config","nav2_params.yaml")
    
    
    
    nav2_node =  IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_pkg_share_dir, '/launch', '/bringup_launch.py']),
            # 使用 Launch 参数替换原有参数
            launch_arguments={
                'map': map_yaml_path,
                'params_file': nav2_param_path,
                }.items(),
    )
    
    lifecycle_nodes = ["map_server_node"]
    nav2_lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        emulate_tty=True,  # 模拟TTY以保留输出格式和颜色
        parameters=[
            {"node_names": lifecycle_nodes},
            {"autostart": True}
        ],
    )

     
    nav2_map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server_node",
        output="screen",
        emulate_tty=True,  # 模拟TTY以保留输出格式和颜色
        parameters=[
            {"yaml_filename":map_yaml_path},
        ]
    )

    
    static_tf_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("agrobot_car_navigation"),"/launch","/static_tf_launch.py"]),
    )
    
    return launch.LaunchDescription([
            # static_tf_node,
            nav2_node,
            # nav2_map_server_node,
            # nav2_lifecycle_manager_node,
            ])
