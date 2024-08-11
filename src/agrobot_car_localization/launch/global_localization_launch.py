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
    
    pkg_share_dir = get_package_share_directory("agrobot_car_localization")
    amcl_config_path = os.path.join(pkg_share_dir,"config","amcl.yaml")
    
    map_pkg_share_dir = get_package_share_directory("agrobot_car_mapping")
    map_path = os.path.join(map_pkg_share_dir,"maps","laboratory_map.yaml")
    
    lifecycle_nodes = ["map_server","amcl"]
    
    use_sim_time = False
    
    amcl_config = LaunchConfiguration("amcl_config")
    arg_amcl_config = DeclareLaunchArgument(
        "amcl_config",
        default_value=amcl_config_path
    )

    nav2_map_server_node = LifecycleNode(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        namespace='',
        parameters=[
            {"yaml_filename":map_path},
            {"use_sim_time":use_sim_time},
        ]
    )
    
    # 主要用于在已知地图上进行定位，它依赖于已知的地图
    # 它是在运动中实现定位
    nav2_amcl_node = LifecycleNode(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        emulate_tty=True,
        namespace='',
        parameters=[
            amcl_config,
            {"use_sim_time":use_sim_time},
        ],
    )

    nav2_lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time":use_sim_time},
            {"autostart": True}
        ],
    )

    static_tf_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("agrobot_car_localization"),"/launch","/static_tf_launch.py"]),
    )
    

    return LaunchDescription([
        # static_tf_node,
        arg_amcl_config,
        nav2_map_server_node,
        nav2_amcl_node,
        nav2_lifecycle_manager_node
        ])
