import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node,LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription,TimerAction

def generate_launch_description():
        
    map_yaml_path = os.path.join(get_package_share_directory("agrobot_car_mapping"),"maps","laboratory_map.yaml")
    nav2_param_path = os.path.join(get_package_share_directory("agrobot_car_navigation"),"config","nav2_keepout_filter_params.yaml")
    
    lifecycle_nodes = ["map_server_node"]
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    arg_sim = DeclareLaunchArgument(
        'use_sim_time',
        default_value="false"
    )
    
    nav2_node =  IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('nav2_bringup'), '/launch', '/bringup_launch.py']),
            # 使用 Launch 参数替换原有参数
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
    )

    map_server_node = LifecycleNode(
        package="nav2_map_server",
        executable="map_server",
        name="map_server_node",
        output="screen",
        namespace='',
        emulate_tty=True,  # 模拟TTY以保留输出格式和颜色
        parameters=[
            {"yaml_filename":map_yaml_path},
        ]
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

    keepout_filter_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("agrobot_car_navigation"),"/launch","/keepout_filter_launch.py"]),
        launch_arguments={
            "mask_yaml_file":os.path.join(get_package_share_directory("agrobot_car_mapping"),"maps","laboratory_mask.yaml")
        }.items()
    )
    
    static_tf_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("agrobot_car_navigation"),"/launch","/static_tf_launch.py"]),
    )

    # return launch.LaunchDescription([
    #     arg_sim,
    #     map_server_node,
    #     nav2_lifecycle_manager_node,
    #     TimerAction(
    #         period=10.0,
    #         actions=[keepout_filter_node]
    #     ),
    #     TimerAction(
    #         period=20.0,
    #         actions=[nav2_node]
    #     ),
    #     # static_tf_node,
    # ])
 
    return launch.LaunchDescription([
        arg_sim,
        nav2_node,
        keepout_filter_node
        # TimerAction(
        #     period=20.0,
        #     actions=[keepout_filter_node]
        # )
        # static_tf_node,
    ])
