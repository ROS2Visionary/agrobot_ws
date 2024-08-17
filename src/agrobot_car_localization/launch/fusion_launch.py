import os
from launch import LaunchDescription  
from ament_index_python.packages import get_package_share_directory  
from launch_ros.actions import Node, LifecycleNode  
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription  
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution  
from launch.launch_description_sources import PythonLaunchDescriptionSource  
from nav2_common.launch import RewrittenYaml  

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    arg_sim = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description=''  
    )

    slam_config = os.path.join(get_package_share_directory("agrobot_localization"), "config", "mapper_params_localization.yaml") 
    localization_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("slam_toolbox"), "/launch", "/localization_launch.py"]),
        launch_arguments={
            "use_sim_time": use_sim_time,  
            "slam_params_file": slam_config  
        }.items()  
    )

    ekf_fusion_node = Node(
                    package='robot_localization',
                    executable='ekf_node',
                    name='ekf_filter_node',
                    output='screen',
                    parameters=[os.path.join(get_package_share_directory("agrobot_localization"), 'fusion_config', 'ekf.yaml')],
                )


    ukf_fusion_node = Node(
            package='robot_localization',
            executable='ukf_node',
            name='ukf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("agrobot_localization"), 'fusion_config', 'ukf.yaml')],
           )


    return LaunchDescription([
        arg_sim,
        localization_node,
        TimerAction(
            period=15.0,
            actions=[ekf_fusion_node]
        )
        # ukf_fusion_node,
    ])