import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # 获取启动目录
    costmap_filters_demo_dir = get_package_share_directory('agrobot_car_navigation')

    # 创建我们自己的临时YAML文件，包括替换
    lifecycle_nodes = ['filter_mask_server', 'costmap_filter_info_server']

    # 参数配置
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file_file = LaunchConfiguration('params_file_file')
    mask_yaml_file = LaunchConfiguration('mask_yaml_file')

    # 声明启动参数
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file_file',
        default_value=os.path.join(costmap_filters_demo_dir, 'config', 'keepout_params.yaml'),
        description='')

    declare_mask_yaml_file_cmd = DeclareLaunchArgument(
        'mask_yaml_file',
        default_value=os.path.join(costmap_filters_demo_dir, 'maps', 'keepout_mask.yaml'),
        description='要加载的过滤掩码yaml文件的完整路径')

    # 生成重写的yaml文件
    param_substitutions = {
        'use_sim_time': use_sim_time,  # 替换原始参数文件中的 use_sim_time 参数
        'yaml_filename': mask_yaml_file  # 替换原始参数文件中的 yaml_filename 参数
    }

    # 作用：将用户输入的参数覆盖掉params_file_file文件中对应的参数，(主要就是修改use_sim_time和yaml_filename)
    configured_params = RewrittenYaml(
        source_file=params_file_file,  # 原始参数文件的路径
        root_key=namespace,  # 参数文件的根键值，通常对应于节点的命名空间
        param_rewrites=param_substitutions,  # 要替换的参数及其新值
        convert_types=True  # 是否在重写参数时转换参数类型
    )

    # 节点启动命令
    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap_filters',
        namespace=namespace,
        output='screen',
        emulate_tty=True,  # 模拟TTY以保留输出格式和颜色
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}])

    start_map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server', # 根据名称提取params_file_file文件中对应的参数
        namespace=namespace,
        output='screen',
        emulate_tty=True,  # 模拟TTY以保留输出格式和颜色
        parameters=[configured_params])

    start_costmap_filter_info_server_cmd = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server', # 根据名称提取params_file_file文件中对应的参数
        namespace=namespace,
        output='screen',
        emulate_tty=True,  # 模拟TTY以保留输出格式和颜色
        parameters=[configured_params])
    
     
    ld = LaunchDescription()

    # 添加启动参数动作
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_mask_yaml_file_cmd)

    # 添加节点启动动作
    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(start_map_server_cmd)
    ld.add_action(start_costmap_filter_info_server_cmd)

    return ld

if __name__ == '__main__':
    generate_launch_description()
  