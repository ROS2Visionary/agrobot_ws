
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    liser_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_laser',
            output='screen',
            arguments=['--x', '0.3', 
                       '--y', '0', 
                       '--z', '0.09', 
                       '--qx', '0', 
                       '--qy', '0', 
                       '--qz', '0', 
                       '--qw', '1', 
                       '--frame-id', 'base_link', 
                       '--child-frame-id', 'liser_link'
                       ]
        )
    
    base_link_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_base_footprint',
            output='screen',
            arguments=['--x', '0', 
                       '--y', '0', 
                       '--z', '0.05', 
                       '--qx', '0', 
                       '--qy', '0', 
                       '--qz', '0', 
                       '--qw', '1', 
                       '--frame-id', 'base_footprint', 
                       '--child-frame-id', 'base_link'
                       ]
        )
    
    base_footprint_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_odom',
            output='screen',
            arguments=['--x', '0', 
                       '--y', '0', 
                       '--z', '0', 
                       '--qx', '0', 
                       '--qy', '0', 
                       '--qz', '0', 
                       '--qw', '1', 
                       '--frame-id', 'odom', 
                       '--child-frame-id', 'base_footprint'
                       ]
        )
    
    odom_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_map',
            output='screen',
            arguments=['--x', '0', 
                       '--y', '0', 
                       '--z', '0', 
                       '--qx', '0', 
                       '--qy', '0', 
                       '--qz', '0', 
                       '--qw', '1', 
                       '--frame-id', 'map', 
                       '--child-frame-id', 'odom'
                       ]
        )

    return LaunchDescription([
            liser_tf_node,
            base_link_tf_node,
            base_footprint_tf_node,
            odom_tf_node
        ])

