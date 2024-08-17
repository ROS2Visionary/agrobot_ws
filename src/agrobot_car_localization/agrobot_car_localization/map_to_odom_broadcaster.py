import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations

class MapToOdomBroadcaster(Node):
    def __init__(self):
        super().__init__('map_to_odom_broadcaster')

        # 创建TransformBroadcaster对象，用于发布TF变换
        self.tf_broadcaster = TransformBroadcaster(self)

        # 定义并初始化map到odom的静态变换
        self.map_to_odom_transform = TransformStamped()

        # 创建定时器，定期发布TF变换
        self.timer = self.create_timer(1.0, self.broadcast_transform)  

        self.index = 1

    def broadcast_transform(self):
        
        # 更新时间戳
        self.map_to_odom_transform.header.stamp = self.get_clock().now().to_msg()

        # 设置帧ID
        self.map_to_odom_transform.header.frame_id = "map"
        self.map_to_odom_transform.child_frame_id = "odom"

        # 设置变换的初始位置和旋转
        self.map_to_odom_transform.transform.translation.x = self.index * 0.2
        self.map_to_odom_transform.transform.translation.y = 0.0
        self.map_to_odom_transform.transform.translation.z = 0.0

        # 设置旋转，使用四元数表示
        quat = tf_transformations.quaternion_from_euler(0, 0, 0)
        self.map_to_odom_transform.transform.rotation.x = quat[0]
        self.map_to_odom_transform.transform.rotation.y = quat[1]
        self.map_to_odom_transform.transform.rotation.z = quat[2]
        self.map_to_odom_transform.transform.rotation.w = quat[3]

        # 发布TF变换
        self.tf_broadcaster.sendTransform(self.map_to_odom_transform)
        self.index += 1
        self.get_logger().info('发布map到odom的TF变换')

def main(args=None):
    rclpy.init(args=args)
    node = MapToOdomBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
