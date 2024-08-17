import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf_transformations

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        # 创建发布者，发布到/initialpose话题
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # 创建PoseWithCovarianceStamped消息
        self.initial_pose = PoseWithCovarianceStamped()

        # 定时器，用于定期调用发布函数
        self.timer = self.create_timer(1.0, self.publish_initial_pose)  # 每5秒发布一次初始位置


        self.index = 1

    def publish_initial_pose(self):


        # 设置时间戳和参考系
        self.initial_pose.header.stamp = self.get_clock().now().to_msg()
        self.initial_pose.header.frame_id = 'map'

        # 设置位置信息（这里可以根据实际情况调整）
        self.initial_pose.pose.pose.position.x = self.index * 0.2
        self.initial_pose.pose.pose.position.y = 0.0
        self.initial_pose.pose.pose.position.z = 0.0

        # 设置朝向，使用四元数
        quat = tf_transformations.quaternion_from_euler(0, 0, 0)  # 假设初始朝向是零度
        self.initial_pose.pose.pose.orientation.x = quat[0]
        self.initial_pose.pose.pose.orientation.y = quat[1]
        self.initial_pose.pose.pose.orientation.z = quat[2]
        self.initial_pose.pose.pose.orientation.w = quat[3]

        # 发布初始位置
        self.pose_pub.publish(self.initial_pose)
        self.index += 1
        self.get_logger().info('已发布初始位置到/initialpose话题')

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
