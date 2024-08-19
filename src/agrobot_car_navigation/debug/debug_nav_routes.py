import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import tf_transformations
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import threading
import random

class debug_nav_routes(Node):

    def __init__(self):
        super().__init__('debug_nav_routes')

        # 创建TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)

        # 初始化机器人的位置和方向
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        self.group2 = MutuallyExclusiveCallbackGroup()
        # 订阅/cmd_vel话题
        self.twist_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_callback,
            10,
            callback_group=self.group2
        )

        # 创建发布者，发布到/initialpose话题
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.initial_pose = PoseWithCovarianceStamped()


    def twist_callback(self, msg):
        # 获取当前时间
        current_time = self.get_clock().now()

        # 计算时间差
        dt = (current_time - self.last_time).nanoseconds * 1e-9

         # 从Twist消息中提取线速度和角速度，并添加噪声
        noise_level = 0.01  # 噪声级别，可以根据需要调整
        vx = msg.linear.x + random.uniform(-noise_level, noise_level)
        vy = msg.linear.y + random.uniform(-noise_level, noise_level)  # 对于差动驱动机器人，vy通常为0
        vth = msg.angular.z + random.uniform(-noise_level, noise_level)

        self.get_logger().info(f"{vx} , {vy} , {vth}\n")

        # 计算位置的变化
        delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        delta_theta = vth * dt

        # 更新机器人的位置
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # 创建变换消息
        transform = TransformStamped()

        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_footprint"

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        # 从yaw角度创建四元数
        quat = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]

        # 发送变换
        self.tf_broadcaster.sendTransform(transform)

        # 更新上一次的时间
        self.last_time = current_time

    
    def pub_TF(self):

        # # 设置时间戳和参考系
        # self.initial_pose.header.stamp = self.get_clock().now().to_msg()
        # self.initial_pose.header.frame_id = 'map'

        # # 设置位置信息（这里可以根据实际情况调整）
        # self.initial_pose.pose.pose.position.x = -0.3
        # self.initial_pose.pose.pose.position.y = 4.0
        # self.initial_pose.pose.pose.position.z = 0.0

        # # 设置朝向，使用四元数
        # quat = tf_transformations.quaternion_from_euler(0, 0, -1.5611132853006096)  # 假设初始朝向是零度
        # self.initial_pose.pose.pose.orientation.x = quat[0]
        # self.initial_pose.pose.pose.orientation.y = quat[1]
        # self.initial_pose.pose.pose.orientation.z = quat[2]
        # self.initial_pose.pose.pose.orientation.w = quat[3]

        # 发布初始位置
        # self.pose_pub.publish(self.initial_pose)
        # self.get_logger().info('已发布初始位置到/initialpose话题')

        # 创建变换消息
        transform = TransformStamped()

        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "odom"

        transform.transform.translation.x = -0.3
        transform.transform.translation.y = 4.0
        transform.transform.translation.z = 0.0

        # 从yaw角度创建四元数
        quat = tf_transformations.quaternion_from_euler(0, 0, -1.5611132853006096)
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]

        # 发送变换
        self.tf_broadcaster.sendTransform(transform)

        pose_timer = threading.Timer(0.2,self.pub_TF)
        pose_timer.start()

def main(args=None):
    rclpy.init(args=args)

    node = debug_nav_routes()
    node.pub_TF()

    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
