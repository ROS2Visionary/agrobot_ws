import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations
import math
from rclpy.time import Time

class fusion_odom(Node):

    def __init__(self):
        super().__init__('odom_tf_publisher')
        
        # 创建订阅者,订阅'car/odom'和'gps/odom'话题
        self.gps_odom_sub = self.create_subscription(Odometry, 'gps/odom', self.gps_odom_callback, 10)
        self.car_odom_sub = self.create_subscription(Odometry, 'delta/odom', self.delta_odom_callback, 10)
        
        self.odom_pub = self.create_publisher(Odometry,"odom",10)
        
        # 创建TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 初始化状态
        self.gps_odom = None
        self.delta_odom = None
        self.last_odom = None
        self.use_gps = False
        self.gps_last_received = None  # 记录上次接收到GPS消息的时间
        
        # 定时器,用于定期检查并发布TF
        self.timer = self.create_timer(0.1, self.publish_tf)

    def gps_odom_callback(self, msg):
        """接收gps/odom话题的回调函数"""
        self.gps_odom = msg
        self.use_gps = True
        self.gps_last_received = self.get_clock().now()  # 更新接收时间
        self.get_logger().info("GPS数据可靠,使用gps/odom")

    def delta_odom_callback(self, msg):
        """接收car/odom话题的回调函数"""
        self.delta_odom = msg

    def publish_tf(self):
        """根据当前状态发布TF变换"""
        current_time = self.get_clock().now()
        
        # 检查GPS数据的时间戳是否过期
        if self.gps_last_received and (current_time.nanoseconds - self.gps_last_received.nanoseconds)/1e9 > 2.0:  # 2秒内无GPS更新
            self.use_gps = False
            self.get_logger().warn("GPS数据超时,切换到car/odom")

        if self.use_gps and self.gps_odom:
            # 使用GPS数据发布TF
            self.broadcast_tf(self.gps_odom)
        elif self.delta_odom:
            # 使用car/odom并在前一个TF基础上累积
            if self.last_odom:
                delta_transform = self.combine_transforms(self.last_odom, self.delta_odom)
                self.broadcast_tf(delta_transform)
            else:
                # 如果没有上次的TF数据,直接发布car/odom数据
                self.broadcast_tf(self.delta_odom)

    def broadcast_tf(self, odom_msg):
        
        # 创建TF变换消息
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = odom_msg.pose.pose.position.z
        
        t.transform.rotation = odom_msg.pose.pose.orientation
        
        # 记录并广播TF
        self.last_odom = odom_msg
        self.tf_broadcaster.sendTransform(t)



    def combine_transforms(self, last_odom, delta_odom):
        """将上一次的TF变换与增量变换结合"""
        combined_odom = Odometry()
        
        # 计算新的平移
        combined_odom.pose.pose.position.x = last_odom.pose.pose.position.x + delta_odom.pose.pose.position.x
        combined_odom.pose.pose.position.y = last_odom.pose.pose.position.y + delta_odom.pose.pose.position.y
        combined_odom.pose.pose.position.z = last_odom.pose.pose.position.z + delta_odom.pose.pose.position.z

        
        delta_roll, delta_pitch, delta_yaw = tf_transformations.euler_from_quaternion([delta_odom.pose.pose.orientation.x, delta_odom.pose.pose.orientation.y,
                   delta_odom.pose.pose.orientation.z, delta_odom.pose.pose.orientation.w])
        last_roll, last_pitch, last_yaw = tf_transformations.euler_from_quaternion([last_odom.pose.pose.orientation.x, last_odom.pose.pose.orientation.y,
                  last_odom.pose.pose.orientation.z, last_odom.pose.pose.orientation.w])
        
        yaw = delta_yaw + last_yaw

        # 设置方向信息
        combined_odom.pose.pose.orientation.x = 0.0
        combined_odom.pose.pose.orientation.y = 0.0
        combined_odom.pose.pose.orientation.z = math.sin(yaw / 2)
        combined_odom.pose.pose.orientation.w = math.cos(yaw / 2)
        
        return combined_odom

def main(args=None):
    rclpy.init(args=args)
    node = fusion_odom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
