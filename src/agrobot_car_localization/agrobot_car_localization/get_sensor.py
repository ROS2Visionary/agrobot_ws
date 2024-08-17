from rclpy.node import Node
from sensor_msgs.msg import Imu
import rclpy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry


class get_navigation_m1(Node):
 
    def __init__(self):
        super().__init__("get_navigation_point")
        # self.odom_sub = self.create_subscription(Odometry,"/odom",self.accept_odom,10)
        self.odom_sub = self.create_subscription(Imu,"/imu",self.accept_imu,10)


    def accept_odom(self,msg:Odometry):
        self.get_logger().info(f"odom: {msg}\n")

    def accept_imu(self,msg:Imu):
        # self.get_logger().info(f"imu: {msg}\n")
        self.get_logger().info(f"四元数：{msg.orientation}")
        self.get_logger().info(f"角速度：{msg.angular_velocity}")
        self.get_logger().info(f"线性加速度：{msg.linear_acceleration}\n")


def main(args=None):
    rclpy.init(args=args)
    node = get_navigation_m1()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 处理 Ctrl-C 中断
        pass
    finally:
        # 在节点销毁前调用清理方法
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":

    main()






