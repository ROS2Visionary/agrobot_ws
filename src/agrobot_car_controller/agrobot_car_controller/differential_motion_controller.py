#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.constants import S_TO_NS
from rclpy.time import Time
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
from tf_transformations import quaternion_from_euler


class differential_motion_controller(Node):

    def __init__(self):
        super().__init__("simple_controller")
        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.17)

        # 获取并打印参数：轮子半径和轮子间距
        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info("Using wheel radius %f" % self.wheel_radius)
        self.get_logger().info("Using wheel separation %f" % self.wheel_separation)

        # 初始化变量：左右轮的前一位置、机器人的x、y坐标和角度theta
        self.left_wheel_prev_pos = 0.0
        self.right_wheel_prev_pos = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # 创建发布器和订阅器
        self.wheel_cmd_pub = self.create_publisher(Float64MultiArray, "wheel_cmd", 10)
        self.odom_pub = self.create_publisher(Odometry, "odom_noisy", 10)
        self.vel_sub = self.create_subscription(Twist, "cmd_vel_unstamped", self.velCallback, 10)
        self.joint_sub = self.create_subscription(JointState, "joint_states", self.jointCallback, 10)        
        

        # 计算速度转换矩阵
        self.speed_conversion = np.array([[self.wheel_radius/2, self.wheel_radius/2],
                                           [self.wheel_radius/self.wheel_separation, -self.wheel_radius/self.wheel_separation]])
        self.get_logger().info("转换矩阵： %s" % self.speed_conversion)

        # 填充Odometry消息的固定参数
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_footprint"
        self.odom_msg.pose.pose.orientation.x = 0.0
        self.odom_msg.pose.pose.orientation.y = 0.0
        self.odom_msg.pose.pose.orientation.z = 0.0
        self.odom_msg.pose.pose.orientation.w = 1.0

        # 填充TF消息
        self.br = TransformBroadcaster(self)
        self.transform_stamped = TransformStamped()
        self.transform_stamped.header.frame_id = "odom"
        self.transform_stamped.child_frame_id = "base_footprint"

        # 初始化时间变量
        self.prev_time = self.get_clock().now()


    def velCallback(self, msg):
        # 实现差动运动学模型
        # 根据线速度v和角速度w计算轮子速度
        robot_speed = np.array([[msg.linear.x],
                                [msg.angular.z]])
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion), robot_speed) 

        # 创建并发布轮子速度消息
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1, 0], wheel_speed[0, 0]]
        self.wheel_cmd_pub.publish(wheel_speed_msg)

    
    def jointCallback(self, msg):
        # 实现逆差动运动学模型
        # 根据轮子位置计算其速度，计算机器人在机器人坐标系下的速度，转换为全局坐标系并发布TF
        dp_left = msg.position[1] - self.left_wheel_prev_pos
        dp_right = msg.position[0] - self.right_wheel_prev_pos
        dt = Time.from_msg(msg.header.stamp) - self.prev_time

        # 更新前一次轮子位置和时间
        self.left_wheel_prev_pos = msg.position[1]
        self.right_wheel_prev_pos = msg.position[0]
        self.prev_time = Time.from_msg(msg.header.stamp)

        # 计算每个轮子的角速度
        fi_left = dp_left / (dt.nanoseconds / S_TO_NS)
        fi_right = dp_right / (dt.nanoseconds / S_TO_NS)

        # 计算机器人的线速度和角速度
        linear = (self.wheel_radius * fi_right + self.wheel_radius * fi_left) / 2
        angular = (self.wheel_radius * fi_right - self.wheel_radius * fi_left) / self.wheel_separation

        # 计算位置增量
        d_s = (self.wheel_radius * dp_right + self.wheel_radius * dp_left) / 2
        d_theta = (self.wheel_radius * dp_right - self.wheel_radius * dp_left) / self.wheel_separation
        self.theta += d_theta
        self.x += d_s * math.cos(self.theta)
        self.y += d_s * math.sin(self.theta)
        
        # 填充并发布Odometry消息
        q = quaternion_from_euler(0, 0, self.theta)
        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.orientation.x = q[0]
        self.odom_msg.pose.pose.orientation.y = q[1]
        self.odom_msg.pose.pose.orientation.z = q[2]
        self.odom_msg.pose.pose.orientation.w = q[3]
        self.odom_msg.twist.twist.linear.x = linear
        self.odom_msg.twist.twist.angular.z = angular
        self.odom_pub.publish(self.odom_msg)

        # 发布TF
        self.transform_stamped.transform.translation.x = self.x
        self.transform_stamped.transform.translation.y = self.y
        self.transform_stamped.transform.rotation.x = q[0]
        self.transform_stamped.transform.rotation.y = q[1]
        self.transform_stamped.transform.rotation.z = q[2]
        self.transform_stamped.transform.rotation.w = q[3]
        self.transform_stamped.header.stamp = self.get_clock().now().to_msg()
        self.br.sendTransform(self.transform_stamped)


def main(args=None):
    rclpy.init(args=args)
    node = differential_motion_controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 处理 Ctrl-C 中断
        pass
    finally:
        # 在节点销毁前调用清理方法
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
