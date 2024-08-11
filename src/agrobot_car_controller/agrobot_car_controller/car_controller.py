#! /usr/bin/env python3
import rclpy
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class car_controller(Node):
    def __init__(self):
        super().__init__('car_controller')

        # 目标位置
        self.target_x = 1.0  # 目标 x 坐标
        self.target_y = 1.0  # 目标 y 坐标
        self.current_x = 0.0  # 当前 x 坐标
        self.current_y = 0.0  # 当前 y 坐标
        self.roll=0.0
        self.pitch=0.0
        self.yaw=0.0
        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()

        # 订阅当前位置
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10, callback_group=self.group1
        )
        self.odom_sub_1 = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback_1,
            10, callback_group=self.group2
        )
        # 发布控制指令
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 标记当前步骤
        self.current_step = 1

        # 角速度
        self.angular_velocity = math.pi / 2  # 90 度/秒
        self.timer=None
        self.first_yaw =None
        self.timer_count=0

    def odom_callback(self, odom_msg):
        self.current_x = odom_msg.pose.pose.position.x
        self.current_y = odom_msg.pose.pose.position.y
        orientation=odom_msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation)
        self.get_logger().info('Current Position: x={:.2f}, y={:.2f}, roll={:.2f}, pitch={:.2f}, yaw={:.2f}'.format(self.current_x, self.current_y, self.roll, self.pitch, self.yaw))


    def odom_callback_1(self,odom_msg):

        if self.current_step == 1:  # 走完 x 轴
            if abs(self.current_x - self.target_x) < 0.05:
                self.get_logger().info('X direction target position reached!')
                self.current_step = 2  # 进入第二步：旋转
                self.stop_car()
                return
            
            cmd_vel = Twist()
            if self.current_x < self.target_x:
                cmd_vel.linear.x = 0.5  # 正向线速度，可根据需要进行调整
            else:
                cmd_vel.linear.x = -0.5  # 反向线速度，可根据需要进行调整

            self.cmd_vel_pub.publish(cmd_vel)

        elif self.current_step == 2 and abs(self.current_y - self.target_y) > 0.05:  # 根据 y 轴坐标差选择旋转方向
            dy = self.target_y - self.current_y
            if dy > 0:
                angle = math.pi / 2  # 向左旋转 90 度
            else :
                angle = -math.pi / 2  # 向右旋转 90 度
            
            if not self.first_yaw:
                self.first_yaw=self.yaw+angle

            while abs(self.first_yaw-self.yaw)>0.05:
                cmd_vel = Twist()
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = angle
                self.cmd_vel_pub.publish(cmd_vel)   


            self.stop_car()
            self.current_step = 3      

        elif self.current_step == 3:  # 走完 y 轴
            if abs(self.current_y - self.target_y) < 0.05:
                self.get_logger().info('Target position reached!')
                self.stop_car()
                return

            cmd_vel = Twist()
            cmd_vel.linear.x = 0.5  # 前进线速度，可根据需要进行调整
            self.cmd_vel_pub.publish(cmd_vel)


    def stop_car(self):
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    car_controller = car_controller()
    # executor = MultiThreadedExecutor(num_threads=2)
    executor = SingleThreadedExecutor()
    # Add the node to the executor
    executor.add_node(car_controller)
    try:
        executor.spin()
    finally:
        # Shutdown the executor
        executor.shutdown()
        car_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
