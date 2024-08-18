import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String
import math
import numpy as np
import threading
import time
from struct import pack, unpack
import binascii
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import TransformStamped
import tf2_ros
import random
from imu_processor import ImuProcessor
from odometry_processor import OdometryProcessor

class CarControlAndFeedback(Node):
    
    def __init__(self):
        super().__init__("motor_control")
        
        # 初始化串口连接，设置波特率为115200
        self.ser = serial.Serial("/dev/ttyACM0", 115200, timeout=5.0)
        
        # 创建TF广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 记录上一个数据时间点
        self.last_time = self.get_clock().now()
        # 用于保存位姿和角度信息
        self.position_x = 0.0
        self.position_y = 0.0
        self.orientation_z = 0.0
        self.orientation_w = 1.0
        self.theta = 0.0
        
        self.is_turn = False 
        self.is_init_pose = False
        self.output_tags = 0
        self.temp_arr = []

        self.imu_processor = ImuProcessor()
        self.odom_processor = OdometryProcessor()

        
    def read_data(self):
        while self.ser.is_open:
            #  0x7B 帧头
            # 0x7D 帧尾
            if self.ser.in_waiting:
                head = binascii.b2a_hex(self.ser.read(1))
                if head == b"7b":
                    self.ser.read(1)
                    
                    x_linear = self.decode_speed(unpack("B", self.ser.read(1))[0], unpack("B", self.ser.read(1))[0]) * 0.001
                    y_linear = self.decode_speed(unpack("B", self.ser.read(1))[0], unpack("B", self.ser.read(1))[0]) * 0.001
                    z_linear = self.decode_speed(unpack("B", self.ser.read(1))[0], unpack("B", self.ser.read(1))[0]) * 0.001

                    x_acceleration = self.decode_speed(unpack("B", self.ser.read(1))[0], unpack("B", self.ser.read(1))[0]) / 1672
                    y_acceleration = self.decode_speed(unpack("B", self.ser.read(1))[0], unpack("B", self.ser.read(1))[0]) / 1672
                    z_acceleration = self.decode_speed(unpack("B", self.ser.read(1))[0], unpack("B", self.ser.read(1))[0]) / 1672

                    x_angular_vel = self.decode_speed(unpack("B", self.ser.read(1))[0], unpack("B", self.ser.read(1))[0]) / 3753
                    y_angular_vel = self.decode_speed(unpack("B", self.ser.read(1))[0], unpack("B", self.ser.read(1))[0]) / 3753
                    z_angular_vel = self.decode_speed(unpack("B", self.ser.read(1))[0], unpack("B", self.ser.read(1))[0]) / 3753
                    
                    self.ser.read(3)  # 忽视电压和校验位

                    # self.get_logger().info(f"线速度：{x_linear} , {y_linear} , {z_linear}")
                    # self.get_logger().info(f"加速度：{x_acceleration} , {y_acceleration} , {z_acceleration}")
                    # self.get_logger().info(f"角速度：{x_angular_vel} , {y_angular_vel} , {z_angular_vel}\n\n")

                    if binascii.b2a_hex(self.ser.read(1)) == b"7d":  # 通过帧尾来判断数据是否有误
                        
                        # if x_linear == 0.0:
                        #     z_angular_vel = 0.0
                        now = self.get_clock().now()
                        self.get_logger().info("\n\n")
                        self.release_odometer(x_linear, z_angular_vel,now)
                        self.imu_processor.processor([x_acceleration, y_acceleration, z_acceleration],[x_angular_vel, y_angular_vel, z_angular_vel],now)

    
    def release_odometer(self, x_linear, omega,now):
        # 算线速度（x_linear）和角速度（omega）

        # 更新车辆的位置和角度
        dt = (now.nanoseconds - self.last_time.nanoseconds) / 1e9
        self.position_x += x_linear * dt * math.cos(self.theta)
        self.position_y += x_linear * dt * math.sin(self.theta)
        self.theta += omega * dt

        # 设置位置信息
        pose = Pose()
        pose.position.x = self.position_x
        pose.position.y = self.position_y
        pose.position.z = 0.0

        # 设置方向信息
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = self.orientation_z + math.sin(self.theta / 2)
        pose.orientation.w = self.orientation_w + math.cos(self.theta / 2)

        # 设置速度信息
        twist = Twist()
        twist.linear.x = x_linear
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = omega

        self.odom_processor.processor(pose,twist,now)


        # 创建TransformStamped消息
        # t = TransformStamped()
        # t.header.stamp = self.get_clock().now().to_msg()
        # t.header.frame_id = 'odom'
        # t.child_frame_id = 'base_footprint'
        # t.transform.translation.x = random.choice([0.0005, 0.00101, 0.00151])
        # t.transform.translation.x = odom.pose.pose.position.x
        # t.transform.translation.y = odom.pose.pose.position.y
        # t.transform.translation.z = odom.pose.pose.position.z
        # t.transform.rotation = odom.pose.pose.orientation
        # self.tf_broadcaster.sendTransform(t)


    def decode_speed(self, high_byte, low_byte):
        # 组合高字节和低字节
        combined_value = (high_byte << 8) | low_byte

        # 判断是正数还是负数
        if high_byte & 0x80:  # 如果高字节的最高位为1，表示负数
            speed_mm_s = combined_value - 0x10000
        else:
            speed_mm_s = combined_value

        return speed_mm_s

    
def main(args=None):
    rclpy.init(args=args)
    node = CarControlAndFeedback()
    
    # 启动子线程接收机器人发出的数据
    accept_agrobot_data_thread = threading.Thread(target=node.read_data)
    accept_agrobot_data_thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 处理 Ctrl-C 中断
        node.stop()
        pass
    finally:
        # 节点关闭前的清理工作
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
