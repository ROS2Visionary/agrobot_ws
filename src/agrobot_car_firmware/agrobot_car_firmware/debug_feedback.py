import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
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

class CarControlAndFeedback(Node):
    
    def __init__(self):
        super().__init__("motor_control")
        
        # 初始化串口连接，设置波特率为115200
        self.ser = serial.Serial("/dev/ttyACM0", 115200, timeout=5.0)

        # 创建TF广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # 创建里程计数据发布者
        self.pub_odom = self.create_publisher(Odometry, '/odom_noisy', 10)
        # 创建IMU数据发布者
        self.pub_imu = self.create_publisher(Imu, '/imu_nosiy', 10)

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

        # 初始化方向四元数
        self.orientation_q = np.array([1.0, 0.0, 0.0, 0.0])

        
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

                    if binascii.b2a_hex(self.ser.read(1)) == b"7d":  # 通过帧尾来判断数据是否有误
                        if x_linear == 0.0:
                            z_angular_vel = 0.0
                        
                        now = self.get_clock().now()
                        self.release_odometer(x_linear, z_angular_vel,now)
                        self.release_imu([x_acceleration, y_acceleration, z_acceleration],
                                         [x_angular_vel, y_angular_vel, z_angular_vel],now)

            
    def release_odometer(self, v, omega,now):
        # 算线速度（v）和角速度（omega）

        # 更新车辆的位置和角度
        dt = (now.nanoseconds - self.last_time.nanoseconds) / 1e9
        self.position_x += v * dt * math.cos(self.theta)
        self.position_y += v * dt * math.sin(self.theta)
        self.theta += omega * dt

        # 创建Odometry消息对象
        odom = Odometry()
        odom.header.stamp = now.to_msg()  # 设置消息的时间戳为当前时间
        odom.header.frame_id = 'odom'  
        odom.child_frame_id = "base_footprint"

        # 设置位置信息
        odom.pose.pose.position.x = self.position_x
        odom.pose.pose.position.y = self.position_y
        odom.pose.pose.position.z = 0.0

        # 设置方向信息
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = self.orientation_z + math.sin(self.theta / 2)
        odom.pose.pose.orientation.w = self.orientation_w + math.cos(self.theta / 2)

        # 设置速度信息
        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = omega

        # 设置协方差矩阵 (长度为 36，所有值都是浮点数)
        odom.pose.covariance = [
            0.001, 0.0, 0.0, 0.0, 0.0, 0.0,   # x方向
            0.0, 0.001, 0.0, 0.0, 0.0, 0.0,   # y方向
            0.0, 0.0, 1e-6, 0.0, 0.0, 0.0,    # z方向，始终为0，协方差极小
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,     # 关于x轴的旋转，极高协方差
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,     # 关于y轴的旋转，极高协方差
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01     # 关于z轴的旋转，实际变化的协方差
        ]

        odom.twist.covariance = [
            0.001, 0.0, 0.0, 0.0, 0.0, 0.0,   # x方向
            0.0, 0.001, 0.0, 0.0, 0.0, 0.0,   # y方向
            0.0, 0.0, 1e-6, 0.0, 0.0, 0.0,    # z方向，始终为0，协方差极小
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,     # 关于x轴的旋转，极高协方差
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,     # 关于y轴的旋转，极高协方差
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01     # 关于z轴的旋转，实际变化的协方差
        ]


        self.last_time = now

        self.output_tags += 1
        if self.output_tags > 10:
            self.output_tags = 0
            self.get_logger().info(f"Z轴转角: {np.degrees(self.theta)} - 位置:{odom.pose.pose.position}")
        self.pub_odom.publish(odom)

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

    def release_imu(self, linear_acceleration, angular_velocity, now):
        # 创建IMU消息对象
        imu = Imu()
        imu.header.stamp = now.to_msg()
        imu.header.frame_id = 'base_footprint'

        # 填充线性加速度数据
        imu.linear_acceleration.x = linear_acceleration[0]
        imu.linear_acceleration.y = linear_acceleration[1]
        imu.linear_acceleration.z = linear_acceleration[2]

        # 填充角速度数据
        imu.angular_velocity.x = angular_velocity[0]
        imu.angular_velocity.y = angular_velocity[1]
        imu.angular_velocity.z = angular_velocity[2]

        # 使用四元数积分更新IMU的方向信息
        dt = (now.nanoseconds - self.last_time.nanoseconds) / 1e9
        self.update_orientation(angular_velocity, dt)

        # 填充方向四元数
        imu.orientation.x = self.orientation_q[0]
        imu.orientation.y = self.orientation_q[1]
        imu.orientation.z = self.orientation_q[2]
        imu.orientation.w = self.orientation_q[3]

        imu.orientation_covariance = [0.001, 0.0, 0.0,
                                    0.0, 0.001, 0.0,
                                    0.0, 0.0, 0.01]  # z轴旋转较小，协方差略高

        imu.angular_velocity_covariance = [0.001, 0.0, 0.0,
                                        0.0, 0.001, 0.0,
                                        0.0, 0.0, 0.01]

        imu.linear_acceleration_covariance = [0.01, 0.0, 0.0,
                                            0.0, 0.01, 0.0,
                                            0.0, 0.0, 0.01]


        # 发布IMU消息
        self.pub_imu.publish(imu)

    def update_orientation(self, angular_velocity, dt):
        # 计算角速度的四元数
        wx, wy, wz = angular_velocity
        theta = math.sqrt(wx**2 + wy**2 + wz**2) * dt

        if theta > 0:
            axis = np.array([wx, wy, wz]) / theta
            q_delta = np.array([
                math.cos(theta / 2),
                axis[0] * math.sin(theta / 2),
                axis[1] * math.sin(theta / 2),
                axis[2] * math.sin(theta / 2)
            ])
        else:
            q_delta = np.array([1.0, 0.0, 0.0, 0.0])

        # 更新当前方向的四元数
        self.orientation_q = self.quaternion_multiply(self.orientation_q, q_delta)

    def quaternion_multiply(self, q1, q2):
        # 四元数乘法
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        ])


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
