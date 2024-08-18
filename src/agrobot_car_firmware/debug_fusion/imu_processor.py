import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
import math
from scipy.spatial.transform import Rotation as R

class ImuProcessor(Node):
    def __init__(self):
        super().__init__("imu_processor")

        # 发布处理后的 IMU 数据
        self.imu_publisher = self.create_publisher(Imu, '/imu', 10)

        # 初始化四元数为单位四元数
        self.orientation_q = np.array([1.0, 0.0, 0.0, 0.0])
        
        # 初始化用于协方差计算的变量
        self.acc_data = []
        self.gyro_data = []

        # 记录上一个数据时间点
        self.last_time = self.get_clock().now()

    def processor(self, linear_acc, angular_vel, now):
        # 计算时间差
        dt = (now.nanoseconds - self.last_time.nanoseconds) / 1e9

        # 计算四元数
        self.update_orientation(angular_vel, dt)

        # 收集数据用于协方差计算
        self.acc_data.append(linear_acc)
        self.gyro_data.append(angular_vel)
        
        # 计算协方差
        acc_covariance = self.calculate_covariance(self.acc_data)
        gyro_covariance = self.calculate_covariance(self.gyro_data)

        # 打印协方差矩阵（用于调试）
        self.get_logger().info(f"acc_covariance: {acc_covariance}")
        self.get_logger().info(f"gyro_covariance: {gyro_covariance}")
        
        # 创建并发布新的IMU消息
        new_imu_msg = Imu()
        new_imu_msg.header.stamp = now.to_msg()
        new_imu_msg.header.frame_id = "imu_link"
        new_imu_msg.orientation.x = self.orientation_q[0]
        new_imu_msg.orientation.y = self.orientation_q[1]
        new_imu_msg.orientation.z = self.orientation_q[2]
        new_imu_msg.orientation.w = self.orientation_q[3]
        
        new_imu_msg.angular_velocity.x = angular_vel[0]
        new_imu_msg.angular_velocity.y = angular_vel[1]
        new_imu_msg.angular_velocity.z = angular_vel[2]
        new_imu_msg.linear_acceleration.x = linear_acc[0]
        new_imu_msg.linear_acceleration.y = linear_acc[1]
        new_imu_msg.linear_acceleration.z = linear_acc[2]
        
        # 将协方差矩阵填入消息
        new_imu_msg.orientation_covariance = gyro_covariance.flatten().tolist()
        new_imu_msg.angular_velocity_covariance = gyro_covariance.flatten().tolist()
        new_imu_msg.linear_acceleration_covariance = acc_covariance.flatten().tolist()
        
        self.last_time = now
        self.imu_publisher.publish(new_imu_msg)

    def update_orientation(self, angular_vel, dt):
        # 角速度转为四元数增量
        theta = np.linalg.norm(angular_vel) * dt
        if theta > 0:
            axis = angular_vel / np.linalg.norm(angular_vel)
            q_delta = np.concatenate(([math.cos(theta / 2)], axis * math.sin(theta / 2)))
        else:
            q_delta = np.array([1.0, 0.0, 0.0, 0.0])
        
        # 四元数更新
        self.orientation_q = self.quaternion_multiply(self.orientation_q, q_delta)

        # 四元数规范化
        self.orientation_q /= np.linalg.norm(self.orientation_q)
    
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
    
    def calculate_covariance(self, data):
        # 使用滑动窗口机制来限制数据量
        window_size = 100  # 可以根据需要调整窗口大小
        if len(data) > window_size:
            data = data[-window_size:]  # 只保留最近的window_size个数据

        # 计算协方差矩阵
        if len(data) > 1:
            covariance_matrix = np.cov(np.array(data).T)
        else:
            covariance_matrix = np.zeros((3, 3))

        # 将所有为零或接近零的值替换为一个小的正数
        epsilon = 1e-6
        covariance_matrix[covariance_matrix < epsilon] = epsilon

        return covariance_matrix


