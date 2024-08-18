import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np

class OdometryProcessor(Node):
    def __init__(self):
        super().__init__("odometry_processor")

        # 发布处理后的里程计数据
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

        # 初始化用于协方差计算的变量
        self.pose_data = []
        self.twist_data = []

        # 记录上一个数据时间点
        self.last_time = self.get_clock().now()

    def processor(self, pose, twist, now):
        # 计算时间差
        dt = (now.nanoseconds - self.last_time.nanoseconds) / 1e9

        # 收集数据用于协方差计算
        self.pose_data.append([pose.position.x, pose.position.y, pose.position.z])
        self.twist_data.append([twist.linear.x, twist.linear.y, twist.linear.z,
                                twist.angular.x, twist.angular.y, twist.angular.z])
        
        # 计算协方差
        pose_covariance = self.calculate_covariance(self.pose_data)
        twist_covariance = self.calculate_covariance(self.twist_data)

        # 打印协方差矩阵（用于调试）
        self.get_logger().info(f"pose_covariance: {pose_covariance}")
        self.get_logger().info(f"twist_covariance: {twist_covariance}")
        
        # 创建并发布新的里程计消息
        new_odom_msg = Odometry()
        new_odom_msg.header.stamp = now.to_msg()
        new_odom_msg.header.frame_id = "odom"
        new_odom_msg.child_frame_id = "base_footprint"

        new_odom_msg.pose.pose = pose
        new_odom_msg.twist.twist = twist
        
        # 将协方差矩阵填入消息
        # new_odom_msg.pose.covariance = self.flatten_covariance(pose_covariance)
        # new_odom_msg.twist.covariance = self.flatten_covariance(twist_covariance)
        new_odom_msg.pose.covariance = [1.e-01, 0.e+00, 0.e+00, 0.e+00, 0.e+00, 0.e+00,
                                        0.e+00, 1.e-01, 0.e+00, 0.e+00, 0.e+00, 0.e+00,
                                        0.e+00, 0.e+00, 1.e-01, 0.e+00, 0.e+00, 0.e+00,
                                        0.e+00, 0.e+00, 0.e+00, 1.e-01, 0.e+00, 0.e+00,
                                        0.e+00, 0.e+00, 0.e+00, 0.e+00, 1.e-01, 0.e+00,
                                        0.e+00, 0.e+00, 0.e+00, 0.e+00, 0.e+00, 1.e-01]
        
        new_odom_msg.twist.covariance = [1.e-01, 0.e+00, 0.e+00, 0.e+00, 0.e+00, 0.e+00,
                                        0.e+00, 1.e-01, 0.e+00, 0.e+00, 0.e+00, 0.e+00,
                                        0.e+00, 0.e+00, 1.e-01, 0.e+00, 0.e+00, 0.e+00,
                                        0.e+00, 0.e+00, 0.e+00, 1.e-01, 0.e+00, 0.e+00,
                                        0.e+00, 0.e+00, 0.e+00, 0.e+00, 1.e-01, 0.e+00,
                                        0.e+00, 0.e+00, 0.e+00, 0.e+00, 0.e+00, 1.e-01]
        
        self.last_time = now
        self.odom_publisher.publish(new_odom_msg)

    def calculate_covariance(self, data):
        # 使用滑动窗口机制来限制数据量
        window_size = 100  # 可以根据需要调整窗口大小
        if len(data) > window_size:
            data = data[-window_size:]  # 只保留最近的window_size个数据

        # 计算协方差矩阵
        if len(data) > 1:
            covariance_matrix = np.cov(np.array(data).T)
        else:
            covariance_matrix = np.zeros((len(data[0]), len(data[0])))

        # 将所有为零或接近零的值替换为一个小的正数
        epsilon = 1e-9
        covariance_matrix[covariance_matrix < epsilon] = epsilon

        return covariance_matrix

    def flatten_covariance(self, cov_matrix):
        # 将协方差矩阵展平为长度为36的列表，并填充缺少的值
        cov_list = cov_matrix.flatten().tolist()
        if len(cov_list) < 36:
            # 使用高不确定性值（如1.0或更大）来填充剩余的部分，而不是0.0
            cov_list.extend([1.0] * (36 - len(cov_list)))
        return cov_list


