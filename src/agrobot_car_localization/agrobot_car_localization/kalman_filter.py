#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class kalman_filter(Node):

    def __init__(self):
        super().__init__("kalman_filter")

        # 创建订阅器和发布器
        self.odom_subscriber = self.create_subscription(Odometry, "odom_noisy", self.odomCallback, 10)
        self.imu_subscriber = self.create_subscription(Imu, "imu", self.imuCallback, 10)
        self.odom_publisher = self.create_publisher(Odometry, "odom_kalman", 10)
        
        # 初始化 Kalman 滤波器参数
        self.mean = 0.0  # 初始状态的均值
        self.variance = 1000.0  # 初始状态的方差

        # 模拟传感器和运动的不确定性
        self.motion_variance = 4.0  # 运动模型的方差
        self.measurement_variance = 0.5  # 测量模型的方差

        # 存储 IMU 测量的角速度信息
        self.imu_angular_z = 0.0

        # 状态标记
        self.is_first_odom = True
        self.last_angular_z = 0.0
        self.motion = 0.0

        # 发布滤波后的里程计消息
        self.kalman_odom = Odometry()


    def odomCallback(self, odom):
        # 更新滤波器的当前状态
        self.kalman_odom = odom

        if self.is_first_odom:
            # 对于第一次收到的里程计数据，初始化状态
            self.last_angular_z = odom.twist.twist.angular.z
            self.is_first_odom = False
            self.mean = odom.twist.twist.angular.z
            return
        
        # 计算运动量
        self.motion = odom.twist.twist.angular.z - self.last_angular_z

        # 预测状态
        self.statePrediction()

        # 更新测量
        self.measurementUpdate()

        # 更新上一个角速度值
        self.last_angular_z = odom.twist.twist.angular.z

        # 更新并发布滤波后的里程计消息
        self.kalman_odom.twist.twist.angular.z = self.mean
        self.odom_publisher.publish(self.kalman_odom)


    def imuCallback(self, imu):
        # 存储 IMU 的角速度测量
        self.imu_angular_z = imu.angular_velocity.z


    def measurementUpdate(self):
        # 更新状态的均值和方差
        self.mean = (self.measurement_variance * self.mean + self.variance * self.imu_angular_z) \
                   / (self.variance + self.measurement_variance)
                     
        self.variance = (self.variance * self.measurement_variance) \
                       / (self.variance + self.measurement_variance)


    def statePrediction(self):
        # 预测下一个状态的均值和方差
        self.mean = self.mean + self.motion_variance
        self.variance = self.variance + self.motion_variance


def main(args=None):
    rclpy.init(args=args)
    node = kalman_filter()
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
