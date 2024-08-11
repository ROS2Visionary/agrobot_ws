#!/usr/bin/env python3
import random
import time
from math import sin, cos, atan2, sqrt, fabs, pi

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler, euler_from_quaternion

def normalize(z):
    """
    将角度规范化到 [-π, π] 范围内。

    Args:
    - z: 角度值(弧度)

    Returns:
    规范化后的角度值
    """
    return atan2(sin(z), cos(z))

def angle_diff(a, b):
    """
    计算两个角度之间的差值，确保结果在 [-π, π] 范围内。

    Args:
    - a: 第一个角度(弧度)
    - b: 第二个角度(弧度)

    Returns:
    角度差值
    """
    a = normalize(a)
    b = normalize(b)
    d1 = a - b
    d2 = 2 * pi - fabs(d1)
    if d1 > 0:
        d2 *= -1.0
    if fabs(d1) < fabs(d2):
        return d1
    else:
        return d2

class OdometryMotionModel(Node):
    '''
    在使用自适应蒙特卡洛定位算法(ACML)时,在配置ACML的参数时,它也有4个alpha,该运动模型,主要是告诉你这四个参数所控制的是什么
    '''
    def __init__(self):
        super().__init__('odometry_motion_model')
        self.last_odom_x = 0.0
        self.last_odom_y = 0.0
        self.last_odom_theta = 0.0
        self.is_first_odom = True

        # 设置运动模型的噪声参数
        self.declare_parameter('alpha1', 0.2)
        self.declare_parameter('alpha2', 0.2)
        self.declare_parameter('alpha3', 0.2)
        self.declare_parameter('alpha4', 0.2)
        self.declare_parameter('nr_samples', 300)

        self.alpha1 = self.get_parameter('alpha1').get_parameter_value().double_value
        self.alpha2 = self.get_parameter('alpha2').get_parameter_value().double_value
        self.alpha3 = self.get_parameter('alpha3').get_parameter_value().double_value
        self.alpha4 = self.get_parameter('alpha4').get_parameter_value().double_value
        self.nr_samples = self.get_parameter('nr_samples').get_parameter_value().integer_value

        if self.nr_samples >= 0:
            # 初始化样本数组
            self.samples = PoseArray()
            self.samples.poses = [Pose() for _ in range(self.nr_samples)]
        else:
            self.get_logger().fatal('请求的样本数量无效: %d!', self.nr_samples)
            return

        # 订阅里程计信息
        self.odom_sub = self.create_subscription(
            Odometry, 'raw_odom', self.odom_callback, 10
        )
        # 发布样本数组
        self.pose_array_pub = self.create_publisher(
            PoseArray, 'odom', 10
        )

    def odom_callback(self, odom):
        # 获取四元数姿态信息，并转换为欧拉角
        q = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
             odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)

        if self.is_first_odom:
            # 初始化首次里程计信息
            self.samples.header.frame_id = odom.header.frame_id
            self.last_odom_x = odom.pose.pose.position.x
            self.last_odom_y = odom.pose.pose.position.y
            self.last_odom_theta = yaw
            self.is_first_odom = False
            return

        # 计算里程计增量
        odom_x_increment = odom.pose.pose.position.x - self.last_odom_x
        odom_y_increment = odom.pose.pose.position.y - self.last_odom_y
        odom_theta_increment = angle_diff(yaw, self.last_odom_theta)

        # 运动模型
        if sqrt(pow(odom_y_increment, 2) + pow(odom_x_increment, 2)) < 0.01:
            delta_rot1 = 0.0
        else:
            '''
            delta_rot1 表示机器人在直线运动之前的旋转角度变化。
            当机器人从当前位置移动到下一个位置时，首先需要调整姿态角，以确保朝向正确。这个调整角度就是 delta_rot1。
            delta_rot1 考虑了机器人在直线移动前的姿态调整，通常由当前位置到下一个目标位置的方向决定。
            '''
            # 计算机器人在两次里程计更新之间的第一个旋转增量
            delta_rot1 = angle_diff(atan2(odom_y_increment, odom_x_increment), yaw)

        # 计算机器人在两次里程计更新之间的平移(或直线运动)增量
        delta_trans = sqrt(pow(odom_x_increment, 2) + pow(odom_y_increment, 2))
        
        '''
        delta_rot2 表示机器人在直线运动之后的旋转角度变化。
        一旦机器人完成直线移动，它可能需要进一步调整姿态，以匹配目标方向或路径的曲线。
        delta_rot2 考虑了在直线移动后可能需要进行的额外旋转调整，以确保最终姿态正确。
        '''
        # 计算机器人在两次里程计更新之间的第二个旋转增量
        delta_rot2 = angle_diff(odom_theta_increment, delta_rot1)


        
        # 计算机器人在直线移动前的姿态调整中所带来的方差，考虑了旋转增量 delta_rot1 和直线移动增量 delta_trans 的影响
        '''
        它由两部分组成:
            self.alpha1 * delta_rot1:这部分考虑了机器人在旋转过程中由于其自身旋转精度不确定性(或噪声)导致的误差。
            self.alpha2 * delta_trans:这部分考虑了机器人在直线移动时，由于移动距离的不确定性(或噪声)导致的旋转误差。
        '''
        rot1_variance = self.alpha1 * delta_rot1 + self.alpha2 * delta_trans
        
        # 计算机器人在直线移动过程中的位置调整带来的方差，考虑了直线移动增量 delta_trans 和旋转增量 delta_rot1、delta_rot2 的综合影响
        '''
        它由两部分组成:
            self.alpha3 * delta_trans:这部分考虑了机器人直线移动过程中的移动距离不确定性(或噪声)。
            self.alpha4 * (delta_rot1 + delta_rot2):这部分考虑了机器人在移动过程中由于旋转调整(包括 delta_rot1 和 delta_rot2)所引起的移动误差。
        '''
        trans_variance = self.alpha3 * delta_trans + self.alpha4 * (delta_rot1 + delta_rot2)
        
        # 计算机器人在直线移动后可能进行的姿态调整中所带来的方差，考虑了旋转增量 delta_rot2 和直线移动增量 delta_trans 的影响
        '''
        它由两部分组成:
            self.alpha1 * delta_rot2:这部分考虑了机器人在旋转过程中由于其自身旋转精度不确定性(或噪声)导致的误差。
            self.alpha2 * delta_trans:这部分考虑了机器人在直线移动时，由于移动距离的不确定性(或噪声)导致的旋转误差。
        '''
        rot2_variance = self.alpha1 * delta_rot2 + self.alpha2 * delta_trans
        
        '''
        总体原理:
            这些方差的计算原理是为了在机器人运动模型中引入随机性(或称为噪声)，以更真实地模拟机器人在现实世界中的运动。
            通过调整 self.alpha1 到 self.alpha4 这些参数，可以控制噪声的级别，以适应不同的运动环境和具体的机器人动作模式。
        
        这些方差通常在机器人运动模型中用于模拟环境中，以模拟机器人在真实世界中的运动不确定性和误差。具体来说：
            1. 模拟环境中的应用：
            - 在仿真和模拟环境中，这些方差用于模拟机器人在执行路径规划和运动控制时可能遇到的各种不确定性和测量误差。
            - 通过在仿真中引入这些噪声方差，可以更真实地评估和验证机器人系统在不同条件下的性能表现，包括姿态估计、路径跟踪等方面。

            2. 真实环境中的应用：
            - 在真实环境中，虽然无法精确地预测和控制所有的运动误差和不确定性，但是这些方差的概念仍然有用。
            - 实际部署时，通常会结合传感器数据的实时反馈来调整和优化机器人的运动控制策略。
            - 例如，在自主导航和路径规划中，机器人可以通过实时测量和环境感知来校正其预期的运动路径，以应对真实环境中的障碍物、地形变化和其他变量。

            总之，这些方差在模拟环境中帮助优化算法和验证策略的同时，也能够部分地适用于真实环境中，尽管在实际部署中可能需要结合更多的传感器和反馈机制来处理实时的环境变化和误差调整。
        '''
        
        
        random.seed(int(time.time()))

        for sample in self.samples.poses:
            # 生成噪声
            # 从高斯分布（正态分布）中生成一个随机数，其中 mean 是均值（这里为0.0），stddev 是标准差（这里为 rot1_variance）
            rot1_noise = random.gauss(0.0, rot1_variance)
            trans_noise = random.gauss(0.0, trans_variance)
            rot2_noise = random.gauss(0.0, rot2_variance)
            
            # 计算根据噪声 rot1_noise 调整后的旋转增量 delta_rot1
            delta_rot1_draw = angle_diff(delta_rot1, rot1_noise)
            
            # 计算根据噪声 trans_noise 调整后的直线移动增量 delta_trans
            delta_trans_draw = delta_trans - trans_noise # 这一步用于模拟机器人在实际运动中由于测量误差或环境变化而产生的移动偏差
            
            # 计算根据噪声 rot2_noise 调整后的第二个旋转增量 delta_rot2
            delta_rot2_draw = angle_diff(delta_rot2, rot2_noise)

            # 更新样本位置
            # 提取样本当前的四元数姿态信息到列表 sample_q 中
            sample_q = [sample.orientation.x, sample.orientation.y,
                        sample.orientation.z, sample.orientation.w]

            # 将四元数转换为欧拉角（roll、pitch、yaw）
            sample_roll, sample_pitch, sample_yaw = euler_from_quaternion(sample_q)

            # 根据 delta_trans_draw 和调整后的旋转增量 delta_rot1_draw 更新样本的 x 坐标
            sample.position.x += delta_trans_draw * cos(sample_yaw + delta_rot1_draw)

            # 根据 delta_trans_draw 和调整后的旋转增量 delta_rot1_draw 更新样本的 y 坐标
            sample.position.y += delta_trans_draw * sin(sample_yaw + delta_rot1_draw)

            # 根据当前样本的偏航角 sample_yaw、delta_rot1_draw 和 delta_rot2_draw 计算新的姿态四元数
            q = quaternion_from_euler(0.0, 0.0, sample_yaw + delta_rot1_draw + delta_rot2_draw)

            # 将计算得到的新四元数 q 分配给样本的姿态四元数 sample.orientation.x, sample.orientation.y, sample.orientation.z, sample.orientation.w
            sample.orientation.x, sample.orientation.y, sample.orientation.z, sample.orientation.w = q


        # 更新下次迭代的数据
        self.last_odom_x = odom.pose.pose.position.x
        self.last_odom_y = odom.pose.pose.position.y
        self.last_odom_theta = yaw

        # 发布样本数组
        self.pose_array_pub.publish(self.samples)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryMotionModel()
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
