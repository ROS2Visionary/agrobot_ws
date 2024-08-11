#!/usr/bin/env python3
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from twist_mux_msgs.action import JoyTurbo
from rclpy.action import ActionClient
from visualization_msgs.msg import Marker, MarkerArray


class State(Enum):
    FREE = 0        # 自由状态，无障碍物
    WARNING = 1     # 警告状态，接近障碍物
    DANGER = 2      # 危险状态，距离障碍物很近


class SafetyStop(Node):
    def __init__(self):
        super().__init__('safety_stop_node')
        self.declare_parameter('warning_distance', 0.6)    # 警告距离参数，默认0.6米
        self.declare_parameter('danger_distance', 0.2)     # 危险距离参数，默认0.2米
        self.declare_parameter('scan_topic', 'scan')       # 激光扫描话题，默认为'scan'
        self.declare_parameter('safety_stop_topic', 'safety_stop')  # 安全停止话题，默认为'safety_stop'
        self.warning_distance = self.get_parameter('warning_distance').get_parameter_value().double_value
        self.danger_distance = self.get_parameter('danger_distance').get_parameter_value().double_value
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.safety_stop_topic = self.get_parameter('safety_stop_topic').get_parameter_value().string_value
        self.is_first_msg = True    # 第一条消息标志位
        self.state = State.FREE     # 初始状态为自由
        self.prev_state = State.FREE    # 上一个状态为自由

        # 创建订阅器和发布器
        self.laser_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.laser_callback, 10
        )
        self.safety_stop_pub = self.create_publisher(
            Bool, self.safety_stop_topic, 10
        )
        self.zones_pub = self.create_publisher(
            MarkerArray, 'zones', 10
        )
        self.decrease_speed_client = ActionClient(self, JoyTurbo, 'joy_turbo_decrease')   # 减速动作客户端
        self.increase_speed_client = ActionClient(self, JoyTurbo, 'joy_turbo_increase')   # 加速动作客户端
        
        # 等待减速和加速动作服务器可用
        while not self.decrease_speed_client.wait_for_server(timeout_sec=1.0) and rclpy.ok():
            self.get_logger().warn('Action /joy_turbo_decrease not available! Waiting..')
            time.sleep(2.0)

        while not self.increase_speed_client.wait_for_server(timeout_sec=1.0) and rclpy.ok():
            self.get_logger().warn('Action /joy_turbo_increase not available! Waiting..')
            time.sleep(2.0)

        # 准备用于可视化的区域标记
        self.zones = MarkerArray()
        warning_zone = Marker()
        warning_zone.id = 0
        warning_zone.type = Marker.CYLINDER
        warning_zone.action = Marker.ADD
        warning_zone.scale.z = 0.001
        warning_zone.scale.x = self.warning_distance * 2
        warning_zone.scale.y = self.warning_distance * 2
        warning_zone.color.r = 1.0
        warning_zone.color.g = 0.984
        warning_zone.color.b = 0.0
        warning_zone.color.a = 0.5
        danger_zone = Marker()
        danger_zone.id = 1
        danger_zone.type = Marker.CYLINDER
        danger_zone.action = Marker.ADD
        danger_zone.scale.z = 0.001
        danger_zone.scale.x = self.danger_distance * 2
        danger_zone.scale.y = self.danger_distance * 2
        danger_zone.color.r = 1.0
        danger_zone.color.g = 0.0
        danger_zone.color.b = 0.0
        danger_zone.color.a = 0.5
        danger_zone.pose.position.z = 0.01
        
        self.zones.markers = [warning_zone, danger_zone]

    def laser_callback(self, msg: LaserScan):
        self.state = State.FREE   # 初始状态设为自由

        for range_value in msg.ranges:
            if range_value <= self.warning_distance:
                self.state = State.WARNING   # 如果距离小于等于警告距离，则设为警告状态

                if range_value <= self.danger_distance:
                    self.state = State.DANGER   # 如果距离小于等于危险距离，则设为危险状态
                    # 立即停止！
                    break

        # 如果当前状态与前一个状态不同
        if self.state != self.prev_state:
            is_safety_stop = Bool()

            if self.state == State.WARNING:
                is_safety_stop.data = False   # 发布非安全停止信号
                self.decrease_speed_client.send_goal_async(JoyTurbo.Goal())   # 发送减速动作目标
                self.zones.markers[0].color.a = 1.0   # 警告区域颜色透明度设为1
                self.zones.markers[1].color.a = 0.5   # 危险区域颜色透明度设为0.5
            elif self.state == State.DANGER:
                is_safety_stop.data = True   # 发布安全停止信号
                self.zones.markers[0].color.a = 1.0   # 警告区域颜色透明度设为1
                self.zones.markers[1].color.a = 1.0   # 危险区域颜色透明度设为1
            elif self.state == State.FREE:
                is_safety_stop.data = False   # 发布非安全停止信号
                self.increase_speed_client.send_goal_async(JoyTurbo.Goal())   # 发送加速动作目标
                self.zones.markers[0].color.a = 0.5   # 警告区域颜色透明度设为0.5
                self.zones.markers[1].color.a = 0.5   # 危险区域颜色透明度设为0.5

            self.prev_state = self.state   # 更新前一个状态
            self.safety_stop_pub.publish(is_safety_stop)   # 发布安全停止/非安全停止信号

        if self.is_first_msg:
            for zone in self.zones.markers:
                zone.header.frame_id = msg.header.frame_id

            self.is_first_msg = False

        self.zones_pub.publish(self.zones)   # 发布区域标记


def main():
    rclpy.init()
    node = SafetyStop()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
