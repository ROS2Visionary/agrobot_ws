import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,PoseWithCovarianceStamped
from std_msgs.msg import String
import math
import numpy as np
import threading
import time
from struct import pack,unpack
import binascii
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup


class CarControlAndFeedback(Node):
    
    def __init__(self):
        super().__init__("motor_control")
        
        # 初始化串口连接，设置波特率为115200
        self.ser = serial.Serial("/dev/ttyACM1", 115200, timeout=5.0)
 
        # 创建订阅者，订阅Twist类型消息，用于控制电机
        self.sub = self.create_subscription(Twist, "/cmd_vel", self.control_motor, 10)

        self.amcl_pose_group = MutuallyExclusiveCallbackGroup()
        self.amcl_pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 10,callback_group=self.amcl_pose_group)

        
        # 创建里程计数据发布者
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)

        # 记录上一个数据时间点
        self.last_time = self.get_clock().now()
        # 用于保存位姿和角度信息
        self.position_x = 0.0
        self.position_y = 0.0
        self.orientation_z = 0.0
        self.orientation_w = 1.0
        self.theta = 0.0

        self.rate = self.create_rate(2)
        
        self.is_turn = False 
        self.output_tags = 0
        self.temp_arr = []

    # AMCL初始位置回调函数
    def amcl_pose_callback(self, msg:PoseWithCovarianceStamped):
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        self.orientation_z = msg.pose.pose.orientation.z
        self.orientation_w = msg.pose.pose.orientation.w
        self.theta = 0.0
        
    def control_motor(self, msg: Twist):
        # 控制电机的方法，接收Twist消息类型作为参数
                
        x_speed = msg.linear.x
        # 没有用到y方向的速度，可以直接赋值为0
        y_speed = 0.0 
        z_angular_vel = msg.angular.z

        if z_angular_vel != 0.0:
            self.is_turn = True
        else:
            self.is_turn = False

        # 该电机实际接收的不是角速度，而是Z方向的线速度(具体可以看文档)，所以要将角速度按比例转换一下(X1.4)
        straight_cmd = self.pack_motor_cmd(x_speed,y_speed, z_angular_vel * 1.4) 
        self.ser.write(straight_cmd)
        

    def read_data(self):
        while self.ser.is_open:
            #  0x7B 帧头
            # 0x7D 帧尾
            if self.ser.in_waiting:
                head = binascii.b2a_hex(self.ser.read(1))
                if head == b"7b":
                    self.ser.read(1)
                    
                    x_linear = self.decode_speed(unpack("B",self.ser.read(1))[0],unpack("B",self.ser.read(1))[0]) * 0.001
                    self.ser.read(2) # 暂不引入y方向的速度,因为这履带车的运动模型原因y方向的速度永远为0
                    # y_linear = self.decode_speed(unpack("B",self.ser.read(1))[0],unpack("B",self.ser.read(1))[0]) * 0.001
                    z_linear = self.decode_speed(unpack("B",self.ser.read(1))[0],unpack("B",self.ser.read(1))[0]) * 0.001

                    self.ser.read(6) # 暂不引入x、y、z轴加速度，因为小车为履带车，只会在较低的速度下运行，而且小车加速的时间较短，并且控制板自带的imu精度较差
                    # x_acceleration = self.decode_speed(unpack("B",self.ser.read(1))[0],unpack("B",self.ser.read(1))[0]) / 1672
                    # y_acceleration = self.decode_speed(unpack("B",self.ser.read(1))[0],unpack("B",self.ser.read(1))[0]) / 1672
                    # z_acceleration = self.decode_speed(unpack("B",self.ser.read(1))[0],unpack("B",self.ser.read(1))[0]) / 1672

                    self.ser.read(4) # 暂不引入x、y轴角速度，因为在平地上运行的小车，可以忽视x、y轴的角速度
                    z_angular_vel = self.decode_speed(unpack("B",self.ser.read(1))[0],unpack("B",self.ser.read(1))[0]) / 3753
                    
                    self.ser.read(3) # 忽视电压和校验位

                    if binascii.b2a_hex(self.ser.read(1)) == b"7d": # 通过帧尾来判断数据是否有误
                        # 调试输出
                        # if z_linear > 0.0:
                        #     self.temp_arr.append(z_angular_vel / z_linear)
                        #     mean = np.sum(self.temp_arr) / len(self.temp_arr)
                        #     print(f"{x_linear} - {z_linear} - {mean}")

                        # if not self.is_turn:
                        #     z_angular_vel = 0.0
                        self.release_odometer(x_linear,z_angular_vel)

            
    def release_odometer(self,v, omega):
        # 算线速度（v）和角速度（omega）
        now = self.get_clock().now()
    
        # 更新车辆的位置和角度
        dt = (now.nanoseconds - self.last_time.nanoseconds) / 1e9
        self.position_x += v * dt * math.cos(self.theta)
        self.position_y += v * dt * math.sin(self.theta)
        self.theta += omega * dt
        

        # 创建Odometry消息对象
        odom = Odometry()
        odom.header.stamp = now.to_msg()  # 设置消息的时间戳为当前时间
        # 必须指定id，指定了坐标id才能让TF变换系统(需要先发布TF变换)自动查找并处理这些坐标框架之间的变换关系
        odom.header.frame_id = 'odom'  
        odom.child_frame_id = "base_link"

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

        self.last_time = now

        # self.output_tags += 1
        # if self.output_tags > 20:
        #     self.output_tags = 0
        #     if v != 0.0 or omega != 0.0:
        #         print(f"Z轴转角: {np.degrees(self.theta)} - 位置:{odom.pose.pose.position}")
        self.pub_odom.publish(odom)


    def stop(self):
        # 停止电机运行，将电机速度设置为0
        cmd = self.pack_motor_cmd(0, 0)
        self.ser.write(cmd) 

    def pack_motor_cmd(self,x_speed,y_speed, z_speed):
        # 打包电机控制命令
        header = 0x7B
        reserved_bit_1 = 0x00
        reserved_bit_2 = 0x00

        x_high_bit, x_low_bit = self.encode_speed(x_speed)
        y_high_bit, y_low_bit = self.encode_speed(y_speed)
        z_high_bit, z_low_bit = self.encode_speed(z_speed)

        etx = 0x7D

        cmd_list = [header, reserved_bit_1, reserved_bit_2,
                        x_high_bit, x_low_bit,
                        y_high_bit, y_low_bit,
                        z_high_bit, z_low_bit]
            
        # 计算前9个字节的BBC校验值
        checksum = self.bbc_checksum(cmd_list)
        check = checksum
            
        cmd_list.append(check)
        cmd_list.append(etx)

        
        cmd = []
        for b in cmd_list:
            cmd.append(pack("B",b))

        cmd = b"".join(cmd)

        # print(binascii.b2a_hex(cmd))
        return cmd

    def bbc_checksum(self,data):
        calculated_checksum = 0
        for byte in data:
            calculated_checksum ^= byte
        return calculated_checksum

    #  speed_m_s (int): 速度,以m/s为单位。可以是正数(前进)或负数(后退)。
    def encode_speed(self, speed_m_s):
        speed_mm_s = int(speed_m_s * 1000)
        # 对速度进行限制，以免速度过快，弄坏电机
        if not -200 <= speed_mm_s <= 200:
            if speed_mm_s < 0:
                speed_mm_s = -200
            if speed_mm_s > 0:
                speed_mm_s = 200
            # raise ValueError("速度必须在 -200 到 200 mm/s 范围内")

        if speed_mm_s >= 0:
            # 正数速度(前进)
            high_byte = (speed_mm_s // 256) & 0x7F  # 高字节，清除最高位
            low_byte = speed_mm_s % 256
        else:
            # 负数速度(后退)
            high_byte = (speed_mm_s >> 8) & 0xFF  # 取高8位
            low_byte = speed_mm_s & 0xFF          # 取低8位

        return high_byte, low_byte # 表示指令高字节和低字节的两个字节
    
    def decode_speed(self,high_byte, low_byte):
        # 组合高字节和低字节
        combined_value = (high_byte << 8) | low_byte

        # 判断是正数还是负数
        if high_byte & 0x80:  # 如果高字节的最高位为1，表示负数
            speed_mm_s = combined_value - 0x10000
        else:
            speed_mm_s = combined_value

        return speed_mm_s
    
    
    def test(self):
        # 测试方法，用于模拟控制电机的操作流程
        while rclpy.ok():
            msg = Twist()
            # msg.linear.x = 0.07
            msg.linear.z = 0.2
            self.control_motor(msg)
            self.rate.sleep()
            
    
def main(args=None):
    rclpy.init(args=args)
    node = CarControlAndFeedback()
    
    # test_thread = threading.Thread(target=node.test)
    # test_thread.start()

    test_thread_2 = threading.Thread(target=node.read_data)
    test_thread_2.start()
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
