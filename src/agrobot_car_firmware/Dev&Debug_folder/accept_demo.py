import serial
import rclpy
import sys
import rclpy.clock
from rclpy.node import Node
import random
import string
import time
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math
import numpy as np
from struct import unpack
import binascii
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from struct import unpack
   
class motor_control(Node):
    
    def __init__(self):
        super().__init__("motor_control")
        
        self.ser = serial.Serial("/dev/ttyACM1", 115200, timeout=5.0)

        # 创建里程计数据发布者
        self.pub_odom = self.create_publisher(Odometry, '/odom/data', 10)


                  
    def read_data(self):
        while self.ser.is_open:

            if self.ser.in_waiting:
                head = binascii.b2a_hex(self.ser.read(1))
                if head == b"7b":
                    self.ser.read(1)
                    
                    x = self.decode_speed(unpack("B",self.ser.read(1))[0],unpack("B",self.ser.read(1))[0])
                    self.ser.read(2) # 暂不引入y方向的速度,因为这履带车的运动模型原因y方向的速度永远为0
                    # y = self.decode_speed(unpack("B",self.ser.read(1))[0],unpack("B",self.ser.read(1))[0])
                    z = self.decode_speed(unpack("B",self.ser.read(1))[0],unpack("B",self.ser.read(1))[0])

                    self.ser.read(6) # 暂不引入x、y、z轴加速度，因为小车为履带车，只会在较低的速度下运行，而且小车加速的时间较短，并且控制板自带的imu精度较差
                    # x_acceleration = self.decode_speed(unpack("B",self.ser.read(1))[0],unpack("B",self.ser.read(1))[0]) / 1672
                    # y_acceleration = self.decode_speed(unpack("B",self.ser.read(1))[0],unpack("B",self.ser.read(1))[0]) / 1672
                    # z_acceleration = self.decode_speed(unpack("B",self.ser.read(1))[0],unpack("B",self.ser.read(1))[0]) / 1672

                    self.ser.read(4) # 暂不引入x、y轴角速度，因为在平地上运行的小车，可以忽视x、y轴的角速度
                    z_angular_vel = self.decode_speed(unpack("B",self.ser.read(1))[0],unpack("B",self.ser.read(1))[0]) / 3753
                    
                    self.ser.read(3) # 忽视电压和校验位

                    if binascii.b2a_hex(self.ser.read(1)) == b"7d": # 通过帧尾来判断数据是否有误
                        print(f"{x} - {z} - {z_angular_vel}")


                    
    def decode_speed(self,high_byte, low_byte):
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
    node = motor_control()
    try:
        node.read_data()
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 处理 Ctrl-C 中断
        linear = np.array(node.linear_arr)
        linear_mear = np.mean(linear,axis=0)
        linear_mear[2] = linear_mear[2] - 9.81
        print(f"{linear_mear}")
        pass
    finally:
        # 在节点销毁前调用清理方法
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    
    main()
        











