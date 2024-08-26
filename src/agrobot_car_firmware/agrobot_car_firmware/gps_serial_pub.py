
import rclpy
from rclpy.node import Node
import serial
import pynmea2
from sensor_msgs.msg import NavSatFix, NavSatStatus

class gps_serial_pub(Node):

    def __init__(self):
        super().__init__('gps_serial_publisher')
        
        # 配置串口
        self.serial_port = '/dev/ttyUSB0'  # 串口路径,根据实际情况调整
        self.baud_rate = 115200  # 串口波特率,根据设备配置调整
        self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)  # 初始化串口
        
        self.get_logger().info(f"连接到串口 {self.serial_port},波特率 {self.baud_rate}")
        
        # 创建GPS数据发布者
        self.navsatfix_pub = self.create_publisher(NavSatFix, 'gps/fix', 10)  # 发布NavSatFix消息
        
        # 设置定时器周期 (以秒为单位)
        timer_period = 1.0  # 每秒读取一次GPS数据
        self.timer = self.create_timer(timer_period, self.read_and_publish_gps_data)

    def read_and_publish_gps_data(self):
        """读取串口数据并解析NMEA语句,发布到'gps/fix'话题"""
        try:
            # 读取一行NMEA数据
            line = self.ser.readline().decode('ascii', errors='replace').strip()
            if line.startswith('$GNGGA'):
                msg = pynmea2.parse(line)  # 解析NMEA语句
                self.publish_navsatfix(msg)  # 发布NavSatFix消息
        except serial.SerialException as e:
            self.get_logger().error(f"串口错误: {e}")
        except pynmea2.ParseError as e:
            self.get_logger().error(f"解析错误: {e}")

    def publish_navsatfix(self, msg):
        """解析GNGGA语句并发布NavSatFix消息"""
        navsatfix_msg = NavSatFix()
        
        # 填充NavSatFix消息头
        navsatfix_msg.header.stamp = self.get_clock().now().to_msg()  # 设置时间戳
        navsatfix_msg.header.frame_id = 'gps'  # 坐标系名称
        
        # 设置GPS状态信息
        navsatfix_msg.status.status = int(msg.gps_qual)  # GPS质量状态
        navsatfix_msg.status.service = NavSatStatus.SERVICE_GPS  # 使用的服务类型
        
        # 设置位置信息
        navsatfix_msg.latitude = float(msg.latitude)  # 纬度
        navsatfix_msg.longitude = float(msg.longitude)  # 经度
        navsatfix_msg.altitude = float(msg.altitude)  # 高度
        
        # 发布NavSatFix消息
        self.navsatfix_pub.publish(navsatfix_msg)
        self.get_logger().info(f"发布GPS位置: {navsatfix_msg.latitude}, {navsatfix_msg.longitude}, {navsatfix_msg.altitude}")

def main(args=None):
    rclpy.init(args=args)
    node = GpsSerialPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
