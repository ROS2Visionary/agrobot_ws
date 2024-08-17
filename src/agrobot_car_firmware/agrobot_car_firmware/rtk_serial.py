import rclpy
from rclpy.node import Node
import serial
import pynmea2
import binascii

class gps_serial(Node):

    def __init__(self):
        super().__init__('nmea_serial_node')
        
        # 配置串口
        self.serial_port = '/dev/ttyUSB0'  # 根据实际情况调整
        self.baud_rate = 115200  # 根据设备配置调整
        self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
        
        self.get_logger().info(f"连接到串口 {self.serial_port}，波特率 {self.baud_rate}")
        
        # # 定时器周期 (以秒为单位)
        # timer_period = 1.0
        # self.timer = self.create_timer(timer_period, self.read_and_parse_serial)

    def read_and_parse_serial(self):

        while True:
            try:
                # 读取一行NMEA数据
                line = self.ser.readline().decode('ascii', errors='replace').strip()
                if line.startswith('$GNGGA') or line.startswith('$GNRMC'):
                    msg = pynmea2.parse(line)
                    if line.startswith('$GNGGA'):
                        self.parse_gngga(msg)
                    elif line.startswith('$GNRMC'):
                        self.parse_gnrmc(msg)
            except serial.SerialException as e:
                self.get_logger().error(f"串口错误: {e}")
            except pynmea2.ParseError as e:
                self.get_logger().error(f"解析错误: {e}")

    def parse_gngga(self, msg):
        self.get_logger().info("GNGGA 语句解析结果:")
        self.get_logger().info(f"  时间(UTC): {msg.timestamp}")
        self.get_logger().info(f"  纬度: {msg.latitude} {msg.lat_dir}")
        self.get_logger().info(f"  经度: {msg.longitude} {msg.lon_dir}")
        self.get_logger().info(f"  定位质量: {msg.gps_qual} (1=标准定位, 4=RTK固定解, 5=RTK浮动解)")
        self.get_logger().info(f"  卫星数量: {msg.num_sats}")
        self.get_logger().info(f"  水平精度因子(HDOP): {msg.horizontal_dil}")
        self.get_logger().info(f"  海拔高度: {msg.altitude} {msg.altitude_units}")
        self.get_logger().info("\r\n")

    def parse_gnrmc(self, msg):
        self.get_logger().info("GNRMC 语句解析结果:")
        self.get_logger().info(f"  时间(UTC): {msg.timestamp}")
        self.get_logger().info(f"  定位状态: {msg.status} (A=有效定位, V=无效定位)")
        self.get_logger().info(f"  纬度: {msg.latitude} {msg.lat_dir}")
        self.get_logger().info(f"  经度: {msg.longitude} {msg.lon_dir}")
        self.get_logger().info(f"  速度: {msg.spd_over_grnd} 节")
        self.get_logger().info(f"  航向: {msg.true_course} 度")
        self.get_logger().info(f"  日期: {msg.datestamp}")

def main(args=None):
    rclpy.init(args=args)
    node = gps_serial()
    node.read_and_parse_serial()
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
