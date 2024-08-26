import rclpy
from rclpy.node import Node
import math
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
import pyproj  
class gps_odom_pub(Node):

    def __init__(self):
        super().__init__('gps_odom_publisher')
        
        # 创建订阅者，订阅'gps/fix'话题
        self.subscription = self.create_subscription(NavSatFix, 'gps/fix', self.listener_callback, 10)
        
        # 创建发布者，发布'gps/odom'话题
        self.odom_pub = self.create_publisher(Odometry, 'gps/odom', 10)
        
        # 初始化变量，用于存储前一个GPS点的经纬度和初始参考点
        self.prev_easting = None
        self.prev_northing = None
        self.initial_easting = None
        self.initial_northing = None
        
        # 初始化UTM投影
        self.utm_proj = pyproj.Proj(proj='utm', zone=33, ellps='WGS84')  # 需要根据你的GPS位置设置正确的UTM区域

    def listener_callback(self, msg):
        """接收来自'gps/fix'话题的消息，计算并发布'gps/odom'话题"""

        # 判断GPS数据是否可靠
        if msg.status.status != 4:  
            self.get_logger().warn("GPS数据不可靠,忽略当前数据")
            return

        # 将经纬度转换为UTM坐标
        easting, northing = self.utm_proj(msg.longitude, msg.latitude)
        
        # 如果这是第一次接收到GPS数据，保存初始UTM坐标作为参考点
        if self.initial_easting is None or self.initial_northing is None:
            self.initial_easting = easting
            self.initial_northing = northing

        # 计算当前位置相对于初始位置的平面坐标
        delta_x = easting - self.initial_easting  # x方向 (东向)
        delta_y = northing - self.initial_northing  # y方向 (北向)

        # 计算航向角 (Yaw)
        yaw_degrees = 0.0  # 默认航向角为0
        if self.prev_easting is not None and self.prev_northing is not None:
            d_easting = easting - self.prev_easting
            d_northing = northing - self.prev_northing
            yaw_degrees = math.degrees(math.atan2(d_northing, d_easting))
            yaw_degrees = (yaw_degrees + 360) % 360  # 确保角度在 [0, 360) 范围内

        # 创建并发布Odometry消息
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()  # 时间戳
        odom_msg.header.frame_id = 'odom'  # 里程计坐标系
        odom_msg.child_frame_id = 'base_footprint'  # 子坐标系（机器人自身坐标系）
        odom_msg.pose.pose.position.x = delta_x  # x坐标（东向距离）
        odom_msg.pose.pose.position.y = delta_y  # y坐标（北向距离）
        odom_msg.pose.pose.position.z = 0.0  # z坐标（假设平面运动）

        # 将yaw转换为四元数
        q = quaternion_from_euler(0, 0, math.radians(yaw_degrees))
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        # 发布Odometry消息
        self.odom_pub.publish(odom_msg)
        self.get_logger().info(f"发布位置和航向: x={delta_x:.2f}, y={delta_y:.2f}, yaw={yaw_degrees:.2f}度")
        
        # 更新上一个位置
        self.prev_easting = easting
        self.prev_northing = northing

def main(args=None):
    rclpy.init(args=args)
    node = gps_odom_pub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
