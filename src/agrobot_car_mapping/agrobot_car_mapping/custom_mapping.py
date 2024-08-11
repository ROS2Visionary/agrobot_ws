from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
from tf2_ros import Buffer, TransformListener, LookupException
import rclpy
import math
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy

# 定义常量
PRIOR_PROB = 0.5  # 先验概率
OCC_PROB = 0.9    # 占据概率
FREE_PROB = 0.35  # 自由概率

# 定义姿态类
class Pose:
    def __init__(self, px=0, py=0):
        self.x = px
        self.y = py

# 将坐标转换为地图上的姿态（网格单元）
def coordinatesToPose(px, py, map_info: MapMetaData):
    pose = Pose()
    pose.x = round((px - map_info.origin.position.x) / map_info.resolution)
    pose.y = round((py - map_info.origin.position.y) / map_info.resolution)
    return pose

# 检查姿态是否在地图范围内
def poseOnMap(pose: Pose, map_info: MapMetaData):
    return pose.x < map_info.width and pose.x >= 0 and pose.y < map_info.height and pose.y >= 0

# 将姿态转换为占据网格地图中的单元格索引
def poseToCell(pose: Pose, map_info: MapMetaData):
    return map_info.width * pose.y + pose.x

# 使用 Bresenham 算法计算从起点到终点的线段上的所有点
def bresenham(start: Pose, end: Pose):
    line = []

    dx = end.x - start.x
    dy = end.y - start.y

    xsign = 1 if dx > 0 else -1
    ysign = 1 if dy > 0 else -1

    dx = abs(dx)
    dy = abs(dy)

    if dx > dy:
        xx = xsign
        xy = 0
        yx = 0
        yy = ysign
    else:
        tmp = dx
        dx = dy
        dy = tmp
        xx = 0
        xy = ysign
        yx = xsign
        yy = 0

    D = 2 * dy - dx
    y = 0

    for i in range(dx + 1):
        line.append(Pose(start.x + i * xx + y * yx, start.y + i * xy + y * yy))
        if D >= 0:
            y += 1
            D -= 2 * dx
        D += 2 * dy

    return line

# 反向传感器模型：计算机器人位置和测量点之间的概率
def inverseSensorModel(robot_p: Pose, beam_p: Pose):
    occ_values = []
    line = bresenham(robot_p, beam_p)
    for pose in line[:-1]:
        occ_values.append((pose, FREE_PROB))  # 自由空间概率
    occ_values.append((line[-1], OCC_PROB))   # 占据空间概率
    return occ_values

# 概率转换为对数几率
def prob2logodds(p):
    return math.log(p / (1 - p))

# 对数几率转换为概率
def logodds2prob(l):
    try:
        return 1 - (1 / (1 + math.exp(l)))
    except OverflowError:
        return 1.0 if l > 0 else 0.0

class custom_mapping(Node):
    
    def __init__(self, node_name: str):
        super().__init__(node_name)

        # 声明地图参数
        self.declare_parameter("width", 50.0)
        self.declare_parameter("height", 50.0)
        self.declare_parameter("resolution", 0.1)
        
        # 获取地图参数
        width = self.get_parameter("width").value
        height = self.get_parameter("height").value
        resolution = self.get_parameter("resolution").value
        
        # 初始化占据网格地图
        self.map = OccupancyGrid()
        self.map.header.frame_id = "odom"
        self.map.info.resolution = resolution
        self.map.info.width = round(width / resolution)
        self.map.info.height = round(height / resolution)

        self.map.info.origin.position.x = float(-round(width / 2.0))
        self.map.info.origin.position.y = float(-round(height / 2.0))
        self.map.data = [-1] * (self.map.info.width * self.map.info.height)
        
        # 初始化概率地图
        self.probability_map = [prob2logodds(PRIOR_PROB)] * (self.map.info.width * self.map.info.height)
        
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # 创建地图发布器、激光扫描订阅器和定时器
        self.map_pub = self.create_publisher(OccupancyGrid, "map", qos_profile)
        self.scan_sub = self.create_subscription(LaserScan, "scan", self.scan_callback, 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # 初始化 TF2 Buffer 和 TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
    # 激光扫描回调函数
    def scan_callback(self, scan_msg: LaserScan):
        try:
            t = self.tf_buffer.lookup_transform(self.map.header.frame_id, scan_msg.header.frame_id, rclpy.time.Time())
        except LookupException:
            self.get_logger().error("无法在 /odom 和 /base_footprint 之间转换")
            return
    
        # 获取机器人当前位置
        robot_p = coordinatesToPose(t.transform.translation.x, t.transform.translation.y, self.map.info)
        
        # 检查机器人是否在地图范围内
        if not poseOnMap(robot_p, self.map.info):
            self.get_logger().error("机器人已经超出地图范围！")
            return
        
        # 获取机器人当前姿态
        roll, pitch, yaw = euler_from_quaternion([t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w])
        
        # 处理每个激光束的数据
        for i in range(len(scan_msg.ranges)):
            if math.isinf(scan_msg.ranges[i]):
                continue
            
            # 计算激光束的角度和位置
            angle = scan_msg.angle_min + (i * scan_msg.angle_increment) + yaw
            px = scan_msg.ranges[i] * math.cos(angle)
            py = scan_msg.ranges[i] * math.sin(angle)
            px += t.transform.translation.x
            py += t.transform.translation.y
            
            # 将激光束位置转换为地图上的姿态
            beam_p = coordinatesToPose(px, py, self.map.info)
            if not poseOnMap(beam_p, self.map.info):
                continue
            
            # 计算反向传感器模型，更新概率地图
            poses = inverseSensorModel(robot_p, beam_p)
            for pose, value in poses:
                cell_index = poseToCell(pose, self.map.info)
                self.probability_map[cell_index] += prob2logodds(value) - prob2logodds(PRIOR_PROB)
            
        # 可选：标记机器人运动路径
        # robot_cell_index = poseToCell(robot_p, self.map.info)
        # self.map.data[robot_cell_index] = 100
        
    # 定时器回调函数：将概率地图数据转换为发布格式并发布
    def timer_callback(self):
        self.map.data = [int(logodds2prob(value) * 100) for value in self.probability_map]
        self.map.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.map)


def main(args=None):
    rclpy.init(args=args)
    node = custom_mapping("mapping_with_known_poses")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 处理 Ctrl-C 中断
        pass
    finally:
        # 在节点销毁前调用清理方法
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
