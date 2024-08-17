import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration

class get_navigation_m3(Node):

    def __init__(self):
        super().__init__('robot_pose_subscriber')
        # 创建一个 Buffer 对象，用于存储 tf 变换数据
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        # 创建一个 TransformListener 对象，用于监听 tf 变换
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # 创建一个定时器，每秒调用一次 timer_callback 方法
        self.timer = self.create_timer(2.0, self.timer_callback)

    def timer_callback(self):
        try:
            # 获取当前时间
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'odom', now, Duration(seconds=1.0))
            
            # 提取平移信息
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z

            # 提取旋转信息（四元数）
            qx = trans.transform.rotation.x
            qy = trans.transform.rotation.y
            qz = trans.transform.rotation.z
            qw = trans.transform.rotation.w
            
            # 输出机器人的位置信息和姿态信息
            self.get_logger().info("")
            self.get_logger().info(f'位置: x: {x}, y: {y}, z: {z}')
            self.get_logger().info(f'姿态: qx: {qx}, qy: {qy}, qz: {qz}, qw: {qw}')
        except Exception as e:
            # 如果变换查找失败，输出警告信息
            # self.get_logger().warn(f'Could not transform: {e}')
            pass

def main(args=None):
    # 初始化 ROS 2 Python 客户端库
    rclpy.init(args=args)
    node = get_navigation_m3()
    try:
        # 保持节点运行，直到接收到退出信号
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # 销毁节点
    node.destroy_node()
    # 关闭 ROS 2 客户端库
    rclpy.shutdown()

if __name__ == '__main__':
    main()
