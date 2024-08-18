from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import rclpy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist

class get_diagnostics(Node):
 
    def __init__(self):
        super().__init__("get_diagnostics")
        self.topic_sub = self.create_subscription(DiagnosticArray,"/diagnostics",self.accept_callback,10)


    def accept_callback(self,msg:DiagnosticArray):
        self.get_logger().info(f"{msg}\n\n")
        

def main(args=None):
    rclpy.init(args=args)
    node = get_diagnostics()
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







