from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import rclpy
import numpy as np
from geometry_msgs.msg import Twist

class test_liser(Node):

    def __init__(self):
        super().__init__("test_liser")

        self.topic_sub = self.create_subscription(Twist,"/cmd_vel",self.accept_callback,10)


    def accept_callback(self,msg:Twist):
        print(msg.linear)
        print(msg.angular)
        

def main(args=None):
    rclpy.init(args=args)
    node = test_liser()
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






