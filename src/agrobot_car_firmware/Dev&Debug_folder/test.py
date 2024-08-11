import binascii
from struct import pack
from rclpy.node import Node
from geometry_msgs.msg import Twist
import rclpy
import threading

class test(Node):

    def __init__(self):
        super().__init__("test")

        self.vel_pub = self.create_publisher(Twist,"cmd_vel",10)
        self.rate = self.create_rate(2)
    
    def send_twist(self):

        while rclpy.ok():
            msg = Twist()
            msg.linear.x = 0.1
            # msg.linear.z = 0.1
            self.vel_pub.publish(msg)
            self.rate.sleep()


def main(args=None):
    rclpy.init(args=args)
    node = test()
    try:
        thread = threading.Thread(target=node.send_twist)
        thread.start()
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 处理 Ctrl-C 中断
        pass
    finally:
        # 在节点销毁前调用清理方法
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":

    main()
