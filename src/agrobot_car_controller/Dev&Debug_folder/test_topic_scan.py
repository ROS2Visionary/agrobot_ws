from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import rclpy
import numpy as np

class test_liser(Node):

    def __init__(self):
        super().__init__("test_liser")

        self.liser_sub = self.create_subscription(LaserScan,"/raw_scan",self.accept_liser,10)
        self.liser_pub = self.create_publisher(LaserScan,"/scan",10)


    def accept_liser(self,msg:LaserScan):
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "liser_link"

        # count = (msg.angle_max - msg.angle_min) / msg.angle_increment
        # print(f"{count} - {len(msg.ranges)}")

        msg.angle_increment = (msg.angle_max - msg.angle_min) / 530

        # count = (msg.angle_max - msg.angle_min) / msg.angle_increment
        # print(f"{count} - {len(msg.ranges)}")

        if len(msg.ranges) > 530:
            for _ in range(len(msg.ranges) - 530):
                del msg.ranges[265]
                del msg.intensities[265]
        if len(msg.ranges) < 530:
            for _ in range(530 -len(msg.ranges)):
                msg.ranges.insert(256, float("inf"))
                msg.intensities.insert(256, 0)

        # print(f"{count} - {len(msg.ranges)}")

        self.liser_pub.publish(msg)



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






