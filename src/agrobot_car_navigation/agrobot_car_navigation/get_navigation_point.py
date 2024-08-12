from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import rclpy
import numpy as np
from geometry_msgs.msg import PoseStamped
import os
import math

class get_navigation_point(Node):

    def __init__(self):
        super().__init__("get_navigation_point")
        self.topic_sub = self.create_subscription(PoseStamped,"/goal_pose",self.accept_callback,10)
        self.index = 0
        self.goal_pose_list = []
        # 指定保存的文件名
        self.file_path = "/root/ros2_projects/simulation_agrobot_ws/src/agrobot_car_navigation/nav_waypoint.txt"

        # 检查文件是否存在
        if os.path.exists(self.file_path):
            # 如果文件存在，删除它
            os.remove(self.file_path)


    def accept_callback(self,msg:PoseStamped):
        print(f"\n位置: {msg.pose.position}\n姿态: {msg.pose.orientation}")

        goal_pose_str = (
                        f"goal_pose_{self.index} = PoseStamped()\n"
                        f"goal_pose_{self.index}.header.frame_id = 'map'\n"
                        f"goal_pose_{self.index}.pose.position.x = {msg.pose.position.x}\n"
                        f"goal_pose_{self.index}.pose.position.y = {msg.pose.position.y}\n"
                        f"goal_pose_{self.index}.pose.orientation.z = {msg.pose.orientation.z}\n"
                        f"goal_pose_{self.index}.pose.orientation.w = {msg.pose.orientation.w}"
                        )
        
        if self.index == 0:
            theta_radians, theta_degrees = self.quaternion_to_angle(msg.pose.orientation.z,msg.pose.orientation.w)
            map_start_pose = f"map_start_pose : [{msg.pose.position.x},{msg.pose.position.y},{theta_radians}]"
            # 打开文件并追加一行
            with open(self.file_path, 'a') as file:
                file.write(map_start_pose + '\n\n')  # 追加上换行符
                file.close()

        self.index += 1
        self.goal_pose_list.append(goal_pose_str)

        # 打开文件并追加一行
        with open(self.file_path, 'a') as file:
            file.write(goal_pose_str + '\n\n')  # 追加上换行符
            file.close()

    # 将二维四元数转换为旋转角度。
    def quaternion_to_angle(self,z,w):
        # 计算旋转角度（弧度）
        theta_radians = 2 * math.atan2(z, w)
        # 将角度从弧度转换为度数
        theta_degrees = math.degrees(theta_radians)
        return theta_radians, theta_degrees

def main(args=None):
    rclpy.init(args=args)
    node = get_navigation_point()
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
