import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import Header
import time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformListener, Buffer
from enum import Enum
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import threading

class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3

class multi_point_navigator(BasicNavigator):
    def __init__(self):
        super().__init__()
        self.waypoints = []

        self.amcl_pose_group = MutuallyExclusiveCallbackGroup()

        self.amcl_pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 10,callback_group=self.amcl_pose_group)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        

        self.timer = None

        self.init_robot_pose()
        
        # 阻塞直到Nav2完全在线并且生命周期节点处于活动状态
        self.waitUntilNav2Active()
        

    def start_timer(self):
        if self.timer is None:
            self.timer = threading.Timer(3.0,self.get_navigation_progress)
            self.timer.start()
            self.get_logger().info('计时器已启动')

    def stop_timer(self):
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
            self.get_logger().info('计时器已暂停')


    def init_robot_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        
        # 设置初始位置的位置信息和方向信息
        pose.pose.position.x = 7.631  # 根据实际需要设置
        pose.pose.position.y = -5.072  # 根据实际需要设置
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.7069657693523876
        pose.pose.orientation.w = 0.707247764905614

        self.setInitialPose(pose)

    # AMCL初始位置回调函数
    def amcl_pose_callback(self, msg:PoseWithCovarianceStamped):
        self.get_logger().info(f"\n接收到AMCL位置: {msg.pose.pose.position}\n姿态: {msg.pose.pose.orientation}")
        

    def get_navigation_progress(self):

        feedback = self.getFeedback()
        if feedback:
            self.get_logger().info(f'当前位置: ({feedback.current_pose.pose.position.x}, {feedback.current_pose.pose.position.y}), '
                                    f'预计剩余时间: {feedback.estimated_time_remaining.sec + feedback.estimated_time_remaining.nanosec / 1e9} 秒, '
                                    f'剩余距离: {feedback.distance_remaining} 米')
        if self.isTaskComplete():
            result = self.getResult()
            if result.value == TaskResult.SUCCEEDED.value:
                self.get_logger().info('导航成功')
            elif result.value == TaskResult.CANCELED.value:
                self.get_logger().info('导航取消!')
            elif result.value == TaskResult.FAILED.value:
                self.get_logger().info('导航失败!')
            elif result.value == TaskResult.UNKNOWN.value:
                self.get_logger().info('目标具有无效的返回状态!')
                

    def test_goThroughPoses(self):
        # # 创建一些导航点（这里假设已经有导航点的位置信息）
        waypoint1 = PoseStamped()
        waypoint1.header.frame_id = 'map'
        waypoint1.header.stamp = self.get_clock().now().to_msg()
        waypoint1.pose.position.x = 7.681
        waypoint1.pose.position.y = -2.859
        waypoint1.pose.orientation.z = 0.7069644079795863
        waypoint1.pose.orientation.w = 0.707249125732986

        waypoint2 = PoseStamped()
        waypoint2.header.frame_id = 'map'
        waypoint2.header.stamp = self.get_clock().now().to_msg()
        waypoint2.pose.position.x = 6.987
        waypoint2.pose.position.y = -1.336
        waypoint2.pose.orientation.z = 0.9715529033344713
        waypoint2.pose.orientation.w = 0.2368226256554883

        waypoint3 = PoseStamped()
        waypoint3.header.frame_id = 'map'
        waypoint3.header.stamp = self.get_clock().now().to_msg()
        waypoint3.pose.position.x = 3.928
        waypoint3.pose.position.y = -0.729
        waypoint3.pose.orientation.z = 0.9999705514724166
        waypoint3.pose.orientation.w = 0.0076743851838934115

        waypoint4 = PoseStamped()
        waypoint4.header.frame_id = 'map'
        waypoint4.header.stamp = self.get_clock().now().to_msg()
        waypoint4.pose.position.x = 0.555
        waypoint4.pose.position.y = -0.239
        waypoint4.pose.orientation.z = 0.8995992576736566
        waypoint4.pose.orientation.w = 0.4367163559943755

        # 开始导航
        self.goThroughPoses([waypoint1,waypoint2,waypoint3,waypoint4])


def main(args=None):
    rclpy.init(args=args)
    node = multi_point_navigator()
    node.test_goThroughPoses()
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
