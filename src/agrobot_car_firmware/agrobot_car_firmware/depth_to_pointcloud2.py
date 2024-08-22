import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pcl2
import pyrealsense2 as rs
import numpy as np
import threading

class DepthToPointCloud2(Node):
    def __init__(self):
        """
        初始化DepthToPointCloud2节点,负责从RealSense相机接收深度数据并发布为PointCloud2消息。
        """
        super().__init__('depth_to_pointcloud2_node')
        
        # 配置RealSense流
        self.pipeline = rs.pipeline()  # 创建RealSense数据管道
        config = rs.config()  # 创建配置对象

        # 启用深度流，配置分辨率为640x480，帧率为15fps
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)

        # 启动管道以开始接收数据
        self.pipeline.start(config)
        
        # 创建一个ROS2发布者，用于发布PointCloud2消息
        self.pointcloud_pub = self.create_publisher(PointCloud2, 'pointcloud', 10, callback_group=ReentrantCallbackGroup())
        
        # 创建一个回调组以支持多线程回调
        self.callback_group = ReentrantCallbackGroup()

        # 控制是否使用事件锁来同步数据接收和发布。True表示使用锁，False表示不使用锁。
        self.use_lock = False

        # 创建事件锁对象（如果需要）
        self.data_lock = threading.Lock() if self.use_lock else None
        self.depth_frame = None  # 用于存储最新的深度帧数据

        # 启动接收数据的线程
        self.data_thread = threading.Thread(target=self.receive_data)
        self.data_thread.start()

        # 创建一个定时器，每0.1秒调用一次publish_pointcloud函数以发布PointCloud2消息
        self.timer = self.create_timer(0.1, self.publish_pointcloud, callback_group=self.callback_group)

    def receive_data(self):
        """
        从RealSense相机接收深度数据。此函数在独立线程中运行,持续更新self.depth_frame变量。
        """
        while rclpy.ok():  # 保持循环运行直到ROS2系统关闭
            # 获取新的帧
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()

            if depth_frame:
                # 根据是否使用锁的选择，更新深度数据
                if self.use_lock:
                    with self.data_lock:
                        self.depth_frame = depth_frame
                else:
                    self.depth_frame = depth_frame

    def publish_pointcloud(self):
        """
        发布PointCloud2消息。此函数从self.depth_frame生成点云数据并发布。
        """
        # 如果使用锁，确保在访问深度数据时锁住它
        if self.use_lock:
            with self.data_lock:
                if self.depth_frame is None:
                    return  # 如果没有深度数据，跳过这次发布
                depth_frame = self.depth_frame
        else:
            # 不使用锁时，直接使用最新的深度数据
            if self.depth_frame is None:
                return
            depth_frame = self.depth_frame  # 使用上一个接收到的数据

        # 将深度帧转换为numpy数组
        depth_array = np.asanyarray(depth_frame.get_data())
        depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics  # 获取相机内参

        # 生成像素坐标网格
        h, w = depth_intrinsics.height, depth_intrinsics.width
        i, j = np.meshgrid(np.arange(w), np.arange(h), indexing='xy')

        # 将像素坐标和深度值打平以便于处理
        i = i.flatten()
        j = j.flatten()
        depth_array = depth_array.flatten() * 0.001  # 深度值从毫米转换为米

        # 过滤掉无效的深度值（深度为0表示无效点）
        valid_depth_mask = depth_array > 0
        i = i[valid_depth_mask]
        j = j[valid_depth_mask]
        depth_array = depth_array[valid_depth_mask]

        # 将像素坐标转换为3D点云坐标
        points = np.stack([i, j], axis=-1)
        points_3d = np.array([rs.rs2_deproject_pixel_to_point(depth_intrinsics, point, depth) for point, depth in zip(points, depth_array)])

        # 调整坐标轴以匹配ROS的坐标系：RealSense的x -> ROS的y, RealSense的y -> ROS的z, RealSense的z -> ROS的x
        points_3d_ros = np.zeros_like(points_3d)
        points_3d_ros[:, 0] = points_3d[:, 2]  # ROS的x = 深度图的z
        points_3d_ros[:, 1] = -points_3d[:, 0]  # ROS的y = -深度图的x
        points_3d_ros[:, 2] = -points_3d[:, 1]  # ROS的z = -深度图的y  (反转 z 轴)

        # 创建PointCloud2消息
        header = Header()
        header.stamp = self.get_clock().now().to_msg()  # 设置消息的时间戳
        header.frame_id = "camera_link"  # 设置消息的坐标系

        # 定义点云消息的字段（x, y, z）
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        # 创建PointCloud2消息
        pointcloud_msg = pcl2.create_cloud(header, fields, points_3d_ros)
        
        # 发布点云消息
        self.pointcloud_pub.publish(pointcloud_msg)

    def destroy_node(self):
        """
        销毁节点前停止RealSense管道和数据线程。
        """
        self.pipeline.stop()  # 停止RealSense数据流
        self.data_thread.join()  # 等待数据线程结束
        super().destroy_node()  # 调用父类的销毁方法以确保正确清理资源

def main(args=None):
    rclpy.init(args=args)  
    node = DepthToPointCloud2()  
    executor = MultiThreadedExecutor()  # 创建多线程执行器
    executor.add_node(node)  # 将节点添加到执行器中

    try:
        executor.spin()  # 开始执行器，处理所有的回调函数
    finally:
        node.destroy_node()  # 在退出时销毁节点
        rclpy.shutdown()  # 关闭rclpy库

if __name__ == '__main__':
    main()
