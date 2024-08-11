import rclpy
from rclpy.node import Node
from slam_toolbox.srv import SerializePoseGraph
import os
import shutil

class sequence_map_saver(Node):
    '''
    用于保存slam_toolbox的序列化地图(每个地图包含两个文件.posegraph 和 .data )
    '''
    def __init__(self):
        super().__init__('sequence_map_saver')

        # 声明并获取参数
        self.declare_parameter("time_interval", 10.0)  # 保存地图的时间间隔，默认为10秒
        self.declare_parameter("save_dir_path", "")  # 地图保存路径，默认为default_save_path

        # 获取参数值
        self.time_interval = self.get_parameter("time_interval").get_parameter_value().double_value
        self.save_dir_path = self.get_parameter("save_dir_path").get_parameter_value().string_value

        # 如果保存路径存在，则删除该目录及其内容，然后重新创建该目录
        if os.path.exists(self.save_dir_path):
            shutil.rmtree(self.save_dir_path)
            os.makedirs(self.save_dir_path)

            # 设置地图文件名前缀
            self.map_file_name = os.path.join(self.save_dir_path, "map")

            self.client = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        
            # 等待服务可用
            while not self.client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('服务尚未可用，请等待...')
        
            # 创建一个定时器，每隔指定时间调用一次 save_map 方法
            self.timer = self.create_timer(self.time_interval, self.save_map)
        
            self.index = 1

    def save_map(self):
        self.get_logger().info('尝试保存地图...')
        request = SerializePoseGraph.Request()
        request.filename = self.map_file_name + str(self.index)

        future = self.client.call_async(request)
        future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            response = future.result()
            self.index += 1
            self.get_logger().info('地图保存成功：%s' % response)
        except Exception as e:
            self.get_logger().error('保存地图失败：%r' % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = sequence_map_saver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 处理 Ctrl-C 中断
        pass
    finally:
        # 在节点销毁前调用清理方法
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
