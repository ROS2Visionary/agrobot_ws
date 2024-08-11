import rclpy
from rclpy.node import Node
from subprocess import call
import os
from pathlib import Path
import shutil

class map_saver(Node):

    def __init__(self):
        super().__init__('map_saver')


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

            # 创建一个定时器，每隔指定的时间间隔调用一次save_map方法
            self.timer = self.create_timer(self.time_interval, self.save_map)

            self.index = 0  # 地图文件索引，用于区分不同时间保存的地图文件

    def save_map(self):
        self.index += 1  # 增加文件索引
        try:
            # 调用外部命令保存地图
            call(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-t', 'map', '-f', f"{self.map_file_name}_{self.index}"])
        except BaseException:
            pass  # 忽略所有异常

def main(args=None):
    rclpy.init(args=args)
    node = map_saver()
    try:
        rclpy.spin(node)  # 保持节点运行
    except KeyboardInterrupt:
        # 处理 Ctrl-C 中断
        pass
    finally:
        # 在节点销毁前调用清理方法
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
