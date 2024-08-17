import rclpy
from rclpy.node import Node
from slam_toolbox.srv import LoopClosure

class LoopClosureTrigger(Node):
    def __init__(self):
        super().__init__('loop_closure_trigger')
        
        # 创建一个客户端，用于与slam_toolbox的手动回环检测服务通信
        self.client = self.create_client(LoopClosure, '/slam_toolbox/manual_loop_closure')

        # 等待服务可用，如果服务未启动则每秒输出一次等待信息
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待手动回环闭合服务...')

        # 创建一个定时器，每隔指定时间触发一次回环检测
        self.timer = self.create_timer(1.0, self.trigger_loop_closure)  # 每30秒触发一次回环检测

    def trigger_loop_closure(self):
        # 创建回环检测请求
        request = LoopClosure.Request()
        # 在这里可以设置请求的其他参数（如果有需要）git

        # 异步调用回环检测服务，并在完成后执行回调函数
        future = self.client.call_async(request)
        future.add_done_callback(self.loop_closure_callback)

    def loop_closure_callback(self, future):
        try:
            # 获取服务调用的结果
            response = future.result()
            self.get_logger().info('手动回环检测已成功触发。')
        except Exception as e:
            # 如果服务调用失败，输出错误信息
            self.get_logger().error(f'触发手动回环检测失败：{e}')

def main(args=None):
    # 初始化ROS 2 Python客户端库
    rclpy.init(args=args)
    
    # 创建并运行LoopClosureTrigger节点
    node = LoopClosureTrigger()
    rclpy.spin(node)

    # 在节点销毁前调用清理方法
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
