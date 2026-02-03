#!/usr/bin/env python3
"""
连续触发雷达采集服务的脚本
按指定时间间隔请求服务
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import argparse
import time
from typing import Optional


class ServiceTrigger(Node):
    def __init__(self, service_name: str, interval: float, count: Optional[int] = None):
        """
        初始化服务触发器
        
        Args:
            service_name: 服务名称，例如 '/collect_BAD8I539'
            interval: 触发间隔时间（秒）
            count: 触发次数，None表示无限循环
        """
        super().__init__('service_trigger_client')
        self.service_name = service_name
        self.interval = interval
        self.count = count
        self.trigger_count = 0
        
        # 创建服务客户端
        self.client = self.create_client(Trigger, service_name)
        
        self.get_logger().info(f'初始化服务触发器，服务名: {service_name}')
        self.get_logger().info(f'触发间隔: {interval}秒，触发次数: {count if count else "无限"}')
    
    def wait_for_service(self, timeout_sec: float = 5.0) -> bool:
        """
        等待服务可用
        
        Args:
            timeout_sec: 超时时间（秒）
            
        Returns:
            服务是否可用
        """
        self.get_logger().info(f'等待服务 {self.service_name} 可用...')
        if self.client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().info(f'服务 {self.service_name} 已就绪')
            return True
        else:
            self.get_logger().error(f'服务 {self.service_name} 超时未响应')
            return False
    
    def call_service(self) -> bool:
        """
        调用服务
        
        Returns:
            调用是否成功
        """
        request = Trigger.Request()
        self.trigger_count += 1
        
        self.get_logger().info(f'[{self.trigger_count}] 发送触发请求...')
        
        try:
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.done():
                response = future.result()
                if response.success:
                    self.get_logger().info(
                        f'[{self.trigger_count}] ✓ 触发成功: {response.message}'
                    )
                else:
                    self.get_logger().warn(
                        f'[{self.trigger_count}] ✗ 触发失败: {response.message}'
                    )
                return response.success
            else:
                self.get_logger().error(f'[{self.trigger_count}] 服务调用超时')
                return False
                
        except Exception as e:
            self.get_logger().error(f'[{self.trigger_count}] 调用异常: {str(e)}')
            return False
    
    def run(self):
        """运行触发循环"""
        # 等待服务可用
        if not self.wait_for_service():
            return
        
        # 开始触发循环
        self.get_logger().info('=' * 60)
        self.get_logger().info('开始定时触发服务')
        self.get_logger().info('=' * 60)
        
        try:
            while rclpy.ok():
                # 调用服务
                self.call_service()
                
                # 检查是否达到指定次数
                if self.count is not None and self.trigger_count >= self.count:
                    self.get_logger().info(f'已完成 {self.count} 次触发，退出')
                    break
                
                # 等待指定间隔
                self.get_logger().info(f'等待 {self.interval} 秒后进行下一次触发...')
                time.sleep(self.interval)
                
        except KeyboardInterrupt:
            self.get_logger().info('\n收到中断信号，停止触发')
        finally:
            self.get_logger().info(f'总共触发 {self.trigger_count} 次')


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description='连续触发ROS2服务的脚本',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 每5秒触发一次 BAD8I539 雷达的采集服务（无限循环）
  python3 trigger.py -s /collect_BAD8I539 -i 5
  
  # 每10秒触发一次，总共触发3次
  python3 trigger.py -s /collect_BAD8I539 -i 10 -c 3
  
  # 每2秒触发一次 BXXXXX19 雷达
  python3 trigger.py -s /collect_BXXXXX19 -i 2
        """
    )
    
    parser.add_argument(
        '-s', '--service',
        type=str,
        required=True,
        help='服务名称，例如: /collect_BAD8I539'
    )
    
    parser.add_argument(
        '-i', '--interval',
        type=float,
        default=5.0,
        help='触发间隔时间（秒），默认5秒'
    )
    
    parser.add_argument(
        '-c', '--count',
        type=int,
        default=None,
        help='触发次数，不指定则无限循环'
    )
    
    args = parser.parse_args()
    
    # 初始化ROS2
    rclpy.init()
    
    try:
        # 创建并运行触发器
        trigger = ServiceTrigger(
            service_name=args.service,
            interval=args.interval,
            count=args.count
        )
        trigger.run()
    finally:
        # 清理资源
        rclpy.shutdown()


if __name__ == '__main__':
    main()
