#!/usr/bin/env python3
"""
# mecha_control/msg/MechaState.msg
# 1: 展開
# 2: 回収
# 3: 設置
# false: ぼんぼりオフ
# true: ぼんぼり点灯
byte daiza_state
byte hina_state
bool bonbori_state
"""

import rclpy
from rclpy.node import Node
from mecha_control.msg import MechaState

class MechaControlNode(Node):
    def __init__(self):
        super().__init__('mecha_control_node')
        # メカ制御の状態をsubscribe
        self.subscription_ = self.create_subscription(
            MechaState, '/mecha_state', self.mecha_state_callback, 10)
        
        self.subscription_  # prevent unused variable warning

    def mecha_state_callback(self, msg):
        # # メカ制御の状態を表示
        # self.get_logger().info(f'daiza_state: {msg.daiza_state}')
        # self.get_logger().info(f'hina_state: {msg.hina_state}')
        # self.get_logger().info(f'bonbori_state: {msg.bonbori_state}')
        if msg.daiza_state == bytes([1]):
            self.daiza_tenkai()
        elif msg.daiza_state == bytes([2]):
            self.daiza_kaishu()
        elif msg.daiza_state == bytes([3]):
            self.daiza_setti()
        if msg.hina_state == bytes([1]):
            self.hina_tenkai()
        elif msg.hina_state == bytes([2]):
            self.hina_kaishu()
        elif msg.hina_state == bytes([3]):
            self.hina_setti()
        if msg.bonbori_state == True:
            self.bonbori_tento()

    def daiza_tenkai(self):
        self.get_logger().info('daiza_tenkai')
        
    
    def daiza_kaishu(self):
        self.get_logger().info('daiza_kaishu')

    def daiza_setti(self):
        self.get_logger().info('daiza_setti')
    
    def hina_tenkai(self):
        self.get_logger().info('hina_tenkai')

    def hina_kaishu(self):
        self.get_logger().info('hina_kaishu')

    def hina_setti(self):
        self.get_logger().info('hina_setti')

    def bonbori_tento(self):
        self.get_logger().info('bonbori_tento')

def main(args=None):
    rclpy.init(args=args)
    mecha_control_node = MechaControlNode()
    rclpy.spin(mecha_control_node)
    # シャットダウン処理
    mecha_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()