#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mecha_control.msg import PointAndMechaState, PointAndMechaStateArray, MechaState

class PointAndMechaStateArrayPublisher(Node):
    def __init__(self):
        super().__init__('point_and_mecha_state_array_publisher')
        self.publisher_ = self.create_publisher(PointAndMechaStateArray, 'point_and_mecha_state_array', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('point_and_mecha_state_array_publisher has been started')

    def timer_callback(self):
        msg_array = PointAndMechaStateArray()
        
        # 例として3つのポイントを作成
        for i in range(3):
            point_msg = PointAndMechaState()
            point_msg.x = float(i)
            point_msg.y = float(i)
            point_msg.angle = float(i) * 0.1

            command_msg = MechaState()
            command_msg.daiza_state = bytes([1])  # 展開
            command_msg.hina_state = bytes([2])   # 回収
            command_msg.bonbori_state = True  # ぼんぼり点灯

            point_msg.command = command_msg
            msg_array.points.append(point_msg)

        self.publisher_.publish(msg_array)
        self.get_logger().info('Publishing: "%s"' % msg_array)

def main(args=None):
    rclpy.init(args=args)
    node = PointAndMechaStateArrayPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()