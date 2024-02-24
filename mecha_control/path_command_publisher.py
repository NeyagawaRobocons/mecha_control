#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mecha_control.msg import PointAndMechaState, PointAndMechaStateArray, MechaState

class PointAndMechaStateArrayPublisher(Node):
    def __init__(self):
        super().__init__('point_and_mecha_state_array_publisher')
        self.publisher_ = self.create_publisher(PointAndMechaStateArray, 'point_command', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('point_and_mecha_state_array_publisher has been started')

    def timer_callback(self):
        msg_array = PointAndMechaStateArray()

        """
        define points for test:
            [0.0, 0.0], [0.0, 1.0], [1.2, 1.0], [1.2, 2.0]
        now, the angle is 0.0 for all points for test.
        """

        points = [[0.0, 0.0], [0.0, 1.0], [1.2, 1.0], [1.2, 2.0]]

        for point in points:
            point_msg = PointAndMechaState()
            point_msg.x = point[0]
            point_msg.y = point[1]
            point_msg.angle = 0.0

            command_msg = MechaState()
            if point == [1.2, 1.0]:
                command_msg.daiza_state = bytes([1])
            else:
                command_msg.daiza_state = bytes([0]) 
            command_msg.hina_state = bytes([0])
            command_msg.bonbori_state = False

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