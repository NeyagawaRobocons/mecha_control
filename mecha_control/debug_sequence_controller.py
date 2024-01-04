#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mecha_control.msg import MechaState
import PySimpleGUI as sg

class DebugSequenceController(Node):
    def __init__(self):
        super().__init__('debug_sequence_controller')
        self.publisher = self.create_publisher(MechaState, '/mecha_state', 10)

        # GUI Layout
        layout = [
            [sg.Text('台座機構コントロール')],
            [sg.Button('台座展開', key='daiza_tenkai'), sg.Button('台座回収', key='daiza_kaishu'), sg.Button('台座設置', key='daiza_setti')],
            [sg.Text('人形機構コントロール')],
            [sg.Button('人形準備', key='hina_junbi'), sg.Button('人形展開', key='hina_tenkai'), sg.Button('人形回収', key='hina_kaishu'), sg.Button('人形設置', key='hina_setti')],
            [sg.Text('ぼんぼり点灯コントロール')],
            [sg.Button('ぼんぼり点灯開始', key='bonbori_tento')]
        ]

        # Create the window
        self.window = sg.Window('デバッグ用シーケンスコントローラ', layout)

    def run(self):
        initial_msg = MechaState()
        initial_msg.daiza_state = bytes([0])
        initial_msg.hina_state = bytes([0])
        initial_msg.bonbori_state = False
        self.publisher.publish(initial_msg)
        while True:
            event, values = self.window.read()

            # if user closes window or clicks cancel
            if event == sg.WIN_CLOSED:
                break

            # Create MechaState message
            msg = MechaState()
            if event == 'daiza_tenkai':
                msg.daiza_state = bytes([1])  # 展開
            elif event == 'daiza_kaishu':
                msg.daiza_state = bytes([2])  # 回収
            elif event == 'daiza_setti':
                msg.daiza_state = bytes([3])  # 設置
            elif event == 'hina_junbi':
                msg.hina_state = bytes([4])   # 準備
            elif event == 'hina_tenkai':
                msg.hina_state = bytes([1])   # 展開
            elif event == 'hina_kaishu':
                msg.hina_state = bytes([2])   # 回収
            elif event == 'hina_setti':
                msg.hina_state = bytes([3])   # 設置
            elif event == 'bonbori_tento':
                msg.bonbori_state = True      # 点灯開始

            # Publish the message
            self.publisher.publish(msg)

        self.window.close()

def main(args=None):
    rclpy.init(args=args)
    controller = DebugSequenceController()
    controller.run()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
