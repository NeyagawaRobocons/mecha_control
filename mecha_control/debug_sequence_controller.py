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
            [sg.Button('台座展開(0)', key='daiza_tenkai'), sg.Button('台座回収(1)', key='daiza_kaishu'), sg.Button('台座設置(2)', key='daiza_setti'), sg.Button('台座格納(3)', key='daiza_kakunou')],
            [sg.Text('人形機構コントロール')],
            [sg.Button('人形展開(4)', key='hina_tenkai'), sg.Button('人形回収(5)', key='hina_kaishu'), sg.Button('人形設置(6)', key='hina_setti'), sg.Button('人形格納(7)', key='hina_kakunou')],
            [sg.Text('ぼんぼり点灯コントロール')],
            [sg.Button('ぼんぼり点灯開始(8)', key='bonbori_tento')]
        ]

        # Create the window
        self.window = sg.Window('デバッグ用シーケンスコントローラ', layout, return_keyboard_events=True)

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
            if event == 'daiza_tenkai' or event == '1:10':
                msg.daiza_state = bytes([1])  # 展開
            elif event == 'daiza_kaishu' or event == '2:11':
                msg.daiza_state = bytes([2])  # 回収
            elif event == 'daiza_setti' or event == '3:12':
                msg.daiza_state = bytes([3])  # 設置
            elif event == 'daiza_kakunou' or event == '4:13':
                msg.daiza_state = bytes([4])  # 格納
            elif event == 'hina_kakunou' or event == '5:14':
                msg.hina_state = bytes([1])   # 展開
            elif event == 'hina_kaishu' or event == '6:15':
                msg.hina_state = bytes([2])   # 回収
            elif event == 'hina_setti' or event == '7:16':
                msg.hina_state = bytes([3])   # 設置
            elif event == 'hina_kakunou' or event == '8:17':
                msg.hina_state = bytes([4])   # 格納
            elif event == 'bonbori_tento' or event == '9:18':
                msg.daiza_state = bytes([0])
                msg.hina_state = bytes([0])
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
