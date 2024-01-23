#!/usr/bin/env python3
"""
# mecha_control/msg/MechaState.msg
# 1: 展開
# 2: 回収
# 3: 設置
# 4: 格納
# false: ぼんぼりオフ
# true: ぼんぼり点灯
byte daiza_state
byte hina_state
bool bonbori_state
"""

"""
# 機構制御のコマンド一覧（Arduinoにコマンドをシリアルで送信する）

# 台座機構
台座機構展開シリンダを動かす：mechaExpand
台座機構回収シリンダを格納：mechaRetract
台座回収アームの展開：armOpen
台座回収アームの格納：armClose
台座を倒すアームの展開：boxArmExpand
台座を倒すアームの格納：boxArmRetract

# 人形機構
上昇モータを動かす：moveUp
下降モータを動かす：moveDown
アームを展開する：expandArm
アームを格納する：contractArm
開放モータを動かす：open

"""

"""
# 機構動作一覧

# 台座機構
台座展開：mechaExpand, delay(1 sec), armOpen, boxArmExpand
台座回収：boxArmRetract, delay(1 sec), armClose, delay(1 sec), mechaRetract
台座設置：mechaExpand, armOpen
台座格納：armClose, delay(1 sec), mechaRetract

# 人形機構
人形展開：expandArm, delay(1 sec), moveDown
人形回収：contractArm, delay(1 sec), moveUp
人形設置：expandArm, delay(1 sec), open, delay(1 sec), contractArm
人形格納：contractArm

"""

import rclpy
from rclpy.node import Node
from mecha_control.msg import MechaState
import PySimpleGUI as sg
import serial
import threading
from serial.tools import list_ports

# board_type = 'Arduino Mega 2560'
board_type = 'ttyACM0'

def find_arduino_uno_port(_board_type):
    """接続されているArduino UnoのCOMポートを見つける"""
    ports = list_ports.comports()
    for port, desc, hwid in sorted(ports):
        print(port, desc, hwid)
        if _board_type in desc:
            return port
    return None

arduino_port = find_arduino_uno_port(board_type)
if arduino_port is None:
    sg.popup_error(board_type + ' not found. Please connect device and restart the program.')
    exit()

arduino = serial.Serial(port=arduino_port, baudrate=9600, timeout=.1)

def send_command(command):
    arduino.write(bytes(command + "\n", 'utf-8'))

def read_serial_data(window):
    """Arduinoからシリアルデータを読み込み、ウィンドウに表示する"""
    while True:
        if arduino.in_waiting:
            data = arduino.readline().decode('utf-8', errors='replace').rstrip()
            window.write_event_value('-SERIAL-', data)

def window_loop(window):
    while True:
        event, values = window.read()
        if event == '-SERIAL-':
            window['-OUTPUT-'].update(values['-SERIAL-'] + '\n', append=True)


layout = [
    [sg.Text("Arduino Serial Output:", font=("Helvetica", 12))],
    [sg.Multiline(key='-OUTPUT-', size=(100, 10), autoscroll=True, disabled=True)],
    [sg.Button('Exit', key='Exit')]
]

window = sg.Window('Arduino Controller', layout)

threading.Thread(target=read_serial_data, args=(window,), daemon=True).start()
threading.Thread(target=window_loop, args=(window,), daemon=True).start()

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
        elif msg.daiza_state == bytes([4]):
            self.daiza_kakunou()
        if msg.hina_state == bytes([1]):
            self.hina_tenkai()
        elif msg.hina_state == bytes([2]):
            self.hina_kaishu()
        elif msg.hina_state == bytes([3]):
            self.hina_setti()
        elif msg.hina_state == bytes([4]):
            self.hina_kakunou()
        if msg.bonbori_state == True:
            self.bonbori_tento()

    def daiza_tenkai(self):
        self.get_logger().info('daiza_tenkai')
        send_command('mechaExpand')
        send_command('armOpen')
        send_command('boxArmExpand')
    
    def daiza_kaishu(self):
        self.get_logger().info('daiza_kaishu')
        send_command('boxArmRetract')
        send_command('armClose')
        send_command('mechaRetract')

    def daiza_setti(self):
        self.get_logger().info('daiza_setti')
        send_command('mechaExpand')
        send_command('armOpen')
    
    def daiza_kakunou(self):
        self.get_logger().info('daiza_kakunou')
        send_command('armClose')
        send_command('mechaRetract')
    
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
    window.close()
    arduino.close()

if __name__ == '__main__':
    main()