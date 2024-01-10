import PySimpleGUI as sg
import serial
import threading
from serial.tools import list_ports

def find_arduino_uno_port():
    """接続されているArduino UnoのCOMポートを見つける"""
    ports = list_ports.comports()
    for port, desc, hwid in sorted(ports):
        if 'Arduino Uno' in desc:
            return port
        
    return None

# Arduino Unoのポートを自動選択
arduino_port = find_arduino_uno_port()
if arduino_port is None:
    sg.popup_error('Arduino Uno not found. Please connect device and restart the program.')
    exit()

# Arduinoとのシリアル接続を設定
arduino = serial.Serial(port=arduino_port, baudrate=115200, timeout=.1)

def send_command(command):
    """Arduinoにコマンドを送信する関数"""
    arduino.write(bytes(command + "\n", 'utf-8'))

# GUIのレイアウトを定義
layout = [
    [sg.Button('Move Up', key='moveUp'), sg.Button('Move Down', key='moveDown')],
    [sg.Button('Expand Arm', key='expandArm'), sg.Button('Contract Arm', key='contractArm')],
    [sg.Button('Reset Arm', key='resetArm'), sg.Button('Exit', key='Exit')]
]

# ウィンドウの作成
window = sg.Window('Arduino Controller', layout)

# イベントループ
while True:
    event, values = window.read()

    # ウィンドウが閉じられたら終了
    if event == sg.WIN_CLOSED or event == 'Exit':
        break

    # ボタンが押されたら対応するコマンドを送信
    if event in ('moveUp', 'moveDown', 'expandArm', 'contractArm', 'resetArm'):
        threading.Thread(target=send_command, args=(event,), daemon=True).start()

# ウィンドウを閉じる
window.close()
arduino.close()
