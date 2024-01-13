import PySimpleGUI as sg
from flask import Flask, request
import serial
from serial.tools import list_ports
import threading

board_type = 'Arduino Uno'

def find_arduino_uno_port(_board_type):
    """接続されているArduino UnoのCOMポートを見つける"""
    ports = list_ports.comports()
    for port, desc, hwid in sorted(ports):
        print(port, desc, hwid)
        if _board_type in desc:
            return port
    return None

app = Flask(__name__)

# Arduinoへのシリアル接続を設定
arduino_port = find_arduino_uno_port(board_type)
if arduino_port is None:
    print(board_type + ' not found. Please connect device and try again.')
    exit()
arduino = serial.Serial(port=arduino_port, baudrate=9600, timeout=.1)

@app.route('/')
def handle_request():
    command = request.args.get('command', 'none')
    print(f"Received command: {command}")

    # Arduinoにコマンドを送信
    if command != 'none':
        arduino.write(f"{command}\n".encode())

    return f"Command {command} sent to Arduino."

def read_serial_data(window):
    """Arduinoからシリアルデータを読み込み、ウィンドウに表示する"""
    while True:
        if arduino.in_waiting:
            data = arduino.readline().decode('utf-8', errors='replace').rstrip()
            # print(data)
            window.write_event_value('-SERIAL-', data)

# GUIレイアウトの定義
# フォントサイズを大きくし、ウィンドウサイズを調整
layout = [
    [sg.Text("Arduinoからのシリアルデータ:", font=("Helvetica", 14))],
    [sg.Multiline(key='-OUTPUT-', size=(70, 20), autoscroll=True, disabled=True, font=("Helvetica", 30))],
    [sg.Button('Exit', key='Exit', font=("Helvetica", 12))]
]

# ウィンドウの作成、ウィンドウサイズを調整
window = sg.Window('Arduino Serial Monitor', layout, size=(1500, 600))

def window_event_loop():
    """PySimpleGUIのイベントループを実行する"""
    while True:
        event, values = window.read(timeout=100)

        if event == sg.WIN_CLOSED or event == 'Exit':
            break

        if event == '-SERIAL-':
            window['-OUTPUT-'].update(values['-SERIAL-'] + '\n', append=True)

threading.Thread(target=read_serial_data, args=(window,), daemon=True).start()
threading.Thread(target=window_event_loop, args=(), daemon=True).start()

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8000)

window.close()
arduino.close()