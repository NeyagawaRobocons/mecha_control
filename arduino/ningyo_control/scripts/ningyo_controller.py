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

def read_serial_data(window):
    """Arduinoからシリアルデータを読み込み、ウィンドウに表示する"""
    while True:
        if arduino.in_waiting:
            try:
                data = arduino.readline().decode('utf-8', errors='replace').rstrip()
                window.write_event_value('-SERIAL-', data)
            except Exception as e:
                print(f"Error reading serial data: {e}")

arduino_port = find_arduino_uno_port()
if arduino_port is None:
    sg.popup_error('Arduino Uno not found. Please connect device and restart the program.')
    exit()

arduino = serial.Serial(port=arduino_port, baudrate=115200, timeout=.1)

def send_command(command):
    arduino.write(bytes(command + "\n", 'utf-8'))

layout = [
    [sg.Button('Move Up', key='moveUp'), sg.Button('Move Down', key='moveDown')],
    [sg.Button('Expand Arm', key='expandArm'), sg.Button('Contract Arm', key='contractArm')],
    [sg.Button('Reset Arm', key='resetArm'), sg.Button('Exit', key='Exit')],
    [sg.Text("Arduino Serial Output:", font=("Helvetica", 12))],
    [sg.Multiline(key='-OUTPUT-', size=(40, 10), autoscroll=True, disabled=True)]
]

# ウィンドウの作成
window = sg.Window('Arduino Controller', layout)

# ここでウィンドウが作成されてからスレッドを開始
threading.Thread(target=read_serial_data, args=(window,), daemon=True).start()

while True:
    event, values = window.read()

    if event == sg.WIN_CLOSED or event == 'Exit':
        break

    if event in ('moveUp', 'moveDown', 'expandArm', 'contractArm', 'resetArm'):
        threading.Thread(target=send_command, args=(event,), daemon=True).start()

    if event == '-SERIAL-':
        window['-OUTPUT-'].update(values['-SERIAL-'] + '\n', append=True)

window.close()
arduino.close()