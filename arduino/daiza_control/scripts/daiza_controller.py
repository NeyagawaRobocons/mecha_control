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

arduino_port = find_arduino_uno_port()
if arduino_port is None:
    sg.popup_error('Arduino Uno not found. Please connect device and restart the program.')
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

layout = [
    [sg.Button('機構展開', key='mechaExpand'), sg.Button('機構格納', key='mechaRetract')],
    [sg.Button('箱のアーム展開', key='armOpen'), sg.Button('箱のアーム閉じる', key='armClose')],
    [sg.Button('箱を倒すアームを展開', key='boxArmExpand'), sg.Button('箱を倒すアームを縮小', key='boxArmRetract')],
    [sg.Text("Arduino Serial Output:", font=("Helvetica", 12))],
    [sg.Multiline(key='-OUTPUT-', size=(100, 10), autoscroll=True, disabled=True)]
]

window = sg.Window('Arduino Controller', layout)

threading.Thread(target=read_serial_data, args=(window,), daemon=True).start()

while True:
    event, values = window.read()

    if event == sg.WIN_CLOSED or event == 'Exit':
        break

    if event in ('mechaExpand', 'mechaRetract', 'armOpen', 'armClose', 'boxArmExpand', 'boxArmRetract'):
        threading.Thread(target=send_command, args=(event,), daemon=True).start()

    if event == '-SERIAL-':
        window['-OUTPUT-'].update(values['-SERIAL-'] + '\n', append=True)

window.close()
arduino.close()
