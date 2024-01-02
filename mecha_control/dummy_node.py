#!/usr/bin/env python3
"""
## メッセージの中身
### 台座機構

#### `/daiza_clamp`トピック: `ActuatorCommands.msg`（アクチュエータ指令）
- `cylinder_states`（シリンダの状態）
  - `0`: シリンダ1
  - `1`: シリンダ2
  - `2`: シリンダ3
- `motor_positions`（モータの位置）
  - `0`: 角度調整モータ

#### `/daiza_state`トピック: `SensorStates.msg`（センサ状態）
- `limit_switch_states`（リミットスイッチの状態）
  - `0`: リミットスイッチ(上)
  - `1`: リミットスイッチ(下)
  - `2`: リミットスイッチ(台座)
- `cylinder_states` (シリンダの状態)
  - `0`: シリンダ1
  - `1`: シリンダ2
  - `2`: シリンダ3

### 人形機構

#### `/hina_dastpan`トピック: `ActuatorCommands.msg`（アクチュエータ指令）
- `cylinder_states`（シリンダの状態）
  - `0`: シリンダ1
  - `1`: シリンダ2
- `motor_positions`（モータの位置）
  - `0`: モータ1
  - `1`: モータ2

#### `/hina_state`トピック: `SensorStates.msg`（センサ状態）
- `limit_switch_states`（リミットスイッチの状態）
  - `0`: リミットスイッチ(180)1
  - `1`: リミットスイッチ(180)2
  - `2`: リミットスイッチ(壁)1
  - `3`: リミットスイッチ(壁)2
- `cylinder_states` (シリンダの状態)
  - `0`: シリンダ1
  - `1`: シリンダ2
- `potentiometer_angles`（ポテンショメータの角度）
  - `0`: ポテンショメータ1
  - `1`: ポテンショメータ2
"""

import rclpy
from rclpy.node import Node
from mecha_control.msg import ActuatorCommands, SensorStates
import tkinter as tk

class DummyNode(Node):
    def __init__(self):
        super().__init__('dummy_node')
        self.daiza_publisher = self.create_publisher(SensorStates, '/daiza_state', 10)
        self.daiza_states = SensorStates()
        self.hina_publisher = self.create_publisher(SensorStates, '/hina_state', 10)
        self.hina_states = SensorStates()
        self.subscription_daiza = self.create_subscription(
            ActuatorCommands, '/daiza_clamp', self.actuator_callback_daiza, 10)
        self.subscription_hina = self.create_subscription(
            ActuatorCommands, '/hina_dastpan', self.actuator_callback_hina, 10)
        
        # daiza's variables
        self.daiza_states.limit_switch_states = [False] * 3
        self.daiza_states.cylinder_states = [False] * 3
        # daiza_commands 配列の初期化
        self.daiza_commands = ActuatorCommands()
        self.daiza_commands.cylinder_states = [False] * 3
        self.daiza_commands.motor_positions = [0.0]

        # hina's variables
        self.hina_states.limit_switch_states = [False] * 4
        self.hina_states.cylinder_states = [False] * 2
        self.hina_states.potentiometer_angles = [0.0] * 2
        # hina_commands 配列の初期化
        self.hina_commands = ActuatorCommands()
        self.hina_commands.cylinder_states = [False] * 2
        self.hina_commands.motor_positions = [0.0] * 2

        self.init_gui()

    def init_gui(self):
        self.window = tk.Tk()
        self.window.title("Dummy Node GUI")

        # 台座機構のリミットスイッチのボタン
        self.daiza_switch_buttons = {
            "上": tk.Button(self.window, text="台座:リミットスイッチ(上)", command=lambda: self.update_limit_switch("台座", "上")),
            "下": tk.Button(self.window, text="台座:リミットスイッチ(下)", command=lambda: self.update_limit_switch("台座", "下")),
            "台座": tk.Button(self.window, text="台座:リミットスイッチ(台座)", command=lambda: self.update_limit_switch("台座", "台座"))
        }
        for button in self.daiza_switch_buttons.values():
            button.pack()

        # 人形機構のリミットスイッチのボタン
        self.hina_switch_buttons = {
            "180°": tk.Button(self.window, text="人形:リミットスイッチ(180°)", command=lambda: self.update_limit_switch("人形", "180°")),
            "壁": tk.Button(self.window, text="人形:リミットスイッチ(壁)", command=lambda: self.update_limit_switch("人形", "壁"))
        }
        for button in self.hina_switch_buttons.values():
            button.pack()

        self.window.mainloop()

    def update_limit_switch(self, kikou, switch):
        index_map = {
            "台座": {"上": 0, "下": 1, "台座": 2},
            "人形": {"180°": 0, "壁": 1}
        }
        index = index_map[kikou][switch]
        if kikou == "台座":
            self.daiza_states.limit_switch_states[index] = not self.daiza_states.limit_switch_states[index]
        elif kikou == "人形":
            self.hina_states.limit_switch_states[index] = not self.hina_states.limit_switch_states[index]
        self.daiza_publisher.publish(self.daiza_states)
        self.hina_publisher.publish(self.hina_states)

    def actuator_callback_daiza(self, msg):
        # 受け取ったメッセージから台座機構のシリンダ状態を更新
        self.daiza_states.cylinder_states[0] = msg.cylinder_states[0]  # シリンダ1
        self.daiza_states.cylinder_states[1] = msg.cylinder_states[1]
        self.daiza_states.cylinder_states[2] = msg.cylinder_states[2]
        self.daiza_publisher.publish(self.daiza_states)

    def actuator_callback_hina(self, msg):
        # 受け取ったメッセージから人形機構のシリンダ状態を更新
        self.hina_states.cylinder_states[0] = msg.cylinder_states[0]
        self.hina_states.cylinder_states[1] = msg.cylinder_states[1]
        self.hina_publisher.publish(self.hina_states)

    def show_commands(self):
        self.get_logger("台座機構の指令: ", self.daiza_commands, "人形機構の指令: ", self.hina_commands)

def main(args=None):
    rclpy.init(args=args)
    dummy_node = DummyNode()
    rclpy.spin(dummy_node)
    dummy_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()