#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mecha_control.msg import ActuatorCommands, SensorStates
import tkinter as tk

class DummyNode(Node):
    def __init__(self):
        super().__init__('dummy_node')
        self.publisher = self.create_publisher(SensorStates, '/sensor_states', 10)
        self.subscription_daiza = self.create_subscription(
            ActuatorCommands, '/daiza_clamp', self.actuator_callback_daiza, 10)
        self.subscription_hina = self.create_subscription(
            ActuatorCommands, '/hina_dastpan', self.actuator_callback_hina, 10)
        
        self.sensor_states = SensorStates()

        # limit_switch_states 配列の初期化
        self.sensor_states.limit_switch_states = [False] * 6  # 配列のサイズに応じて調整

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
        self.sensor_states.limit_switch_states[index] = not self.sensor_states.limit_switch_states[index]
        self.publisher.publish(self.sensor_states)

    def actuator_callback_daiza(self, msg):
        # 受け取ったメッセージから台座機構のシリンダ状態を更新
        self.sensor_states.limit_switch_states[3] = msg.cylinder_states[0]  # シリンダ1
        self.sensor_states.limit_switch_states[4] = msg.cylinder_states[1]  # シリンダ2
        self.sensor_states.limit_switch_states[5] = msg.cylinder_states[2]  # シリンダ3
        self.publisher.publish(self.sensor_states)

    def actuator_callback_hina(self, msg):
        # 受け取ったメッセージから人形機構のシリンダ状態を更新
        self.sensor_states.limit_switch_states[4] = msg.cylinder_states[0]  # シリンダ1
        self.sensor_states.limit_switch_states[5] = msg.cylinder_states[1]  # シリンダ2
        self.publisher.publish(self.sensor_states)

def main(args=None):
    rclpy.init(args=args)
    dummy_node = DummyNode()
    rclpy.spin(dummy_node)
    dummy_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()