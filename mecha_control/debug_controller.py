#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mecha_control.msg import ActuatorCommands, SensorStates
import tkinter as tk

class DebugController(Node):
    def __init__(self):
        super().__init__('debug_controller')
        # サブスクライバーの設定
        self.subscription_daiza = self.create_subscription(SensorStates, '/daiza_state', self.daiza_state_callback, 10)
        self.subscription_hina = self.create_subscription(SensorStates, '/hina_state', self.hina_state_callback, 10)
        # パブリッシャーの設定
        self.publisher_daiza = self.create_publisher(ActuatorCommands, '/daiza_clamp', 10)
        self.publisher_hina = self.create_publisher(ActuatorCommands, '/hina_dastpan', 10)
        # GUIの初期化
        self.init_gui()

    def init_gui(self):
        self.window = tk.Tk()
        self.window.title("Debug Controller")

        # 台座機構のセクション
        daiza_frame = tk.LabelFrame(self.window, text="台座機構")
        daiza_frame.pack(fill="both", expand="yes", padx=10, pady=5)

        # シリンダのトグルボタン
        self.cylinder_buttons_daiza = []
        for i in range(3):
            button = tk.Button(daiza_frame, text=f"シリンダ{i + 1}", command=lambda i=i: self.toggle_cylinder_daiza(i))
            button.pack(side="left")
            self.cylinder_buttons_daiza.append(button)

        # 角度調整モータの入力
        self.motor_angle_daiza = tk.DoubleVar()
        tk.Entry(daiza_frame, textvariable=self.motor_angle_daiza).pack(side="left")
        tk.Button(daiza_frame, text="角度適用", command=self.apply_motor_angle_daiza).pack(side="left")

        # 人形機構のセクション
        hina_frame = tk.LabelFrame(self.window, text="人形機構")
        hina_frame.pack(fill="both", expand="yes", padx=10, pady=5)

        # シリンダのトグルボタン
        self.cylinder_buttons_hina = []
        for i in range(2):
            button = tk.Button(hina_frame, text=f"シリンダ{i + 1}", command=lambda i=i: self.toggle_cylinder_hina(i))
            button.pack(side="left")
            self.cylinder_buttons_hina.append(button)

        # モータの入力
        self.motor_angles_hina = [tk.DoubleVar(), tk.DoubleVar()]
        for i, var in enumerate(self.motor_angles_hina):
            tk.Entry(hina_frame, textvariable=var).pack(side="left")
        tk.Button(hina_frame, text="角度適用", command=self.apply_motor_angles_hina).pack(side="left")

        # リミットスイッチの状態表示
        self.limit_switch_status = tk.StringVar()
        tk.Label(self.window, textvariable=self.limit_switch_status).pack()

        self.window.mainloop()

    def toggle_cylinder_daiza(self, index):
        # 台座機構のシリンダの状態をトグル
        command = ActuatorCommands()
        command.cylinder_states = [False] * 3  # 3つのシリンダ
        command.cylinder_states[index] = True
        self.publisher_daiza.publish(command)

    def apply_motor_angle_daiza(self):
        # 台座機構の角度調整モータの角度を適用
        command = ActuatorCommands()
        command.motor_positions = [self.motor_angle_daiza.get()]
        self.publisher_daiza.publish(command)

    def toggle_cylinder_hina(self, index):
        # 人形機構のシリンダの状態をトグル
        command = ActuatorCommands()
        command.cylinder_states = [False] * 2  # 2つのシリンダ
        command.cylinder_states[index] = True
        self.publisher_hina.publish(command)

    def apply_motor_angles_hina(self):
        # 人形機構のモータの角度を適用
        command = ActuatorCommands()
        command.motor_positions = [var.get() for var in self.motor_angles_hina]
        self.publisher_hina.publish(command)

    def daiza_state_callback(self, msg):
        # 台座機構の状態をGUIに表示
        state_str = "台座状態: " + ", ".join([str(state) for state in msg.limit_switch_states])
        self.limit_switch_status.set(state_str)

    def hina_state_callback(self, msg):
        # 人形機構の状態をGUIに表示
        state_str = "人形状態: " + ", ".join([str(state) for state in msg.limit_switch_states])
        self.limit_switch_status.set(state_str)

def main(args=None):
    rclpy.init(args=args)
    controller = DebugController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
