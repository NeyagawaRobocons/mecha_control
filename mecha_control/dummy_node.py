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

class DummyNode(Node):
    def __init__(self):
        super().__init__('dummy_node')
        self.daiza_publisher = self.create_publisher(SensorStates, '/daiza_state', 10)
        self.hina_publisher = self.create_publisher(SensorStates, '/hina_state', 10)
        self.subscription_daiza = self.create_subscription(ActuatorCommands, '/daiza_clamp', self.actuator_callback_daiza, 10)
        self.subscription_hina = self.create_subscription(ActuatorCommands, '/hina_dastpan', self.actuator_callback_hina, 10)

        # daiza variables
        self.daiza_states = SensorStates()
        self.daiza_states.limit_switch_states = [True, False, False]
        self.daiza_states.cylinder_states = [False] * 3

        # hina variables
        self.hina_states = SensorStates()
        self.hina_states.limit_switch_states = [True, True, False, False]
        self.hina_states.cylinder_states = [False] * 2

    def actuator_callback_daiza(self, msg):
        self.get_logger().info(f"daiza_commands: {msg}")
        # 受け取ったコマンドを処理
        if all(msg.cylinder_states):  # 展開の場合
            self.daiza_states.limit_switch_states = [False, True, False]  # 下のリミットスイッチが押される
            self.daiza_states.cylinder_states = [True, True, True]        # シリンダが展開
        elif not any(msg.cylinder_states):  # 回収の場合
            self.daiza_states.limit_switch_states = [True, False, True]  # 上のリミットスイッチが押される
            self.daiza_states.cylinder_states = [False, False, False]    # シリンダが縮む
        else:  # 設置の場合
            self.daiza_states.limit_switch_states = [False, True, False]  # 下のリミットスイッチが押される
            self.daiza_states.cylinder_states = [True, True, True]        # シリンダが展開
        self.daiza_publisher.publish(self.daiza_states)
        self.get_logger().info(f"daiza_states: {self.daiza_states}")

    def actuator_callback_hina(self, msg):
        # モータとシリンダの状態に応じて処理
        motor_positions = msg.motor_positions
        cylinder_states = msg.cylinder_states

        if motor_positions == [90.0, 90.0] and not any(cylinder_states):  # 準備の場合
            self.hina_states.limit_switch_states = [True, True, False, False]
            self.hina_states.cylinder_states = [False, False]
            self.hina_states.potentiometer_angles = [90.0, 90.0]
        elif motor_positions == [-10.0, -10.0]:  # 展開の場合
            self.hina_states.potentiometer_angles = [-10.0, -10.0]
        elif all(cylinder_states) and motor_positions == [90.0, 90.0]:  # 回収の場合
            self.hina_states.limit_switch_states = [False, False, True, True]
            self.hina_states.cylinder_states = [True, True]
            self.hina_states.potentiometer_angles = [90.0, 90.0]
        else:  # 設置の場合
            self.hina_states.potentiometer_angles = [-10.0, -10.0]  # ここは適宜調整
        self.hina_publisher.publish(self.hina_states)
        self.get_logger().info(f"hina_states: {self.hina_states}")

def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
