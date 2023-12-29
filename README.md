# mecha_control
機構制御＠ROS

# 仕様
## アクチュエータ・センサ類
### 台座機構
- 角度調整
  - モータ × 1
  - リミットスイッチ × 2
- 台座把持
  - シリンダー × 3（ソレノイドバルブ × 2）
    - 左右のシリンダ：右：シリンダ1, 左：シリンダ2
    - 小台座倒すやつ：自作シリンダ3
- 台座読み取り
  - リミットスイッチ × 2

### 人形機構
- 角度調整
  - モータ × 2
  - リミットスイッチ × 2 (水平を0°として、180°の位置に取り付け -> 格納時に押される)
  - ポテンショメータ × 2
- 上下展開
  - 自作シリンダ × 2
- 壁読み取り
  - リミットスイッチ × 2

### 足回り
  - モータ × 3
  - ロリコン × 6

## 機構の動作
凡例：
- シリンダが伸びている: 1
- シリンダが縮んでいる: 0
- リミットスイッチが押されている: 1
- リミットスイッチが押されていない: 0
### 台座機構
- 初期状態
  - シリンダ1, 2, 3: 縮んでる
  - リミットスイッチ(上): 押されている
  - リミットスイッチ(下): 押されていない
  - リミットスイッチ(台座): 押されていない
1. 展開
- シリンダ1, 2, 3を展開
- 角度調整モータを動かす(リミットスイッチ(下)が押されるまで)
2. 回収
- if (指令が来た) かつ (リミットスイッチ(台座)が押されいる)
  - シリンダ3を縮める: 小台座を倒す
  - シリンダ1, 2を縮める: 台座を挟む
  - 角度調整モータを動かす(リミットスイッチ(上)が押されるまで)
3. 設置
- 角度調整モータを動かす(リミットスイッチ(下)が押されるまで)
- シリンダ1, 2, 3を展開

### 人形機構
- 初期状態
  - ポテンショメータ1, 2: 180° -> 青を1, 赤を2とする
  - リミットスイッチ(180°)1, 2: 押されてる
  - シリンダ1, 2: 伸びてる
  - リミットスイッチ(壁)1, 2: 押されてない
0. 準備
- モータ1, 2: ポテンショメータ1,2が90°になるまで展開
- シリンダ1, 2: 縮める
1. 展開
- モータ1, 2: ポテンショメータが-10° (調整できるようにする)になるまで動かす
2. 回収
- if (リミットスイッチ(壁)1, 2が両方押されたら)
  - モータ1, 2: テンショメータ1,2が90°になるまで縮める
  - シリンダ1, 2: 伸ばす
3. 設置
- if (シリンダ1, 2が伸びてたら)
  - モータ1, 2: ポテンショメータが-10° (調整できるようにする)になるまで動かす
  - 数秒待つ(要調整) or すべて排出できたか検出する -> (**要検討**)
  - モータ1, 2: テンショメータ1,2が90°になるまで縮める
  - ぼんぼり点灯をする

## nucleo_agentとの通信部分
- nucleo_agentとtopicで相互通信し続ける形で指令を送る
- サーボ以外全部1, 0で送る
- 機構ごとにトピックを作成
  - 台座機構指令: /daiza_clampトピック(mecha_control/msg/ActuatorCommands.msg)
  - 台座機構状態: /daiza_stateトピック(mecha_control/msg/SensorStates.msg)
```
# mecha_control/msg/ActuatorCommands.msg
float64[] motor_positions    # モータの位置や角度を制御するための値
bool[] cylinder_states       # エアシリンダのオン/オフ状態
```

```
# mecha_control/msg/SensorStates.msg
float64[] potentiometer_angles   # ポテンショメータで測定される角度
bool[] limit_switch_states       # リミットスイッチのオン/オフ状態
```

## 機構を動かす指令
- これもトピックで実装
- /mecha_stateトピック(mecha_control/msg/MechaState型)

```
# mecha_control/msg/MechaState.msg
# 0: 準備
# 1: 展開
# 2: 回収
# 3: 設置
# false: ぼんぼりオフ
# true: ぼんぼり点灯
byte daiza_state
byte hina_state
bool bonbori_state
```

## デバッグ用のあれこれ仕様
### デバッグ用コントローラ仕様
- 各アクチュエータを動かせるかテストする
  - 各アクチュエータを動かすボタンを配置
  - 角度指定系は角度を入力 -> "apply"
  - GUIは凝らない
  - 状態を表示できるようにする
- 言語：Python
- 入出力：
  - 入力1：指令値
    - GUIから
    - 形式：boolとfloat64
  - 入力2：
    - 機構の状態
    - /daiza_stateトピック(mecha_control/msg/SensorStates型)
    - /hina_stateトピック(mecha_control/msg/SensorStates型)
  - 出力：
    - 機構の指令
    - /daiza_clampトピック(mecha_control/msg/ActuatorCommands型)
    - /hina_dastpanトピック(mecha_control/msg/ActuatorCommands型)

### シーケンス動作チェック用コントローラ仕様
- 以下の3つの動作を指定できる
  - 台座機構：展開、回収、接地
  - 人形機構：展開、回収、接地
  - ぼんぼり点灯：点灯開始
 
### シーケンス制御ノード仕様
- 「機構」の状態を受け取って、指令をシーケンス的に指令するノード
- 言語：C++
- 入出力：
  - 入力1：機構状態
    - /mecha_stateトピック(mecha_control/msg/MechaState型)
  - 入力2：
    - 機構の状態
    - /daiza_stateトピック(mecha_control/msg/SensorStates型)
    - /hina_stateトピック(mecha_control/msg/SensorStates型)
  - 出力：
    - 機構の指令
    - /daiza_clampトピック(mecha_control/msg/ActuatorCommands型)
    - /hina_dastpanトピック(mecha_control/msg/ActuatorCommands型)

### ダミー機構ノード
- 機構指令のトピックを受け取り、機構の状態を返す
- 言語：Python
- 入出力：
  - 入力1：
    - 機構の指令
    - /daiza_clampトピック(mecha_control/msg/ActuatorCommands型)
    - /hina_dastpanトピック(mecha_control/msg/ActuatorCommands型)
  - 入力2：
    - GUIから手動でタイミングきめて入力してリミットスイッチなどの状態の更新
    - (〜が押されるまで)の部分を更新
  - 出力：
    - 機構の状態
    - /daiza_stateトピック(mecha_control/msg/SensorStates型)
    - /hina_stateトピック(mecha_control/msg/SensorStates型)
