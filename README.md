# mecha_control
機構制御＠ROS

# 仕様
## アクチュエータ・センサ類
### 台座機構
- 角度調整
  - シリンダー × 1 -> シリンダ4
- 台座把持
  - シリンダー × 3（ソレノイドバルブ × 2）
    - 左右のシリンダ：右：シリンダ1, 左：シリンダ2
    - 小台座倒すやつ：自作シリンダ3
- 台座読み取り
  - リミットスイッチ × 2

### 人形機構
- 角度調整
  - モータ × 1
  - リミットスイッチ × 1 (水平を0°として、180°の位置に取り付け -> 格納時に押される)
  - ポテンショメータ
- 上下展開
  - モータ × 1
  - リミットスイッチ × 2
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
  - シリンダ1, 2, 3, 4: 縮んでる
  - リミットスイッチ(台座): 押されていない
1. 展開
- シリンダ4, 1, 2, 3を展開
2. 回収
- if (指令が来た) かつ (リミットスイッチ(台座)が押されいる)
  - シリンダ3を縮める: 小台座を倒す
  - シリンダ1, 2を縮める: 台座を挟む
  - シリンダ4を縮める: 持ち上げ
3. 設置
- シリンダ4を展開: 下げる
- シリンダ1, 2, 3を展開

### 人形機構
- 初期状態
  - ポテンショメータ: 180°
  - リミットスイッチ(180°): 押されてる
  - リミットスイッチ(上): 押されてる
  - リミットスイッチ(下): 押されてない
  - リミットスイッチ(壁)1, 2: 押されてない
0. 準備
- モータ2: ポテンショメータが90°になるまで展開
- モータ1: リミットスイッチ(下)が押されるまで縮める
1. 展開
- モータ2: ポテンショメータが-10° (調整できるようにする)になるまで動かす
2. 回収
- if (リミットスイッチ(壁)1, 2が両方押されたら)
  - モータ2: テンショメータ1,2が90°になるまで縮める
  - モータ1: リミットスイッチ(上)が押されるまで展開
3. 設置
- if (リミットスイッチ(上)が押されいる)
  - モータ2: ポテンショメータが-10° (調整できるようにする)になるまで動かす
  - 数秒待つ(要調整) or すべて排出できたか検出する -> (**要検討**)
  - モータ2: テンショメータが90°になるまで縮める
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
bool[] cylinder_states           # シリンダの状態
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

## メッセージの中身
### 台座機構

#### `/daiza_clamp`トピック: `ActuatorCommands.msg`（アクチュエータ指令）
- `cylinder_states`（シリンダの状態）
  - `0`: シリンダ1
  - `1`: シリンダ2
  - `2`: シリンダ3
  - `3`: シリンダ4
#### `/daiza_state`トピック: `SensorStates.msg`（センサ状態）
- `limit_switch_states`（リミットスイッチの状態）
  - `0`: リミットスイッチ(台座)
- `cylinder_states` (シリンダの状態)
  - `0`: シリンダ1
  - `1`: シリンダ2
  - `2`: シリンダ3
  - `3`: シリンダ4

### 人形機構

#### `/hina_dastpan`トピック: `ActuatorCommands.msg`（アクチュエータ指令）
- `motor_positions`（モータの位置）
  - `0`: モータ1 -> 上下
  - `1`: モータ2 -> 傾き

#### `/hina_state`トピック: `SensorStates.msg`（センサ状態）
- `limit_switch_states`（リミットスイッチの状態）
  - `0`: リミットスイッチ(上)
  - `1`: リミットスイッチ(下)
  - `2`: リミットスイッチ(壁)1
  - `3`: リミットスイッチ(壁)2
  - `4`: リミットスイッチ(180)
- `potentiometer_angles`（ポテンショメータの角度）
  - `0`: ポテンショメータ

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
