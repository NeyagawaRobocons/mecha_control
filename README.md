# mecha_control
機構制御＠ROS

# 仕様
## 機構の仕様
台座機構
- 角度調整
  - モータ　×１
  - リミットスイッチ　×2
- 台座把持
  - シリンダー　×３（ソレノイドバルブ×2）
- 箱読み取り
  - リミットスイッチ　×2

人形機構
- 角度調整
  - モータ　×2
  - リミットスイッチ　×2
  - ポテンショメータ　×2
- 高さ調整
  - モータ　×2
  - リミットスイッチ　×6（各3個ずつ）
- 壁読み取り
  - リミットスイッチ　×2

足回り
  - モータ　×3
  - ロリコン　×6

## nucleo_agentとの通信部分
- nucleo_agentとtopicで相互通信し続ける形で指令を送る
- サーボ以外全部1, 0で送る
- 機構ごとにトピックを作成
```
# mecha_control/msg/DaizaClamp.msg
bool[] state
```

```
# mecha_control/msg/DaizaState.msg

```

```
# mecha_control/msg/HinaDustpan.msg
bool[] state
float64[] servo
```

```
# mecha_control/msg/HinaState.msg
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
    - /daiza_stateトピック(mecha_control/msg/DaizaState型)
    - /hina_stateトピック(mecha_control/msg/HinaState型)
  - 出力：
    - 機構の指令
    - /daiza_clampトピック(mecha_control/msg/DaizaClamp型)
    - /hina_dastpanトピック(mecha_control/msg/HinaDustpan型)

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
    - /daiza_stateトピック(mecha_control/msg/DaizaState型)
    - /hina_stateトピック(mecha_control/msg/HinaState型)
  - 出力：
    - 機構の指令
    - /daiza_clampトピック(mecha_control/msg/DaizaClamp型)
    - /hina_dastpanトピック(mecha_control/msg/HinaDustpan型)

### ダミー機構ノード
- 機構指令のトピックを受け取り、機構の状態を返す
- 言語：Python
- 入出力：
  - 入力：
    - 機構の指令
    - /daiza_clampトピック(mecha_control/msg/DaizaClamp型)
    - /hina_dastpanトピック(mecha_control/msg/HinaDustpan型)
  - 出力：
    - 機構の状態
    - /daiza_stateトピック(mecha_control/msg/DaizaState型)
    - /hina_stateトピック(mecha_control/msg/HinaState型)
