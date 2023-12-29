# mecha_control
機構制御＠ROS

# 仕様
## アクチュエータ・センサ類
### 台座機構
- 角度調整
  - モータ × 1
  - リミットスイッチ × 2
- 台座把持
  - シリンダー × 3（ソレノイドバルブ×2）
    - 左右のシリンダ：右：シリンダ1, 左：シリンダ2
    - 小台座倒すやつ：シリンダ3
- 箱読み取り
  - リミットスイッチ × 2

### 人形機構
- 角度調整
  - モータ × 2
  - リミットスイッチ × 2
  - ポテンショメータ × 2
- 上下展開
  - 自作シリンダ × 2
- 壁読み取り
  - リミットスイッチ × 2

### 足回り
  - モータ　×3
  - ロリコン　×6

## 機構の動作
### 台座機構
1. 展開
- シリンダ1, 2, 3を展開
- 角度調整モータを動かす(リミットスイッチ(下)が押されるまで)
2. 回収
- (指令が来た) かつ (台座読み取り用リミットスイッチが押されいる)ならば
- シリンダ3を縮める: 小台座を倒す
- シリンダ1, 2を縮める: 台座を挟む
- 角度調整モータを動かす(リミットスイッチ(上)が押されるまで)
3. 設置
- 角度調整モータを動かす(リミットスイッチ(下)が押されるまで)
- シリンダ1, 2, 3を展開

### 人形機構
1. 展開
- 
2. 回収
- 
3. 設置
- 

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
