# mecha_control
機構制御＠ROS

# 仕様
## 機構の仕様
台座
- 角度調整
  - モータ　×１
  - リミットスイッチ　×2
- 台座把持
  - シリンダー　×３（ソレノイドバルブ×2）
- 箱読み取り
  - リミットスイッチ　×2
黒板消し
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
### 仮コントローラ仕様
- 各アクチュエータを動かすボタンを配置
- 角度指定系は角度を入力 -> "apply"
- GUIは凝らない

### ダミー機構ノード
- 機構指令のトピックを受け取る
- 機構の状態を返す
