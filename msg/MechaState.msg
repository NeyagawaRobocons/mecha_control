# mecha_control/msg/MechaState.msg
# 1: 展開
# 2: 回収
# 3: 設置
# 4: 格納
# false: ぼんぼりオフ
# true: ぼんぼり点灯

byte daiza_state
byte hina_state
bool bonbori_state