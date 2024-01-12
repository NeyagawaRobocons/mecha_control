// エアシリンダの機構制御コード
/*
送られてくるコマンド
- "mechaExpand": 機構展開
- "mechaRetract": 機構格納
- "armOpen": 箱のアーム展開
- "armClose": 箱のアーム閉じる
- "boxArmExpand": 箱を倒すアームを展開
- "boxArmRetract": 箱を倒すアームを縮小
*/

#include <Arduino.h>
const int armOpenPin = 2;
const int armClosePin = 3;
const int boxArmExpandPin = 4; // 箱を倒すアームの展開ピン
const int boxArmRetractPin = 5; // 箱を倒すアームの閉じるピン
const int mechaExpandPin = 6; // 機構全体を展開するピン
const int mechaRetractPin = 7; // 機構全体を縮小するピン

void setup() {
  pinMode(armOpenPin, OUTPUT);
  pinMode(armClosePin, OUTPUT);
  pinMode(boxArmExpandPin, OUTPUT);
  pinMode(boxArmRetractPin, OUTPUT);
  pinMode(mechaExpandPin, OUTPUT);
  pinMode(mechaRetractPin, OUTPUT);
  Serial.begin(9600);
  Serial.println("起動しました。");
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    if (command == "mechaExpand") {
      Serial.println("mechaExpand");
      digitalWrite(mechaExpandPin, HIGH);
      digitalWrite(mechaRetractPin, LOW);
      delay(1000);
      digitalWrite(mechaExpandPin, LOW); // 1秒待ってオフにする
    } else if (command == "mechaRetract") {
      Serial.println("mechaRetract");
      digitalWrite(mechaExpandPin, LOW);
      digitalWrite(mechaRetractPin, HIGH);
      delay(1000);
      digitalWrite(mechaRetractPin, LOW); // 1秒待ってオフにする
    } else if (command == "armOpen") {
      Serial.println("armOpen");
      digitalWrite(armOpenPin, HIGH);
      digitalWrite(armClosePin, LOW);
      delay(1000);
      digitalWrite(armOpenPin, LOW); // 1秒待ってオフにする
    } else if (command == "armClose") {
      Serial.println("armClose");
      digitalWrite(armOpenPin, LOW);
      digitalWrite(armClosePin, HIGH);
      delay(1000);
      digitalWrite(armClosePin, LOW); // 1秒待ってオフにする
    } else if (command == "boxArmExpand") {
      Serial.println("boxArmExpand");
      digitalWrite(boxArmExpandPin, HIGH);
      digitalWrite(boxArmRetractPin, LOW);
      delay(1000);
      digitalWrite(boxArmExpandPin, LOW); // 1秒待ってオフにする
    } else if (command == "boxArmRetract") {
      Serial.println("boxArmRetract");
      digitalWrite(boxArmExpandPin, LOW);
      digitalWrite(boxArmRetractPin, HIGH);
      delay(1000);
      digitalWrite(boxArmRetractPin, LOW); // 1秒待ってオフにする
    }
  }
  delay(10);
}