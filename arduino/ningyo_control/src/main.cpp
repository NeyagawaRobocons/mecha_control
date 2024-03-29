#include <Arduino.h>
#include "DualVNH5019MotorShield.h"

DualVNH5019MotorShield md;
const int limitSwitchUpperPin = 3;  // 上部リミットスイッチ用ピン
const int limitSwitchLowerPin = 5;  // 下部リミットスイッチ用ピン
const int potentiometerPin = A2;    // ポテンショメータ用ピン
const int resetPin = 11;            // リセット用ピン
String current_command = "";        // コマンドを格納する変数

void stopIfFault();
int calcPotAngle(int potentiometerPin, int resetPin, String command);

void setup() {
  Serial.begin(9600);
  pinMode(limitSwitchUpperPin, INPUT);
  pinMode(limitSwitchLowerPin, INPUT);
  pinMode(resetPin, INPUT_PULLUP);
  md.init();
}

void loop() {
  int potAngle = calcPotAngle(potentiometerPin, resetPin, current_command);
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    
    if (command == "moveUp") { // 上昇
      Serial.println("moveUp");
      // 上部リミットスイッチが押されるまでモーターを動かす
      while (!digitalRead(limitSwitchUpperPin)) {
        md.setM1Speed(400); // 速度を調整
        stopIfFault();
      }
      md.setM1Speed(0); // リミットスイッチが押されたら停止
    } else if (command == "moveDown") { // 下降
      Serial.println("moveDown");
      // 下部リミットスイッチが押されるまでモーターを動かす
      while (!digitalRead(limitSwitchLowerPin)) {
        md.setM1Speed(-400); // 速度を調整
        stopIfFault();
      }
      md.setM1Speed(0); // リミットスイッチが押されたら停止
    }
    if (command == "expandArm" && potAngle > -10) { // 展開
      Serial.print("expandArm, potAngle: "); Serial.println(potAngle);
      current_command = "expandArm";
      // ポテンショメータが-10度より小さくなるまでモーターを動かす
      while (potAngle > -10) {
        potAngle = calcPotAngle(potentiometerPin, resetPin, current_command);
        md.setM1Speed(400); // 速度を調整
        stopIfFault();
      }
      md.setM1Speed(0); // ポテンショメータが-10度より小さくなったら停止
      current_command = "";
    } else if (command == "contractArm") { // 収納
      Serial.print("contractArm, potAngle: "); Serial.println(potAngle);
      current_command = "contractArm";
      // ポテンショメータが90度になるまでモーターを動かす
      while (-5 > (potAngle - 90) || (potAngle - 90) > 5) {
        potAngle = calcPotAngle(potentiometerPin, resetPin, current_command);
        if (potAngle > 90) {
          md.setM1Speed(400); // 速度を調整
        } else {
          md.setM1Speed(-400); // 速度を調整
        }
        stopIfFault();
      }
      md.setM1Speed(0); // ポテンショメータが90度になったら停止
      current_command = "";
    } else if (command == "resetArm" && potAngle < 180){ // 初期位置へ移動
      Serial.print("resetArm, potAngle: "); Serial.println(potAngle);
      current_command = "resetArm";
      while (potAngle < 180 || !digitalRead(resetPin)) {
        potAngle = calcPotAngle(potentiometerPin, resetPin, current_command);
        md.setM1Speed(-400); // 速度を調整
        stopIfFault();
      }
      md.setM1Speed(0); // ポテンショメータが180度になったら停止
      current_command = "";
    }
  }
}

void stopIfFault() {
  if (md.getM1Fault()) {
    Serial.println("M1 fault");
    while(1);
  }
  if (md.getM2Fault()) {
    Serial.println("M2 fault");
    while(1);
  }
}

int calcPotAngle(int potentiometerPin, int resetPin, String command) {
    // アームの展開制御
    int potValue = analogRead(potentiometerPin);
    static int base_angle = 0; // 基準角度を保存する変数（staticで初期化）
    int potAngle = map(potValue, 0, 1023, 0, 300); // ポテンショメータの値を角度に変換

    if (!digitalRead(resetPin)) { // リセットボタンが押されたら基準角度を更新
      base_angle = potAngle;
      potAngle = 180;
    } else { // リセットボタンが押されていない場合は基準角度を考慮
      potAngle = 180 - (potAngle - base_angle);
    }

    Serial.print("command: "); Serial.print(command);
    Serial.print(", potValue: "); Serial.print(potValue);
    Serial.print(", potAngle: "); Serial.println(potAngle);
    return potAngle;
}
