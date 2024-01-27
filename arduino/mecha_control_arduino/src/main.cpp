#include <Arduino.h>
#include <Servo.h> 
Servo servoRight; // サーボモーターのインスタンスを作成
Servo servoLeft; // サーボモーターのインスタンスを作成

// サーボモーター制御用のピン
const int servoPinRight = 11;             // サーボモーター用ピン
const int servoPinLeft = 12;              // サーボモーター用ピン

// 上下展開用のモーター制御用のピン
const int motorExpandPin = 9;       // モーター展開用ピン
const int motorRetractPin = 10;      // モーター縮小用ピン

// アーム展開用のモーター制御用のピン
const int armMotorExpandPin = 6;    // アームモーター展開用ピン
const int armMotorRetractPin = 7;   // アームモーター縮小用ピン

const int expandSpeed = 128;         // 展開時のモーターの速度
const int retractSpeed = 128;        // 縮小時のモーターの速度

const int limitSwitchUpperPin = 3;  // 上部リミットスイッチ用ピン
const int limitSwitchLowerPin = 5;  // 下部リミットスイッチ用ピン
const int potentiometerPin = A2;    // ポテンショメータ用ピン
const int resetPin = 11;            // リセット用ピン

// エアシリンダ制御用のピン
const int armOpenPin = 45;          // 箱のアーム展開ピン
const int armClosePin = 43;         // 箱のアーム閉じるピン
const int boxArmExpandPin = 49;     // 箱を倒すアームの展開ピン
const int boxArmRetractPin = 47;    // 箱を倒すアームの閉じるピン
const int mechaExpandPin = 51;      // 機構全体を展開するピン
const int mechaRetractPin = 53;     // 機構全体を縮小するピン

String current_command = "";        // コマンドを格納する変数

int calcPotAngle(int potentiometerPin, int resetPin, String command);
void activateCylinder(int pin, int delayTime);

void setup() {
  Serial.begin(9600);
  pinMode(limitSwitchUpperPin, INPUT_PULLUP);
  pinMode(limitSwitchLowerPin, INPUT_PULLUP);
  pinMode(resetPin, INPUT_PULLUP);
  pinMode(potentiometerPin, INPUT);
  pinMode(motorExpandPin, OUTPUT);
  pinMode(motorRetractPin, OUTPUT);
  pinMode(armMotorExpandPin, OUTPUT);
  pinMode(armMotorRetractPin, OUTPUT);
  pinMode(armOpenPin, OUTPUT);
  pinMode(armClosePin, OUTPUT);
  pinMode(boxArmExpandPin, OUTPUT);
  pinMode(boxArmRetractPin, OUTPUT);
  pinMode(mechaExpandPin, OUTPUT);
  pinMode(mechaRetractPin, OUTPUT);
  servoRight.attach(servoPinRight); // サーボモーターのピンを指定
  servoLeft.attach(servoPinLeft); // サーボモーターのピンを指定
  servoRight.write(0); // サーボモーターを0度に動かす
  servoLeft.write(0); // サーボモーターを0度に動かす
  Serial.println("起動卍");
}

void loop() {
  int potAngle = calcPotAngle(potentiometerPin, resetPin, current_command);
  // スイッチの状態を表示
  int potValue = analogRead(potentiometerPin);
  Serial.print("potValue: "); Serial.print(potValue);
  Serial.print("limitSwitchUpperPin: "); Serial.print(digitalRead(limitSwitchUpperPin));
  Serial.print(", limitSwitchLowerPin: "); Serial.print(digitalRead(limitSwitchLowerPin));
  Serial.print(", resetPin: "); Serial.print(digitalRead(resetPin));
  Serial.print(", potAngle: "); Serial.println(potAngle);

  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');

    if (command == "mechaExpand") {
      Serial.print("動作開始: mechaExpand");
      activateCylinder(mechaExpandPin, 1000);
      Serial.print("動作完了: mechaExpand");
    } else if (command == "mechaRetract") {
      Serial.print("動作開始: mechaRetract");
      activateCylinder(mechaRetractPin, 1000);
      Serial.print("動作完了: mechaRetract");
    } else if (command == "armOpen") {
      Serial.print("動作開始: armOpen");
      activateCylinder(armOpenPin, 1000);
      Serial.print("動作完了: armOpen");
    } else if (command == "armClose") {
      Serial.print("動作開始: armClose");
      activateCylinder(armClosePin, 2000);
      Serial.print("動作完了: armClose");
    } else if (command == "boxArmExpand") {
      Serial.print("動作開始: boxArmExpand");
      activateCylinder(boxArmExpandPin, 1000);
      Serial.print("動作完了: boxArmExpand");
    } else if (command == "boxArmRetract") {
      Serial.print("動作開始: boxArmRetract");
      activateCylinder(boxArmRetractPin, 1000);
      Serial.print("動作完了: boxArmRetract");
    }
    if (command == "moveUp") { // 上昇
      if (digitalRead(limitSwitchUpperPin)){
        Serial.print("moveUp");
        // 上部リミットスイッチが押されるまでモーターを動かす
        while (digitalRead(limitSwitchUpperPin)) {
          analogWrite(motorExpandPin, expandSpeed); // 速度を調整
          analogWrite(motorRetractPin, 0); // 速度を調整
        }
        Serial.print("moveUp end");
        analogWrite(motorExpandPin, 0); // リミットスイッチが押されたら停止
      } else {
        Serial.print("can't moveUp because limitSwitchUpperPin is pressed");
      }
      command = "";
    } else if (command == "moveDown") { // 下降
      if (digitalRead(limitSwitchLowerPin)) {
        Serial.print("moveDown");
        // 下部リミットスイッチが押されるまでモーターを動かす
        while (digitalRead(limitSwitchLowerPin)) {
          analogWrite(motorRetractPin, expandSpeed); // 速度を調整
          // analogWrite(motorRetractPin, 200); // 速度を調整
          analogWrite(motorExpandPin, 0); // 速度を調整
        }
        Serial.print("moveDown end");
        analogWrite(motorRetractPin, 0); // リミットスイッチが押されたら停止
      } else {
        Serial.print("can't moveDown because limitSwitchLowerPin is pressed");
      }
      command = "";
    }
    if (command == "expandArm" && potAngle > -10) { // 展開
      Serial.print("expandArm");
      current_command = "expandArm";
      // ポテンショメータが-20度より小さくなるまでモーターを動かす
      while (potAngle > -20) {
        potAngle = calcPotAngle(potentiometerPin, resetPin, current_command);
        analogWrite(armMotorExpandPin, retractSpeed); // 速度を調整
        analogWrite(armMotorRetractPin, 0);
        // delay(10);
        // analogWrite(armMotorExpandPin, retractSpeed); // 速度を調整
        // analogWrite(armMotorRetractPin, retractSpeed);
        // delay(100);
      }
      Serial.print("expandArm end");
      analogWrite(armMotorExpandPin, 250); // ポテンショメータが-10度より小さくなったら停止
      analogWrite(armMotorRetractPin, 250);
      current_command = "";
    } else if (command == "contractArm") { // 収納
      Serial.print("contractArm");
      current_command = "contractArm";
      // ポテンショメータが90度になるまでモーターを動かす
      while (-1 > (potAngle - 90) || (potAngle - 90) > 1) {
        potAngle = calcPotAngle(potentiometerPin, resetPin, current_command);
        if (potAngle > 90) {
          analogWrite(armMotorExpandPin, expandSpeed); // 速度を調整
          analogWrite(armMotorRetractPin, 0);
          // delay(10);
          // analogWrite(armMotorExpandPin, 0); // 速度を調整
          // analogWrite(armMotorRetractPin, 0);
          // delay(100);
        } else {
          analogWrite(armMotorExpandPin, 0);
          analogWrite(armMotorRetractPin, expandSpeed); // 速度を調整
          // delay(10);
          // analogWrite(armMotorExpandPin, 0); // 速度を調整
          // analogWrite(armMotorRetractPin, 0);
          // delay(100);
        }
      }
      Serial.print("contractArm end");
      analogWrite(armMotorExpandPin, 250); // ポテンショメータが90度になったら停止
      analogWrite(armMotorRetractPin, 250); // ポテンショメータが90度になったら停止
      current_command = "";
    } else if (command == "resetArm" && potAngle < 180){ // 初期位置へ移動
      if (!digitalRead(limitSwitchUpperPin)) { // 上部リミットスイッチが押されている場合は初期位置へ移動
        Serial.print("resetArm");
        current_command = "resetArm";
        while (potAngle < 180 || !digitalRead(resetPin)) {
          potAngle = calcPotAngle(potentiometerPin, resetPin, current_command);
          analogWrite(armMotorExpandPin, 0);
          analogWrite(armMotorRetractPin, expandSpeed); // 速度を調整
          // delay(10);
          // analogWrite(armMotorExpandPin, 0);
          // analogWrite(armMotorRetractPin, 0); // 速度を調整
          // delay(100);
        }
        Serial.print("resetArm end");
        analogWrite(armMotorExpandPin, 250); // ポテンショメータが180度になったら停止
        analogWrite(armMotorRetractPin, 250); // ポテンショメータが180度になったら停止
        current_command = "";
      } else {
        Serial.print("can't resetArm because limitSwitchUpperPin is not pressed");
        command = "";
      }
    } else if (command == "push" && potAngle < 0) { // 押し出し解放サーボを動かす。
      Serial.print("push");
      servoRight.write(0);
      servoLeft.write(0);
      delay(1000);
      servoRight.write(90);
      servoLeft.write(90);
      delay(1000);
      command = "";
    } else if (command == "0degree") {
      Serial.print("0degree");
      servoRight.write(0);
      servoLeft.write(0);
      command = "";
    } else if (command == "90degree") {
      Serial.print("90degree");
      servoRight.write(90);
      servoLeft.write(90);
      command = "";
    }
    if (command == "bonbori") {
      Serial.println("bonbori");
    }
  }
}

int calcPotAngle(int potentiometerPin, int resetPin, String command) {
    // アームの展開制御
    int potValue = analogRead(potentiometerPin);
    static int base_angle = 0; // 基準角度を保存する変数（staticで初期化）
    int potAngle = map(potValue, 0, 1023, 0, 300); // ポテンショメータの値を角度に変換

    // if (!digitalRead(resetPin)) { // リセットボタンが押されたら基準角度を更新
    //   base_angle = potAngle;
    //   potAngle = 180;
    // } else { // リセットボタンが押されていない場合は基準角度を考慮
    potAngle = 180 - (potAngle - base_angle);
    // }

    // Serial.print("command: "); Serial.print(command);
    // Serial.print(", potValue: "); Serial.print(potValue);
    // Serial.print(", potAngle: "); Serial.println(potAngle);
    return potAngle;
}

void activateCylinder(int pin, int delayTime = 1000) {
  digitalWrite(pin, HIGH);
  delay(delayTime);
  digitalWrite(pin, LOW);
}