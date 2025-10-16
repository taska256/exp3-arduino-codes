#include <Arduino.h>

// ピン定義（constで定義）
const int SENSOR_L = A2;
const int SENSOR_M = A1;
const int SENSOR_R = A0;
const int MOTOR_L_PWM = 10;
const int MOTOR_L_DIR1 = 2;
const int MOTOR_L_DIR2 = 3;
const int MOTOR_R_PWM = 11;
const int MOTOR_R_DIR1 = 4;
const int MOTOR_R_DIR2 = 5;

// センサー閾値
int threshold = 500;

// 基本速度
int baseSpeed = 50;
int turnSpeed = 100;  // 旋回時の速度差
int recoverySpeed = 150;  // ライン見失い時の強い補正速度

// 直前の傾き方向を保持する変数
// -1: 左寄り, 0: 中央, 1: 右寄り
int lastDirection = 0;

void setup() {
    // シリアル通信の初期化
    Serial.begin(9600);
    
    // モーターピンの設定
    pinMode(MOTOR_L_PWM, OUTPUT);
    pinMode(MOTOR_L_DIR1, OUTPUT);
    pinMode(MOTOR_L_DIR2, OUTPUT);
    pinMode(MOTOR_R_PWM, OUTPUT);
    pinMode(MOTOR_R_DIR1, OUTPUT);
    pinMode(MOTOR_R_DIR2, OUTPUT);
    
    Serial.println("Line Tracer Start!");
}

void loop() {
    // センサー値の読み取り
    int sensorL = analogRead(SENSOR_L);
    int sensorM = analogRead(SENSOR_M);
    int sensorR = analogRead(SENSOR_R);
    
    // ON/OFF制御実行
    onOffControl(sensorL, sensorM, sensorR);
    
    // デバッグ情報の表示
    printDebugInfo(sensorL, sensorM, sensorR);
    
    delay(10);  // 10msごとに実行
}

void onOffControl(int sensorL, int sensorM, int sensorR) {
    // センサー値から黒白判定
    bool isBlackL = sensorL > threshold;
    bool isBlackM = sensorM > threshold;
    bool isBlackR = sensorR > threshold;
    
    int leftMotor = 0;
    int rightMotor = 0;
    
    // ON/OFF制御：センサーの状態に応じてモーターを制御
    if (!isBlackL && isBlackM && !isBlackR) {
        // 中央センサーのみ黒：直進
        leftMotor = baseSpeed;
        rightMotor = baseSpeed;
        lastDirection = 0;  // 中央を記録
        Serial.println("状態: 直進");
    } 
    else if (isBlackL && !isBlackR) {
        // 左センサーが黒：右旋回
        leftMotor = baseSpeed;
        rightMotor = turnSpeed;
        lastDirection = -1;  // 左寄りを記録
        Serial.println("状態: 右旋回");
    } 
    else if (!isBlackL && isBlackR) {
        // 右センサーが黒：左旋回
        leftMotor = turnSpeed;
        rightMotor = baseSpeed;
        lastDirection = 1;  // 右寄りを記録
        Serial.println("状態: 左旋回");
    } 
    else if (isBlackL && isBlackM && isBlackR) {
        // 全センサー黒：交差点などで直進
        leftMotor = baseSpeed;
        rightMotor = baseSpeed;
        // lastDirectionは維持
        Serial.println("状態: 交差点");
    } 
    else if (!isBlackL && !isBlackM && !isBlackR) {
        // 全センサー白：ラインを見失った場合は直前の方向に強く補正
        if (lastDirection < 0) {
            // 左寄りだった場合：強く右旋回
            leftMotor = recoverySpeed;
            rightMotor = 0;
            Serial.println("状態: ライン見失い - 強く右旋回");
        } else if (lastDirection > 0) {
            // 右寄りだった場合：強く左旋回
            leftMotor = 0;
            rightMotor = recoverySpeed;
            Serial.println("状態: ライン見失い - 強く左旋回");
        } else {
            // 中央だった場合：停止（安全のため）
            leftMotor = 0;
            rightMotor = 0;
            Serial.println("状態: ライン見失い - 停止");
        }
    }
    
    // モーター制御
    setMotor(leftMotor, rightMotor);
}

void setMotor(int leftSpeed, int rightSpeed) {
    // 左モーター制御
    if (leftSpeed > 0) {
        digitalWrite(MOTOR_L_DIR1, LOW);
        digitalWrite(MOTOR_L_DIR2, HIGH);
        analogWrite(MOTOR_L_PWM, abs(leftSpeed) * 255 / 100);
    } else {
        digitalWrite(MOTOR_L_DIR1, LOW);
        digitalWrite(MOTOR_L_DIR2, LOW);
        analogWrite(MOTOR_L_PWM, 0);
    }
    
    // 右モーター制御
    if (rightSpeed > 0) {
        digitalWrite(MOTOR_R_DIR1, LOW);
        digitalWrite(MOTOR_R_DIR2, HIGH);
        analogWrite(MOTOR_R_PWM, abs(rightSpeed) * 255 / 100);
    } else {
        digitalWrite(MOTOR_R_DIR1, LOW);
        digitalWrite(MOTOR_R_DIR2, LOW);
        analogWrite(MOTOR_R_PWM, 0);
    }
}

void printDebugInfo(int sensorL, int sensorM, int sensorR) {
    static unsigned long lastPrintTime = 0;
    unsigned long currentTime = millis();
    
    // 500msごとに表示
    if (currentTime - lastPrintTime > 500) {
        Serial.print("L:");
        Serial.print(sensorL);
        Serial.print(" M:");
        Serial.print(sensorM);
        Serial.print(" R:");
        Serial.print(sensorR);
        Serial.print(" | LastDir:");
        Serial.println(lastDirection);
        
        lastPrintTime = currentTime;
    }
}

// キャリブレーション関数（オプション）
void calibrateSensors() {
    Serial.println("Calibrating sensors...");
    Serial.println("Place sensors on WHITE surface");
    delay(3000);
    
    int whiteL = analogRead(SENSOR_L);
    int whiteM = analogRead(SENSOR_M);
    int whiteR = analogRead(SENSOR_R);
    
    Serial.println("Place sensors on BLACK surface");
    delay(3000);
    
    int blackL = analogRead(SENSOR_L);
    int blackM = analogRead(SENSOR_M);
    int blackR = analogRead(SENSOR_R);
    
    // 閾値を中間値に設定
    threshold = 500;
    
    Serial.print("Threshold set to: ");
    Serial.println(threshold);
}