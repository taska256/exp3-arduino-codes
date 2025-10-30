#include <Arduino.h>

const int SENSOR_L = A2;
const int SENSOR_C = A1;
const int SENSOR_R = A0;
const int MOTOR_L_PWM = 10;
const int MOTOR_L_DIR1 = 2;
const int MOTOR_L_DIR2 = 3;
const int MOTOR_R_PWM = 11;
const int MOTOR_R_DIR1 = 4;
const int MOTOR_R_DIR2 = 5;

void setup()
{
    Serial.begin(9600);
    pinMode(MOTOR_L_PWM, OUTPUT);
    pinMode(MOTOR_L_DIR1, OUTPUT);
    pinMode(MOTOR_L_DIR2, OUTPUT);
    pinMode(MOTOR_R_PWM, OUTPUT);
    pinMode(MOTOR_R_DIR1, OUTPUT);
    pinMode(MOTOR_R_DIR2, OUTPUT);
}

double p = 0; // マイナスなら左,プラスなら右に機体の頭が寄っている
double i = 0;
double d = 0;

double prevError = 0;

// 移動先: 積分リセットは updatePID で実施するためのフラグ
const long SENSOR_THRESHOLD = 500;
bool resetIntegral = false;

long calcPowerL()
{
    const double Kp = 30;
    const double Ki = 0.001;
    const double Kd = -20;
    double calced = Kp * p + Ki * i + Kd * d;

    return 100 - calced;
}

long calcPowerR()
{
    const double Kp = 30;
    const double Ki = 0.001;
    const double Kd = -20;
    double calced = Kp * p + Ki * i + Kd * d;

    return 100 + calced;
}

void updatePID(double error)
{
    p = error;

    // ここで「加算前」に積分リセット
    if (resetIntegral)
    {
        i = 0;
        prevError = error; // D項のキック抑制
        resetIntegral = false;
    }
    else
    {
        i += error;
        // アンチワインドアップ（必要に応じて調整）
        const double I_MAX = 1000.0;
        if (i > I_MAX)
            i = I_MAX;
        if (i < -I_MAX)
            i = -I_MAX;
    }

    d = (error - prevError);
    prevError = error;
}

long isLeftNow = -1;

// // 全て白の時にエラーが0になってしまう
double calcError(double rawL, double rawC, double rawR)
{
    // リセット条件の判定はここで行い、実際のリセットは updatePID に委ねる
    // 1) 真ん中が黒（ライン上）
    // 2) 左右が片方だけ黒/白（交差や大きな偏り）
    resetIntegral =
        (rawC > SENSOR_THRESHOLD) ||
        ((rawL < SENSOR_THRESHOLD) != (rawR < SENSOR_THRESHOLD));

    // 全白（ラインロスト）時のフェイルセーフ（従来通り）
    if (rawL < SENSOR_THRESHOLD && rawC < SENSOR_THRESHOLD && rawR < SENSOR_THRESHOLD)
    {
        return isLeftNow * 2.5;
    }

    // 白の時低くなる / 黒の時高くなる前提
    double val = -((1024 - rawL) - (1024 - rawR)) * (2.0 - rawC / 1024.0) / 1024.0;
    if (val < 0)
    {
        isLeftNow = -1;
    }
    else
    {
        isLeftNow = 1;
    }

    return val;
}

void printSensorLog(long rawL, long rawC, long rawR)
{
    static unsigned long lastPrintTime = 0;
    unsigned long currentTime = millis();

    if (currentTime - lastPrintTime > 100)
    {

        Serial.print("L:");
        Serial.print(rawL);
        Serial.print(" ");
        Serial.print("C:");
        Serial.print(rawC);
        Serial.print(" ");
        Serial.print("R:");
        Serial.println(rawR);

        // デバッグしたい場合は以下を有効化
        // Serial.print(" P:"); Serial.print(p);
        // Serial.print(" I:"); Serial.print(i);
        // Serial.print(" D:"); Serial.print(d);
        // Serial.print(" resetI:"); Serial.println(resetIntegral);

        lastPrintTime = currentTime;
    }
}

void readSensor()
{
    long rawL = analogRead(SENSOR_L);
    long rawC = analogRead(SENSOR_C);
    long rawR = analogRead(SENSOR_R);

    printSensorLog(rawL, rawC, rawR);

    updatePID(calcError(rawL, rawC, rawR));
}

void setMotor(long speedL, long speedR)
{
    digitalWrite(MOTOR_L_DIR1, LOW);
    digitalWrite(MOTOR_L_DIR2, HIGH);

    digitalWrite(MOTOR_R_DIR1, LOW);
    digitalWrite(MOTOR_R_DIR2, HIGH);

    analogWrite(MOTOR_L_PWM, constrain(speedL, 0, 255));
    analogWrite(MOTOR_R_PWM, constrain(speedR, 0, 255));
}

void loop()
{
    // delayMicroseconds(256);

    readSensor();

    setMotor(calcPowerL(), calcPowerR());
}