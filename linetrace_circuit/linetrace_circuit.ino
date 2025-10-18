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

    Serial.println("------------Start-------------");
}

float p = 0; // マイナスなら左,プラスなら右に機体の頭が寄っている
float i = 0; // loop一回にかかる時間によるが、すぐに1000に飽和する可能性あり その場合はpと変わらず積分の意味がないため修正の必要あり
float d = 0;

float prevError = 0;

int calcPowerL()
{
    // p:[-2~2] almost [-1~1]
    // i:[-1000~1000] almost 0
    // d:[-2~2] almost [-1~1]
    const float Kp = 60;   // Kp*p: [-120~120] almost [-60~60]
    const float Ki = 0.05; // Ki*i: [-50~50] almost 0
    const float Kd = -50;  // Kd*d: [-100~100] almost [-50~50]

    return 255 - Kp * p - Ki * i - Kd * d;
}

int calcPowerR()
{
    const float Kp = 60;
    const float Ki = 0.05;
    const float Kd = -50;

    return 255 + Kp * p + Ki * i + Kd * d;
}

void updatePID(float error)
{
    p = error;
    i += error;
    d = error - prevError;

    const float I_MAX = 1000.0;

    i = constrain(i, -I_MAX, I_MAX);
    prevError = error;
}

float calcError(float rawL, float rawC, float rawR)
{
    // 範囲 -2~2
    //  全てのセンサ(rawL,rawC,rawR)は黒(1023)になり続けることを目標とする

    // 中央センサがライン外の場合は補正を強化
    return (rawL - rawR) * (2.0 - rawC / 1024.0) / 1024.0;
    // idea: L,Rは白でCだけ黒を目指し、L,Rは黒線から少し離すようにすると振動は無くなりそう？
}

void printSensorLog(int rawL, int rawC, int rawR)
{
    static unsigned long lastPrintTime = 0;
    unsigned long currentTime = millis();

    if (currentTime - lastPrintTime > 100)
    {
        Serial.print("\n\n\n\n\n\n\n\n\n\n");

        Serial.print("\n\nL:");
        for (int i = 0; i < rawL * 10 / 1024; i++)
            Serial.print("#");
        Serial.print("\n\nM:");
        for (int i = 0; i < rawC * 10 / 1024; i++)
            Serial.print("#");
        Serial.print("\n\nR:");
        for (int i = 0; i < rawR * 10 / 1024; i++)
            Serial.print("#");

        Serial.println();

        Serial.print("P:");
        Serial.println(p);
        Serial.print(" I:");
        Serial.println(i);
        Serial.print(" D:");
        Serial.println(d);

        lastPrintTime = currentTime;
    }
}

void readSensor()
{
    int rawL = analogRead(SENSOR_L);
    int rawC = analogRead(SENSOR_C);
    int rawR = analogRead(SENSOR_R);

    // printSensorLog(rawL, rawC, rawR);

    updatePID(calcError(rawL, rawC, rawR));
}

void setMotor(int speedL, int speedR)
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
    /*0.3秒で約1000回転すると、0.3秒で大体積分項が1000に飽和する
    この行をつけた場合とつけていない場合で積分項が強く働く波部分のライン走行に違いが出るか確認する*/
    // delayMicroseconds(256);

    readSensor();

    setMotor(calcPowerL(), calcPowerR());
}