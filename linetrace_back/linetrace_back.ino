#include <Arduino.h>

const int SENSOR_L = A2;
const int SENSOR_C = A1;
const int SENSOR_R = A0;
// 追加: 後方センタセンサ
const int SENSOR_BC = A3;
// 追加: 後方センタの生値
int rawBC = 0;

const int MOTOR_L_PWM = 10;
const int MOTOR_L_DIR1 = 2;
const int MOTOR_L_DIR2 = 3;
const int MOTOR_R_PWM = 11;
const int MOTOR_R_DIR1 = 4;
const int MOTOR_R_DIR2 = 5;

void setup()
{
    Serial.begin(115200); // ログを使うとき有効化

    pinMode(MOTOR_L_PWM, OUTPUT);
    pinMode(MOTOR_L_DIR1, OUTPUT);
    pinMode(MOTOR_L_DIR2, OUTPUT);
    pinMode(MOTOR_R_PWM, OUTPUT);
    pinMode(MOTOR_R_DIR1, OUTPUT);
    pinMode(MOTOR_R_DIR2, OUTPUT);
}

float p = 0; // マイナスなら左,プラスなら右に機体の頭が寄っている
float i = 0; // loop一回にかかる時間によるが、すぐに1000に飽和する可能性あり その場合はpと変わらず積分の意味がないため修正の必要あり
float d = 0;

float prevError = 0;

// 追加: 状態管理（0:前進PID, 1:等速後退, 2:後退PID）
int runState = 0;
unsigned long backStartTime = 0;
const int BACK_STRAIGHT_SPEED = 150;
void resetPID()
{
    p = 0;
    i = 0;
    d = 0;
    prevError = 0;
}

int calcPowerL()
{
    // p:[-2~2] almost [-1~1]
    // i:[-1000~1000] almost 0
    // d:[-2~2] almost [-1~1]
    const float Kp = 20;      // Kp*p: [-120~120] almost [-60~60]
    const float Ki = 0.00005; // Ki*i: [-50~50] almost 0
    const float Kd = -10;     // 負で振動抑制 Kd*d: [-100~100] almost [-50~50]
    float calced = Kp * p + Ki * i + Kd * d;

    return (100 - 1.4 * calced);
}

int calcPowerR()
{
    const float Kp = 20;
    const float Ki = 0.00005;
    const float Kd = -10;
    float calced = Kp * p + Ki * i + Kd * d;

    return (100 + 1.4 * calced);
}
int calcPowerLBack()
{
    int error = 0;
    if (rawBC - 960 < 0)
    {
        error = 1;
    }
    else
    {
        error = -1;
    }
    return (80 + 20 * error);
}

int calcPowerRBack()
{

    int error = 0;
    if (rawBC - 960 < 0)
    {
        error = 1;
    }
    else
    {
        error = -1;
    }
    return (80 - 20 * error);
}

void updatePID(float error)
{
    p = error;
    i += error;
    d = (error - prevError);

    const float I_MAX = 10000.0;

    i = constrain(i, -I_MAX, I_MAX);
    prevError = error;
}

int isLeftNow = -1;

// 全て白の時にエラーが0になってしまう
float calcError(float rawL, float rawC, float rawR)
{
    // 範囲 -2~2
    //  全てのセンサ rawL,rawRは白 rawCは黒 -> 0
    // 白の時低くなる
    // 黒の時高くなる

    int threshold = 500;
    if (rawL < threshold && rawC < threshold && rawR < threshold)
    {
        return isLeftNow * 2;
    }

    if (rawC > threshold)
    {
        i = 0;
    }

    float val = -((1024 - rawL) - (1024 - rawR)) * (2.0 - rawC / 1024.0) / 1024.0;
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

void printSensorLog(int rawL, int rawC, int rawR)
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

        lastPrintTime = currentTime;
    }
}
int rawL = 0;
int rawC = 0;
int rawR = 0;

void readSensor()
{
    rawL = analogRead(SENSOR_L);
    rawC = analogRead(SENSOR_C);
    rawR = analogRead(SENSOR_R);
    rawBC = analogRead(SENSOR_BC);

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

void setMotorBack(int speedL, int speedR)
{
    digitalWrite(MOTOR_L_DIR1, HIGH);
    digitalWrite(MOTOR_L_DIR2, LOW);

    digitalWrite(MOTOR_R_DIR1, HIGH);
    digitalWrite(MOTOR_R_DIR2, LOW);

    analogWrite(MOTOR_L_PWM, constrain(speedL, 0, 255));
    analogWrite(MOTOR_R_PWM, constrain(speedR, 0, 255));
}

int checkEdge()
{
    int threshold = 500;
    if (rawL < threshold && rawC < threshold && rawR < threshold)
    {
        return 1;
    }
    return 0;
}

int isBack = 0;

void loop()
{
    readSensor();
    Serial.print("rawBC:");
    Serial.println(rawBC);
    if (runState == 0)
    {
        // 前進PID

        setMotor(calcPowerL(), calcPowerR());

        if (checkEdge())
        {
            runState = 1;
            backStartTime = millis();
            resetPID();
        }
    }

    if (runState == 1)
    {
        // 1秒間 等速で真後ろへ（PIDなし）
        setMotorBack(BACK_STRAIGHT_SPEED, BACK_STRAIGHT_SPEED);
        if (millis() - backStartTime >= 1000)
        {
            runState = 2;
            resetPID();
        }
    }
    if (runState == 2)
    {

        // runState == 2: 後退PID制御（A3を使用）
        setMotorBack(calcPowerLBack(), calcPowerRBack());
    }
}