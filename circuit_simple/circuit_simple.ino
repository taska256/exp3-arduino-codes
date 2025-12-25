
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
const int SENSOR_THRESHOLD = 450;
// マイナスなら左,プラスなら右に寄っている
int p = 0;
long long i = 0;
int d = 0;

int rawL = 0;
int rawC = 1024;
int rawR = 0;

int wasRight = 0;
int prevError = 0;

const int basePower = 200;

const int Kp = 30;
const int KiInverse = 40;
const int Kd = 250;

int calcPowerL()
{
    long long calced = Kp * p + i / KiInverse + Kd * d;
    long long power = basePower - calced;
    if (power < 0)
        power = 0;
    if (power > 255)
        power = 255;
    return power;
}

int calcPowerR()
{
    long long calced = Kp * p + i / KiInverse + Kd * d;
    long long power = basePower + calced;
    if (power < 0)
        power = 0;
    if (power > 255)
        power = 255;
    return power;
}

void readSensor()
{
    rawL = analogRead(SENSOR_L);
    rawC = analogRead(SENSOR_C);
    rawR = analogRead(SENSOR_R);
}
bool resetIntegral = false;

void updatePID(int error)
{
    p = error;
    i += error;
    if (resetIntegral)
    {
        i = 0;
        resetIntegral = false;
    }
    d = (error - prevError);
    prevError = error;
}

int calcError()
{
    int isLBlack = rawL > SENSOR_THRESHOLD;
    int isCBlack = rawC > SENSOR_THRESHOLD;
    int isRBlack = rawR > SENSOR_THRESHOLD;
    resetIntegral = isCBlack;

    int sum = isLBlack + isCBlack + isRBlack;
    if (sum == 0)
    {
        return wasRight * 3;
    }

    // WBW -> 2/1 * 0 = 0
    // WBB -> 2/2 * -1 = -1 BBW-> 2/2 * 1 = 1
    // WWB -> 2/1 * -1 = -2 BWW-> 2/1 * 1 = 2
    int val = 2 / sum * (isLBlack - isRBlack);

    if (val > 0)
        wasRight = 1;
    else if (val < 0)
        wasRight = -1;

    return val;
}

void setMotor(int speedL, int speedR)
{
    digitalWrite(MOTOR_L_DIR1, LOW);
    digitalWrite(MOTOR_L_DIR2, HIGH);

    digitalWrite(MOTOR_R_DIR1, LOW);
    digitalWrite(MOTOR_R_DIR2, HIGH);

    analogWrite(MOTOR_L_PWM, speedL);
    analogWrite(MOTOR_R_PWM, speedR);
}

void loop()
{
    Serial.print("L:");
    Serial.print(rawL);
    Serial.print(" C:");
    Serial.print(rawC);
    Serial.print(" R:");
    Serial.print(rawR);
    Serial.print(" | P:");
    Serial.print(p);
    Serial.print(" I:");
    Serial.print((int)i);
    Serial.print(" D:");
    Serial.print(d);
    Serial.println();

    readSensor();
    long long error = calcError();
    updatePID(error);
    int speedL = calcPowerL();
    int speedR = calcPowerR();

    setMotor(speedL, speedR);
}