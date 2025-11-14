
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
    pinMode(MOTOR_L_PWM, OUTPUT);
    pinMode(MOTOR_L_DIR1, OUTPUT);
    pinMode(MOTOR_L_DIR2, OUTPUT);
    pinMode(MOTOR_R_PWM, OUTPUT);
    pinMode(MOTOR_R_DIR1, OUTPUT);
    pinMode(MOTOR_R_DIR2, OUTPUT);
}
// マイナスなら左,プラスなら右に寄っている
long long p = 0;
long long i = 0;
long long d = 0;

long long rawL = 0;
long long rawC = 1024;
long long rawR = 0;

long long wasRight = 0;

long long prevError = 0;

const long long basePower = 120;

const long long Kp = 30;
const long long KiInverse = 1024;
const long long Kd = 15;

long long calcPowerL()
{
    long long calced = Kp * p + i / KiInverse + Kd * d;
    long long power = basePower - calced;
    if (power < 0)
        power = 0;
    if (power > 255)
        power = 255;
    return power;
}

long long calcPowerR()
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

void updatePID(long long error)
{
    p = error;
    i += error;
    d = (error - prevError);
    prevError = error;
}

long long calcError()
{
    int isLBlack = rawL > SENSOR_THRESHOLD;
    int isCBlack = rawC > SENSOR_THRESHOLD;
    int isRBlack = rawR > SENSOR_THRESHOLD;

    long long sum = isLBlack + isCBlack + isRBlack;
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

void setMotor(long long speedL, long long speedR)
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
    readSensor();
    long long error = calcError();
    updatePID(error);
    long long speedL = calcPowerL();
    long long speedR = calcPowerR();

    setMotor(speedL, speedR);
}