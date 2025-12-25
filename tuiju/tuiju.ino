#include <Arduino.h>
// --- ピン定義 ---
// FC-51 (LOW=障害物, HIGH=なし)
const int FC51_PIN = 13;

// 超音波 (HC-SR04)
const int R_TRIG_PIN = 12;
const int R_ECHO_PIN = 7;
const int L_TRIG_PIN = 8;
const int L_ECHO_PIN = 9;

// TA7291P
const int L_IN1 = 3;
const int L_IN2 = 2;
const int L_VREF = 10;

const int R_IN1 = 4;
const int R_IN2 = 5;
const int R_VREF = 11;

// --- 制御パラメータ ---
const float OUT_OF_RANGE_CM = 30.0f; // これより遠いのは「見失い」扱い
int BASE_SPEED = 0;
// 0/1化する閾値（これ以下なら「検出=1」）
const float DETECT_THRESHOLD_CM = 40.0f;

const float Kp_turn = 60.0f;
const float Ki_turn = 20.0f; // 要調整
const float Kd_turn = 15.0f; // 要調整
const float I_LIMIT = 5.0f;  // 積分の上限（誤差積分値をクランプ）

float g_iTerm = 0.0f;
float g_prevError = 0.0f;
unsigned long g_prevMs = 0;

// 追加: デバッグ出力用（PIDの各寄与）
float g_pOut = 0.0f; // P = Kp * error
float g_iOut = 0.0f; // I = Ki * iTerm
float g_dOut = 0.0f; // D = Kd * dTerm

// 超音波：エコー待ちタイムアウト（無応答で固まるのを防ぐ）
const unsigned long ULTRASONIC_TIMEOUT_US = 30000UL;

struct SensorReadings
{
    float leftCm;
    float rightCm;
    bool obstacle;
};

struct MotorCommand
{
    int left;
    int right;
};

int lastDiff = 0; // -1/0/1 を記憶

// --- 前方宣言 ---
static SensorReadings readSensors();
static MotorCommand computeCommand(const SensorReadings &s);
static void applyMotors(const MotorCommand &cmd);
static float readUltrasonicCm(int trigPin, int echoPin);
static float clampFloat(float v, float lo, float hi);
static int clampInt(int v, int lo, int hi);
static void debugPrint(const SensorReadings &s, const MotorCommand &cmd, int diff, bool lost);

void setup()
{
    Serial.begin(9600); // シリアル通信開始

    // センサーピンのモード設定
    pinMode(FC51_PIN, INPUT_PULLUP); // FC-51 (内蔵プルアップ使用)
    pinMode(R_TRIG_PIN, OUTPUT);
    pinMode(R_ECHO_PIN, INPUT);
    pinMode(L_TRIG_PIN, OUTPUT);
    pinMode(L_ECHO_PIN, INPUT);

    // モータードライバピンのモード設定 (すべて出力)
    pinMode(L_IN1, OUTPUT);
    pinMode(L_IN2, OUTPUT);
    pinMode(L_VREF, OUTPUT);
    pinMode(R_IN1, OUTPUT);
    pinMode(R_IN2, OUTPUT);
    pinMode(R_VREF, OUTPUT);

    applyMotors({0, 0});
    lastDiff = 0;
    BASE_SPEED = 255;
}
void loop()
{
    const SensorReadings s = readSensors();
    const MotorCommand cmd = computeCommand(s);
    applyMotors(cmd);
}

static SensorReadings readSensors()
{
    SensorReadings s;
    s.obstacle = (digitalRead(FC51_PIN) == LOW);

    s.rightCm = readUltrasonicCm(R_TRIG_PIN, R_ECHO_PIN);
    delay(20);
    s.leftCm = readUltrasonicCm(L_TRIG_PIN, L_ECHO_PIN);

    return s;
}
static MotorCommand computeCommand(const SensorReadings &s)
{
    const unsigned long nowMs = millis();

    // A) 最優先停止（FC-51）
    if (s.obstacle)
    {
        // 停止時はPID状態をリセット
        g_iTerm = 0.0f;
        g_prevError = 0.0f;
        g_prevMs = nowMs;

        const MotorCommand cmd{0, 0};
        debugPrint(s, cmd, 0, false);
        return cmd;
    }

    // B) 左右を閾値で 0/1 化（1=検出）
    const int L = (s.leftCm <= DETECT_THRESHOLD_CM) ? 1 : 0;
    const int R = (s.rightCm <= DETECT_THRESHOLD_CM) ? 1 : 0;

    // C) 先導機体が居る側へ曲がる
    int diff = 0;
    if (L == 1 && R == 0)
    {
        diff = -1;
    }
    else if (L == 0 && R == 1)
    {
        diff = 1;
    }
    else
    {
        diff = 0;
    }
    if (L == 1 && R == 1)
    {
        g_iTerm = 0.0f; // 両方見えたら積分リセット
    }

    // D) 両方見えないなら、前回の方向へ探索
    const bool lost = (L == 0 && R == 0);
    if (lost)
    {
        diff = 2 * lastDiff;
    }
    else
    {
        lastDiff = diff;
    }

    // E) PID制御（baseSpeedは固定）
    float dt = (g_prevMs == 0) ? 0.02f : (float)(nowMs - g_prevMs) / 1000.0f;
    if (dt < 0.001f)
        dt = 0.001f;
    g_prevMs = nowMs;

    const float error = (float)diff;

    // LOST中も積分する（要望）
    g_iTerm += error * dt;
    g_iTerm = clampFloat(g_iTerm, -I_LIMIT, I_LIMIT);

    const float dTerm = (error - g_prevError) / dt;
    g_prevError = error;

    g_pOut = Kp_turn * error;
    g_iOut = Ki_turn * g_iTerm;
    g_dOut = Kd_turn * dTerm;

    const float turn = g_pOut + g_iOut + g_dOut;
    // const float turn = g_pOut;

    const long turnAmount = turn;

    MotorCommand cmd;
    cmd.left = clampInt(BASE_SPEED - turnAmount, 50, 255);
    cmd.right = clampInt(BASE_SPEED + turnAmount, 50, 255);

    debugPrint(s, cmd, diff, lost);
    return cmd;
}

static void applyMotors(const MotorCommand &cmd)
{
    // 左モーター
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, HIGH);
    analogWrite(L_VREF, cmd.left);

    // 右モーター
    digitalWrite(R_IN1, LOW);
    digitalWrite(R_IN2, HIGH);
    analogWrite(R_VREF, cmd.right);
}

static float readUltrasonicCm(int trigPin, int echoPin)
{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // タイムアウトを使う（無応答で固まるのを防ぐ）
    const unsigned long duration = pulseIn(echoPin, HIGH, ULTRASONIC_TIMEOUT_US);
    if (duration == 0)
    {
        // 無応答は「範囲外」として扱う
        return OUT_OF_RANGE_CM + 100.0f;
    }

    // 音速 340m/s -> 0.034 cm/us
    return (float)duration * 0.034f / 2.0f;
}

static float clampFloat(float v, float lo, float hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

static int clampInt(int v, int lo, int hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

static void debugPrint(const SensorReadings &s, const MotorCommand &cmd, int diff, bool lost)
{
    Serial.print("L: ");
    Serial.print(s.leftCm);
    Serial.print(" cm, R: ");
    Serial.print(s.rightCm);
    Serial.print(" cm, IR: ");
    Serial.print(s.obstacle ? 0 : 1);

    if (lost)
    {
        Serial.print(" | LOST diff: ");
        Serial.print(diff);
    }
    else
    {
        Serial.print(" | diff: ");
        Serial.print(diff);
    }

    // 追加: PID表示（各寄与）
    Serial.print(" | P:");
    Serial.print(g_pOut, 2);
    Serial.print(" I:");
    Serial.print(g_iOut, 2);
    Serial.print(" D:");
    Serial.print(g_dOut, 2);

    Serial.print(" | L_sp: ");
    Serial.print(cmd.right);
    Serial.print(", R_sp: ");
    Serial.println(cmd.left);
}
