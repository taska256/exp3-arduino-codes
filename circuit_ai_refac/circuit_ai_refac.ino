#include <Arduino.h>

// ======================= ピン定義 =======================
constexpr int SENSOR_L_PIN = A2;
constexpr int SENSOR_C_PIN = A1;
constexpr int SENSOR_R_PIN = A0;

constexpr int MOTOR_L_PWM = 10;
constexpr int MOTOR_L_DIR1 = 2;
constexpr int MOTOR_L_DIR2 = 3;
constexpr int MOTOR_R_PWM = 11;
constexpr int MOTOR_R_DIR1 = 4;
constexpr int MOTOR_R_DIR2 = 5;

// ======================= センサ・PID関連定数 =======================
constexpr long SENSOR_THRESHOLD = 500;
constexpr double KP = 30.0;
constexpr double KI = 0.001;
constexpr double KD = -20.0;
constexpr double I_MAX = 1000.0;
constexpr double LINE_LOST_ERROR = 2.5;   // ラインロスト時の「戻り」強さ
constexpr double LINE_SIDE_DEADBAND = 1.; // 左右判定のデッドバンド

// ======================= 速度プロファイル関係 =======================
constexpr int BASE_STRAIGHT = 255; // 直線
constexpr int BASE_CURVE = 160;    // カーブ
constexpr int BASE_ZIGZAG = 70;    // グネグネ/迷子

constexpr double P_STRAIGHT_THR = 0.15;      // ジグザグ判定用しきい値（pの符号を見る）
constexpr unsigned long ZIGZAG_WINDOW = 300; // ms 内の反転回数でジグザグ判定

// ======================= 構造体・状態 =======================
struct SensorValues
{
    long left;
    long center;
    long right;
};

struct PIDState
{
    double p = 0.0; // 比例項（誤差）
    double i = 0.0; // 積分項
    double d = 0.0; // 微分項
    double prevError = 0.0;
};

PIDState pid;

// 積分リセットフラグ（calcError でセット → updatePID で実際にリセット）
bool resetIntegral = false;

// ベース速度
double basePower = 255.0;
double basePowerFiltered = 255.0;

// ラインが直前にどちら側にあったか（-1: 左, +1: 右）
int lastLineSide = 0;

// ジグザグ判定用
int prevSignP = 0;
int zigzagFlipCount = 0;
unsigned long zigzagWindowStart = 0;

// ======================= ユーティリティ =======================
inline bool isLineLost(const SensorValues &s)
{
    return (s.left < SENSOR_THRESHOLD &&
            s.center < SENSOR_THRESHOLD &&
            s.right < SENSOR_THRESHOLD);
}

// ======================= センサ =======================
SensorValues readSensors()
{
    SensorValues s;
    s.left = analogRead(SENSOR_L_PIN);
    s.center = analogRead(SENSOR_C_PIN);
    s.right = analogRead(SENSOR_R_PIN);
    return s;
}

void logSensors(const SensorValues &s)
{
    static unsigned long lastPrintTime = 0;
    unsigned long now = millis();

    if (now - lastPrintTime > 100)
    {
        Serial.print("L:");
        Serial.print(s.left);
        Serial.print(" C:");
        Serial.print(s.center);
        Serial.print(" R:");
        Serial.println(s.right);

        // デバッグしたければ以下を有効化
        // Serial.print(" P:"); Serial.print(pid.p);
        // Serial.print(" I:"); Serial.print(pid.i);
        // Serial.print(" D:"); Serial.print(pid.d);
        // Serial.print(" base:"); Serial.println(basePower);

        lastPrintTime = now;
    }
}

// ======================= 誤差計算 =======================
// ライン位置から誤差を計算し、lastLineSide も更新する
double calcError(const SensorValues &s)
{
    // 積分リセット条件：
    // 1) 中央センサが黒（ライン上）
    // 2) 左右どちらか片方だけ閾値を超えている（大きな偏り/交差など）
    resetIntegral =
        (s.center > SENSOR_THRESHOLD) ||
        ((s.left < SENSOR_THRESHOLD) != (s.right < SENSOR_THRESHOLD));

    // 全白（ラインロスト）のときは、最後にラインがあった側に戻る
    if (isLineLost(s))
    {
        // lastLineSide が -1:左 +1:右 なので、符号を反転して「そちら側へ回頭」
        return -lastLineSide * LINE_LOST_ERROR;
    }

    // 白の時低くなる / 黒の時高くなる前提
    const double leftLevel = 1024.0 - static_cast<double>(s.left);
    const double rightLevel = 1024.0 - static_cast<double>(s.right);
    const double centerFactor = 2.0 - static_cast<double>(s.center) / 1024.0;

    double val = -(leftLevel - rightLevel) * centerFactor / 1024.0;

    // ノイズで左右判定が頻繁に反転しないようにデッドバンド付きで更新
    if (val < 0)
    {
        lastLineSide = -1;
    }
    else if (val >)
    {
        lastLineSide = 1;
    }
    // ほぼ 0 のときは lastLineSide は変えない（前の情報を維持）

    return val;
}

// ======================= PID 更新 =======================
void updatePID(PIDState &pid, double error)
{
    pid.p = error;

    if (resetIntegral)
    {
        pid.i = 0.0;
        pid.prevError = error; // D項のキック抑制
        resetIntegral = false;
    }
    else
    {
        pid.i += error;
        // アンチワインドアップ
        if (pid.i > I_MAX)
            pid.i = I_MAX;
        if (pid.i < -I_MAX)
            pid.i = -I_MAX;
    }

    pid.d = error - pid.prevError;
    pid.prevError = error;
}

// ======================= 速度プロファイル =======================
void updateSpeedProfile(const SensorValues &s, const PIDState &pid)
{
    unsigned long now = millis();

    // ジグザグ判定用のウィンドウ管理
    if (now - zigzagWindowStart > ZIGZAG_WINDOW)
    {
        zigzagWindowStart = now;
        zigzagFlipCount = 0;
    }

    // p の符号の変化回数で「グネグネ」を検出
    int signP = 0;
    if (pid.p > P_STRAIGHT_THR)
        signP = 1;
    if (pid.p < -P_STRAIGHT_THR)
        signP = -1;

    if (signP != 0 && prevSignP != 0 && signP != prevSignP)
    {
        zigzagFlipCount++;
    }
    prevSignP = signP;

    // 中央センサのヒステリシス判定
    static bool centerOn = false;
    constexpr long C_ON_THR = SENSOR_THRESHOLD + 40;
    constexpr long C_OFF_THR = SENSOR_THRESHOLD - 40;

    if (centerOn)
    {
        if (s.center < C_OFF_THR)
            centerOn = false;
    }
    else
    {
        if (s.center > C_ON_THR)
            centerOn = true;
    }

    // p と |Δp| の EMA を使った直線判定
    static double pEMA = 0.0;
    static double dAbsEMA = 0.0;
    static double prevPDet = 0.0;

    pEMA = 0.9 * pEMA + 0.1 * pid.p;
    double dp = pid.p - prevPDet;
    prevPDet = pid.p;
    dAbsEMA = 0.9 * dAbsEMA + 0.1 * fabs(dp);

    bool lineLost = isLineLost(s);
    bool isZigzag = (zigzagFlipCount >= 3);

    constexpr double P_THR = 50.0;
    constexpr double D_THR = 20.0;
    static unsigned long straightStart = 0;

    bool straightNow = centerOn && fabs(pEMA) < P_THR && dAbsEMA < D_THR;
    if (straightNow)
    {
        if (straightStart == 0)
            straightStart = now;
    }
    else
    {
        straightStart = 0;
    }
    bool isStraight = straightNow && (now - straightStart >= 50);

    int targetBase;
    if (lineLost)
        targetBase = BASE_ZIGZAG; // 迷子時は安全に低速
    else if (isZigzag)
        targetBase = BASE_ZIGZAG;
    else if (isStraight)
        targetBase = BASE_STRAIGHT;
    else
        targetBase = BASE_CURVE;

    // スムージング（急激な速度変化を抑制）
    basePowerFiltered = 0.8 * basePowerFiltered + 0.2 * targetBase;
    basePower = basePowerFiltered;
}

// ======================= モータ出力 =======================
long calcPowerL(const PIDState &pid)
{
    double u = KP * pid.p + KI * pid.i + KD * pid.d;
    return static_cast<long>(basePower - u);
}

long calcPowerR(const PIDState &pid)
{
    double u = KP * pid.p + KI * pid.i + KD * pid.d;
    return static_cast<long>(basePower + u);
}

void setMotor(long speedL, long speedR)
{
    // 進行方向（常に前進） ※必要なら後退制御を追加
    digitalWrite(MOTOR_L_DIR1, LOW);
    digitalWrite(MOTOR_L_DIR2, HIGH);
    digitalWrite(MOTOR_R_DIR1, LOW);
    digitalWrite(MOTOR_R_DIR2, HIGH);

    analogWrite(MOTOR_L_PWM, constrain(speedL, 0, 255));
    analogWrite(MOTOR_R_PWM, constrain(speedR, 0, 255));
}

// ======================= Arduino 標準関数 =======================
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

void loop()
{
    // 1. センサ読み取り & ログ
    SensorValues sensors = readSensors();
    logSensors(sensors);

    // 2. 誤差計算 → PID 更新
    double error = calcError(sensors);
    updatePID(pid, error);

    // 3. 速度プロファイル更新
    updateSpeedProfile(sensors, pid);

    // 4. モータ出力
    long powerL = calcPowerL(pid);
    long powerR = calcPowerR(pid);
    setMotor(powerL, powerR);
}
