// #include <Arduino.h>

// const int SENSOR_L = A2;
// const int SENSOR_C = A1;
// const int SENSOR_R = A0;
// const int MOTOR_L_PWM = 10;
// const int MOTOR_L_DIR1 = 2;
// const int MOTOR_L_DIR2 = 3;
// const int MOTOR_R_PWM = 11;
// const int MOTOR_R_DIR1 = 4;
// const int MOTOR_R_DIR2 = 5;
// const unsigned long INTERVAL = 5; // 5ms

// void setup()
// {
//     // Serial.begin(9600);
//     pinMode(MOTOR_L_PWM, OUTPUT);
//     pinMode(MOTOR_L_DIR1, OUTPUT);
//     pinMode(MOTOR_L_DIR2, OUTPUT);
//     pinMode(MOTOR_R_PWM, OUTPUT);
//     pinMode(MOTOR_R_DIR1, OUTPUT);
//     pinMode(MOTOR_R_DIR2, OUTPUT);
// }

// int State = 0; // 0:直進,1:カーブ,2:ジグザグ

// const int BASE_STRAIGHT = 255; // 直線
// const int BASE_CURVE = 180;    // カーブ
// const int BASE_ZIGZAG = 100;   // グネグネ

// int basePower = BASE_STRAIGHT; // 直線スタート

// // pの絶対値の指数移動平均（モード判定用）
// double pAbsEMA = 0.0;
// const double PEMA_ALPHA = 0.1; // 0.0〜1.0（大きいほど反応が速い）

// const long SENSOR_THRESHOLD = 450;

// void readSensor()
// {
//     int rawL = analogRead(SENSOR_L);
//     int rawC = analogRead(SENSOR_C);
//     int rawR = analogRead(SENSOR_R);

//     // printSensorLog(rawL, rawC, rawR);

//     updatePID(calcError(rawL, rawC, rawR));
// }

// double p = 0; // マイナスなら左,プラスなら右に機体の頭が寄っている [-2,2]
// double i = 0; // [-I_MAX,I_MAX] 1秒間で200回転するため200*2で1秒間で最大400程度増減する想定
// double d = 0; // 大体[-200,200]

// double prevError = 0;

// double absIntegralEMA = 0; // 積分値の絶対値の指数移動平均

// bool resetIntegral = false;

// const double I_MAX = 1000.0;

// void updatePID(double error)
// {
//     p = error;
//     pAbsEMA = (1.0 - PEMA_ALPHA) * pAbsEMA + PEMA_ALPHA * fabs(p);

//     // ここで「加算前」に積分リセット
//     if (resetIntegral)
//     {
//         i = 0;
//         prevError = error; // D項のキック抑制
//         resetIntegral = false;
//     }
//     else
//     {
//         i += error * (INTERVAL / 1000.0);
//         // アンチワインドアップ（必要に応じて調整）
//         if (i > I_MAX)
//             i = I_MAX;
//         if (i < -I_MAX)
//             i = -I_MAX;
//     }

//     d = (error - prevError) * (1000.0 / INTERVAL);
//     prevError = error;
// }

// int isLeftNow = 0;
// bool lineLost = false;
// // // 全て白の時にエラーが0になってしまう
// double calcError(double rawL, double rawC, double rawR)
// {
//     // rawL:白,rawC:黒,rawR:白 を目指す

//     // リセット条件の判定はここで行い、実際のリセットは updatePID に委ねる
//     // rawL:白,rawC:黒,rawR:白 の場合にのみリセット
//     resetIntegral =
//         (rawC > SENSOR_THRESHOLD) ||
//         ((rawL < SENSOR_THRESHOLD) != (rawR < SENSOR_THRESHOLD));
//     // resetIntegral =
//     //     (rawC > SENSOR_THRESHOLD) &&
//     //     (rawL < SENSOR_THRESHOLD) &&
//     //     (rawR < SENSOR_THRESHOLD);

//     // const int SENSOR_LOST_THRESHOLD = 700;
//     const int SENSOR_LOST_THRESHOLD = SENSOR_THRESHOLD;
//     lineLost =
//         (rawL < SENSOR_LOST_THRESHOLD) &&
//         (rawC < SENSOR_LOST_THRESHOLD) &&
//         (rawR < SENSOR_LOST_THRESHOLD);
//     // 全白（ラインロスト）時のフェイルセーフ
//     if (lineLost)
//     {
//         return isLeftNow * 2; // 前回の向きに進む
//     }

//     // 白の時低くなる / 黒の時高くなる前提
//     double val = -((1024 - rawL) - (1024 - rawR)) * (2.0 - rawC / 1024.0) / 1024.0;
//     if (val < 0)
//     {
//         isLeftNow = -1;
//     }
//     else
//     {
//         isLeftNow = 1;
//     }

//     return val;
// }

// void setMotor(long speedL, long speedR)
// {
//     digitalWrite(MOTOR_L_DIR1, LOW);
//     digitalWrite(MOTOR_L_DIR2, HIGH);

//     digitalWrite(MOTOR_R_DIR1, LOW);
//     digitalWrite(MOTOR_R_DIR2, HIGH);

//     analogWrite(MOTOR_L_PWM, constrain(speedL, 0, 255));
//     analogWrite(MOTOR_R_PWM, constrain(speedR, 0, 255));
// }

// long calcPowerL()
// {
//     const double Kp = 30;
//     double Ki = 0.01;
//     const double Kd = -0.5;
//     // const double Kd = 0;

//     if (!lineLost)
//     {
//         Ki = 0;
//     } // ラインを見失っていないときは積分を使わない
//     double calced = Kp * p + Ki * i + Kd * d;

//     return basePower - calced;
// }

// long calcPowerR()
// {
//     const double Kp = 30;
//     double Ki = 0.01;
//     const double Kd = -0.5;
//     // const double Kd = 0;

//     if (!lineLost)
//     {
//         Ki = 0;
//     } // ラインを見失っていないときは積分を使わない
//     double calced = Kp * p + Ki * i + Kd * d;

//     return basePower + calced;
// }

// void updateState()
// {

//     const double PEMA_THR_STRAIGHT = 0.3;
//     const double PEMA_THR_CURVE = 1.3;
//     if (pAbsEMA < PEMA_THR_STRAIGHT)
//     {
//         State = 0; // 直進
//         basePower = (basePower + BASE_STRAIGHT) / 2;
//     }
//     else if (pAbsEMA < PEMA_THR_CURVE)
//     {
//         State = 1; // カーブ
//         basePower = (basePower + BASE_CURVE) / 2;
//     }
//     else
//     {
//         State = 2; // ジグザグ
//         basePower = (basePower + BASE_ZIGZAG) / 2;
//     }
// }

// unsigned long lastExecuteTime = 0;

// void loop()
// {
//     unsigned long currentTime = millis();

//     if (currentTime - lastExecuteTime >= INTERVAL)
//     {
//         lastExecuteTime = currentTime;

//         readSensor();
//         updateState();
//         setMotor(calcPowerL(), calcPowerR());
//     }
// }

// // void printSensorLog(long rawL, long rawC, long rawR)
// // {
// //     static unsigned long lastPrintTime = 0;
// //     unsigned long currentTime = millis();

// //     if (currentTime - lastPrintTime > 100)
// //     {

// //         Serial.print("L:");
// //         Serial.print(rawL);
// //         Serial.print(" ");
// //         Serial.print("C:");
// //         Serial.print(rawC);
// //         Serial.print(" ");
// //         Serial.print("R:");
// //         Serial.println(rawR);

// //         lastPrintTime = currentTime;
// //     }
// // }

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

// 追加: 速度プロファイル関連
double basePower = 100;         // 現在のベース速度
double basePowerFiltered = 100; // スムージング後のベース速度

const int BASE_STRAIGHT = 255; // 直線
const int BASE_CURVE = 160;    // カーブ
const int BASE_ZIGZAG = 70;    // グネグネ

const double P_STRAIGHT_THR = 0.15; // 直線判定のpしきい値
const double D_STRAIGHT_THR = 0.10; // 直線判定のdしきい値

const unsigned long ZIGZAG_WINDOW = 300; // ms 内の反転回数でジグザグ判定
int prevSignP = 0;
int zigzagFlipCount = 0;
unsigned long zigzagWindowStart = 0;

long calcPowerL()
{
    const double Kp = 30;
    const double Ki = 0.001;
    const double Kd = 10;
    double calced = Kp * p + Ki * i + Kd * d;

    return basePower - calced;
}

long calcPowerR()
{
    const double Kp = 30;
    const double Ki = 0.001;
    const double Kd = 10;
    double calced = Kp * p + Ki * i + Kd * d;

    return basePower + calced;
}

void updateSpeedProfile(long rawL, long rawC, long rawR)
{
    // ウィンドウ管理（一定時間ごとに反転カウントをリセット）
    unsigned long now = millis();
    if (now - zigzagWindowStart > ZIGZAG_WINDOW)
    {
        zigzagWindowStart = now;
        zigzagFlipCount = 0;
    }

    // p の符号反転検出（デッドバンド付き）
    int signP = 0;
    if (p > P_STRAIGHT_THR)
        signP = 1;
    if (p < -P_STRAIGHT_THR)
        signP = -1;
    if (signP != 0 && prevSignP != 0 && signP != prevSignP)
    {
        zigzagFlipCount++;
    }
    prevSignP = signP;

    // センター判定にヒステリシスを入れて安定化
    static bool centerOn = false;
    const long C_ON_THR = SENSOR_THRESHOLD + 40;  // 上がる時の閾値
    const long C_OFF_THR = SENSOR_THRESHOLD - 40; // 下がる時の閾値
    if (centerOn)
    {
        if (rawC < C_OFF_THR)
            centerOn = false;
    }
    else
    {
        if (rawC > C_ON_THR)
            centerOn = true;
    }

    // 直線判定用フィルタ（pと|Δp|のEMA）
    static double pEMA = 0.0;
    static double dAbsEMA = 0.0;
    static double prevPDet = 0.0;
    pEMA = 0.9 * pEMA + 0.1 * p;
    double dp = p - prevPDet; // サンプル間差分（未正規化）
    prevPDet = p;
    dAbsEMA = 0.9 * dAbsEMA + 0.1 * fabs(dp);

    bool lineLost = (rawL < SENSOR_THRESHOLD && rawC < SENSOR_THRESHOLD && rawR < SENSOR_THRESHOLD);
    bool isZigzag = (zigzagFlipCount >= 3);

    // EMAに合わせた少し緩めの閾値 + 50msデバウンス
    const double P_THR = 50;
    const double D_THR = 20;
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
        targetBase = BASE_ZIGZAG; // 迷子時は安全に
    else if (isZigzag)
        targetBase = BASE_ZIGZAG;
    else if (isStraight)
        targetBase = BASE_STRAIGHT;
    else
        targetBase = BASE_CURVE;

    // スムージング（急激な速度変化を抑制）
    basePowerFiltered = 0.8 * basePowerFiltered + 0.2 * targetBase;
    basePower = basePowerFiltered;

    // デバッグしたい場合は有効化
    // Serial.print(" p:"); Serial.print(p);
    // Serial.print(" pEMA:"); Serial.print(pEMA);
    // Serial.print(" dAbsEMA:"); Serial.print(dAbsEMA);
    // Serial.print(" C:"); Serial.print(rawC);
    // Serial.print(" ctrOn:"); Serial.print(centerOn);
    // Serial.print(" straight:"); Serial.println(isStraight);
}

void readSensor()
{
    long rawL = analogRead(SENSOR_L);
    long rawC = analogRead(SENSOR_C);
    long rawR = analogRead(SENSOR_R);

    printSensorLog(rawL, rawC, rawR);

    updatePID(calcError(rawL, rawC, rawR));

    // 追加: PID更新後に速度プロファイル更新
    updateSpeedProfile(rawL, rawC, rawR);
}

void updatePID(double error)
{
    p = error;

    // ここで「加算前」に積分リセット
    if (resetIntegral)
    {
        i = 0;
        resetIntegral = false;
    }
    else
    {
        i += error;
        // アンチワインドアップ（必要に応じて調整）
        // const double I_MAX = 1000.0;
        // if (i > I_MAX)
        //     i = I_MAX;
        // if (i < -I_MAX)
        //     i = -I_MAX;
    }

    d = (error - prevError);
    prevError = error;
}

double isLeftNow = -1;

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
    const double LINE_LOST_THRESHOLD = 100;

    if (rawL < LINE_LOST_THRESHOLD && rawC < LINE_LOST_THRESHOLD && rawR < LINE_LOST_THRESHOLD)
    {
        return isLeftNow * 3;
    }

    // 白の時低くなる / 黒の時高くなる前提
    double val = -((1024 - rawL) - (1024 - rawR)) * (2.0 - rawC / 1024.0) / 1024.0;
    if (rawL < SENSOR_THRESHOLD && rawR > SENSOR_THRESHOLD)
    {
        isLeftNow = -1;
    }
    else if (rawR < SENSOR_THRESHOLD && rawL > SENSOR_THRESHOLD)
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