```mermaid
flowchart TD
    A([電源ON / リセット]) --> B[setup():<br/>モータ用ピンを OUTPUT に設定]

    B --> C[[loop() を無限ループ]]

    %% --- メインループ ---
    C --> D[readSensor():<br/>左・中央・右フォトリフレクタを analogRead]

    D --> E[calcError() 呼び出し]

    %% --- calcError() 内部 ---
    subgraph CalcError[calcError()]
        E --> E1[isLBlack, isCBlack, isRBlack を計算<br/>(各センサ値 > SENSOR_THRESHOLD)]
        E1 --> E2[resetIntegral ← isCBlack]

        E2 --> F{sum = isLBlack<br/>+ isCBlack<br/>+ isRBlack}

        F -->|sum == 0| G[error ← wasRight * 3<br/>(ラインロスト時は<br/>直前の向きに大きく曲げる)]

        F -->|sum > 0| H[error ← 2 / sum * (isLBlack - isRBlack)]

        H --> I{error の符号}
        I -->|> 0| J[wasRight ← 1<br/>(右に寄っていると記録)]
        I -->|< 0| K[wasRight ← -1<br/>(左に寄っていると記録)]
        I -->|= 0| L[wasRight は変更しない]

        G --> Eout[calcError() の戻り値 error]
        J --> Eout
        K --> Eout
        L --> Eout
    end

    Eout --> M[updatePID(error) 呼び出し]

    %% --- updatePID() 内部 ---
    subgraph UpdatePID[updatePID(error)]
        M --> M1[p ← error,<br/>i ← i + error,<br/>d ← error - prevError,<br/>prevError ← error]
        M1 --> N{resetIntegral ?}
        N -->|true| O[i ← 0,<br/>resetIntegral ← false]
        N -->|false| P[そのまま継続]
    end

    O --> Q[calcPowerL(), calcPowerR() 呼び出し]
    P --> Q

    %% --- 出力計算 ---
    subgraph CalcPower[calcPowerL / calcPowerR]
        Q --> Q1[calced ← Kp*p + i/KiInverse + Kd*d]
        Q1 --> Q2[左: powerL ← basePower - calced<br/>右: powerR ← basePower + calced]
        Q2 --> Q3[0〜255 にクリップ]
    end

    Q3 --> R[setMotor(speedL, speedR)]

    %% --- モータ出力 ---
    R --> R1[左右モータの回転方向を前進に設定<br/>(DIR1, DIR2 の組合せを固定)]
    R1 --> R2[analogWrite で<br/>左: speedL, 右: speedR を出力]

    R2 --> C

```