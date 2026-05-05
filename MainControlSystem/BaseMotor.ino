// BaseMotor.ino
#include <Arduino.h>

// --- 腳位定義 ---
#define ENCA_PIN 6
#define ENCB_PIN 7
#define PWMA_PIN 2
#define AIN1_PIN 3
#define AIN2_PIN 4

const float PPR = 11;
const float Internal_Gear_Ratio = 46.8;
const float Outer_Gear_Ratio = 3;
const float TICKS_PER_REV = PRP * Internal_Gear_Ratio * Outer_Gear_Ratio; 

// --- PID 參數 ---
float Kp = 2.5;  // 比例常數 (過小推不動，過大會震盪)
float Ki = 0.01; // 積分常數 (消除靜態誤差)
float Kd = 0.5;  // 微分常數 (抑制震盪)

// --- 全域變數 ---
volatile long currentTicks = 0; // 記錄當下編碼器位置
float errorSum = 0;
float lastError = 0;

// 編碼器中斷服務常式 (ISR)
void readEncoder() {
    // 判斷旋轉方向
    int b = digitalRead(ENCB_PIN);
    if (b > 0) {
        currentTicks++;
    } else {
        currentTicks--;
    }
}

void setupBaseMotor() {
    pinMode(ENCA_PIN, INPUT_PULLUP);
    pinMode(ENCB_PIN, INPUT_PULLUP);
    pinMode(PWMA_PIN, OUTPUT);
    pinMode(AIN1_PIN, OUTPUT);
    pinMode(AIN2_PIN, OUTPUT);

    // 設定中斷，當 ENCA 發生上升沿時觸發
    attachInterrupt(digitalPinToInterrupt(ENCA_PIN), readEncoder, RISING);
}

// 驅動 TB6612 的底層函式
void setMotorPower(int power) {
    if (power > 0) {
        digitalWrite(AIN1_PIN, HIGH);
        digitalWrite(AIN2_PIN, LOW);
    } else if (power < 0) {
        digitalWrite(AIN1_PIN, LOW);
        digitalWrite(AIN2_PIN, HIGH);
    } else {
        digitalWrite(AIN1_PIN, LOW);
        digitalWrite(AIN2_PIN, LOW);
    }
    
    // 限制 PWM 範圍在 0~255 (預設 Arduino analogWrite 範圍)
    int pwmValue = abs(power);
    if (pwmValue > 255) pwmValue = 255;
    analogWrite(PWMA_PIN, pwmValue);
}

// 執行 PID 控制
void runBasePID(float targetAngleDegree) {
    // 將 IK 算出的目標角度 (0~180 或 -90~90) 轉換為目標脈衝數
    // 假設 0 度對應 0 ticks
    long targetTicks = (targetAngleDegree / 360.0) * TICKS_PER_REV;
    
    // 1. 計算誤差 (Error)
    long error = targetTicks - currentTicks;
    
    // 2. 計算積分項與微分項
    errorSum += error;
    float dError = error - lastError;
    
    // 積分限幅 (避免長時間卡住導致積分飽和，暴衝)
    if(errorSum > 1000) errorSum = 1000;
    if(errorSum < -1000) errorSum = -1000;

    // 3. PID 輸出計算
    float output = (Kp * error) + (Ki * errorSum) + (Kd * dError);
    
    // 4. 輸出動力到 TB6612
    setMotorPower((int)output);
    
    // 5. 更新上次誤差
    lastError = error;
}