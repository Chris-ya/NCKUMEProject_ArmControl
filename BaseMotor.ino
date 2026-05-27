#include "Config.h"
#include <PicoEncoder.h> // 匯入 PIO 正交編碼器函式庫

const float PPR = 11;
const float Internal_Gear_Ratio = 46.8;
const float Outer_Gear_Ratio = 3;
const float TICKS_PER_REV = PPR * Internal_Gear_Ratio * Outer_Gear_Ratio; //

float Kp = 30;  
float Ki = 0.0; 
float Kd = 1.2;  //

float errorSum = 0;
float lastError = 0;

// 宣告 PIO 編碼器物件
PicoEncoder baseEncoder;

void setupBaseMotor() {
    pinMode(AIN1_PIN, OUTPUT);
    pinMode(AIN2_PIN, OUTPUT); //
    
    // 使用 armpio 的 sm0 來控制 PWMA
    setupPioMotorPWM(armpio, 0, PWMA_PIN);
    
    // 初始化 PIO 編碼器。
    // 參數只需傳入較小的腳位 (ENCA_PIN = 6)，PIO 會自動將相鄰的腳位 (7) 視為 ENCB
    baseEncoder.begin(ENCA_PIN);
}

void setMotorPower(int power) {
    if (power > 0) {
        digitalWrite(AIN1_PIN, HIGH); digitalWrite(AIN2_PIN, LOW); //
    } else if (power < 0) {
        digitalWrite(AIN1_PIN, LOW); digitalWrite(AIN2_PIN, HIGH); //
    } else {
        digitalWrite(AIN1_PIN, LOW); digitalWrite(AIN2_PIN, LOW); //
    }
    
    int pwmValue = abs(power); //
    if (pwmValue > 255) pwmValue = 255; //
    setPioMotorPower(armpio, 0, pwmValue); //
}

void runBasePID(float targetAngleDegree) {
    
    static unsigned long lastTime = micros(); 
    unsigned long currentTime = micros();
    
    float dt = ((float)(currentTime - lastTime)) / 1000000.0;
    
    if (dt <= 0.0) {
        dt = 0.02; 
    }
    lastTime = currentTime; 

    // ---------------------------------------------------------
    // 更新編碼器與計算誤差
    // ---------------------------------------------------------
    baseEncoder.update();
    long currentTicks = baseEncoder.step;
    long targetTicks = (targetAngleDegree / 360.0) * TICKS_PER_REV;
    
    float error = (float)(targetTicks - currentTicks); 
    
    // ---------------------------------------------------------
    // 增益排程 (Gain Scheduling) - 靠近目標時降低 Kp
    // ---------------------------------------------------------
    float errorDegree = (error / TICKS_PER_REV) * 360.0;

    if (abs(errorDegree) < 0.3) {
        setMotorPower(0);     // 強制斷電，關閉馬達
        errorSum = 0;         // 🚨極度重要：清空積分，防止積分累積導致爆衝 (Windup)
        lastError = error;    // 同步狀態
        return;               // 直接跳出函式，不執行後續 PID 計算
    }
    float currentKp = Kp; 
    
    // 若絕對誤差小於 3 度，切換為較小的 Kp 
    if (abs(errorDegree) < 3.0) {
        currentKp = 2; 
    }

    // ---------------------------------------------------------
    // PID 計算
    // ---------------------------------------------------------
    // 【積分 I】誤差累積面積 = 誤差 * 時間
    errorSum += (error * dt);
    
    // 積分抗飽和 (Anti-windup)
    if(errorSum > 1000) errorSum = 1000;
    if(errorSum < -1000) errorSum = -1000;
    
    float dError = (error - lastError) / dt;

    float output = (currentKp * error) + (Ki * errorSum) + (Kd * dError);
    
    setMotorPower((int)output); 
    
    // 紀錄當次的誤差，供下一次微分使用
    lastError = error; 
}