#include "Config.h"
#include <PicoEncoder.h> // 匯入 PIO 正交編碼器函式庫

const float PPR = 11;
const float Internal_Gear_Ratio = 46.8;
const float Outer_Gear_Ratio = 3;
const float TICKS_PER_REV = PPR * Internal_Gear_Ratio * Outer_Gear_Ratio; //

float Kp = 2.5;  
float Ki = 0.01; 
float Kd = 0.5;  //

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
    // 1. 觸發 PIO 更新最新狀態 (此動作執行極快，不會造成 delay)
    baseEncoder.update();
    
    // 2. 獲取硬體解析出來的真實步數 (取代原本以中斷計算的 currentTicks)
    long currentTicks = baseEncoder.step;

    long targetTicks = (targetAngleDegree / 360.0) * TICKS_PER_REV; //
    long error = targetTicks - currentTicks; //
    
    errorSum += error; //
    float dError = error - lastError; //
    
    if(errorSum > 1000) errorSum = 1000; //
    if(errorSum < -1000) errorSum = -1000; //
    
    float output = (Kp * error) + (Ki * errorSum) + (Kd * dError); //
    
    setMotorPower((int)output); //
    lastError = error; //
}