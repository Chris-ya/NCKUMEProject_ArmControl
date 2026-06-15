#include "Config.h"

const float PPR = 11.0;
const float Internal_Gear_Ratio = 46.8;
const float Outer_Gear_Ratio = 3.0;
const float TICKS_PER_REV = PPR * Internal_Gear_Ratio * Outer_Gear_Ratio; 

// PID 參數：可以根據上機狀況微調
float Kp = 20.0;
float Ki = 0.01; 
float Kd = 0.5;  

float errorSum = 0;
float lastError = 0;

volatile long baseTicks = 0;

void baseEncoderISR() {
    if (digitalRead(ENCB_PIN) == LOW) {
        baseTicks++;
    } else {
        baseTicks--;
    }
}

void setupBaseMotor() {
    pinMode(AIN1_PIN, OUTPUT);
    pinMode(AIN2_PIN, OUTPUT);
    
    pinMode(STBY_PIN, OUTPUT);
    digitalWrite(STBY_PIN, HIGH);
    
    pinMode(PWMA_PIN, OUTPUT);
    
    pinMode(ENCA_PIN, INPUT_PULLUP);
    pinMode(ENCB_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCA_PIN), baseEncoderISR, RISING);
}

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
    
    int pwmValue = abs(power);
    if (pwmValue > 255) pwmValue = 255;
    
    analogWrite(PWMA_PIN, pwmValue);
}

void runBasePID(float targetAngleDegree) {
    static unsigned long lastTime = millis();
    static unsigned long lastLogTime = 0;
    unsigned long currentTime = millis();
    
    if (currentTime - lastTime < 10) return; 
    
    float dt = ((float)(currentTime - lastTime)) / 1000.0;
    lastTime = currentTime; 

    noInterrupts();
    long safeTicks = baseTicks;
    interrupts();

    float currentAngle = ((float)safeTicks / TICKS_PER_REV) * 360.0;
    float errorDegree = targetAngleDegree - currentAngle;

    while (errorDegree > 180.0)  errorDegree -= 360.0;
    while (errorDegree < -180.0) errorDegree += 360.0;

    if (abs(errorDegree) < 0.8) {
        setMotorPower(0);
        errorSum = 0;
        lastError = errorDegree;
        
        if (millis() - lastLogTime > 100) {
            // Serial.println("Status: Target Reached! Stopped.");
            lastLogTime = millis();
        }
        return;
    }
    
    // 5. 增益排程：接近目標時調降 Kp 避免震盪打擺子
    float currentKp = Kp;
    if (abs(errorDegree) < 3.0) {
        currentKp = 10.0; // 可以視情況調大調小
    }

    // 6. 積分與 Anti-Windup 限幅
    errorSum += (errorDegree * dt);
    if(errorSum > 100.0) errorSum = 100.0;
    if(errorSum < -100.0) errorSum = -100.0;
    
    // 7. 微分與 PID 出力計算
    float dError = (errorDegree - lastError) / dt;
    float output = (currentKp * errorDegree) + (Ki * errorSum) + (Kd * dError);
    
    // 8. 嚴格輸出限幅：絕對不讓超大數值灌進 setMotorPower
    if (output > 255.0) output = 255.0;
    if (output < -255.0) output = -255.0;

    // 9. 執行馬達出力
    setMotorPower((int)output); 
    lastError = errorDegree; 

    /*// ==========================================
    // 10. Serial 監控：每 100ms (0.1秒) 印出一次
    // ==========================================
    if (millis() - lastLogTime > 100) {
        Serial.print("Target: "); Serial.print(targetAngleDegree);
        Serial.print(" | Curr: "); Serial.print(currentAngle);
        Serial.print(" | Error: "); Serial.print(errorDegree);
        Serial.print(" | PWM_Out: "); Serial.println((int)output);
        lastLogTime = millis();
    }*/
}