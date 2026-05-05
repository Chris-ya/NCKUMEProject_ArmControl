// SerialControl.ino
#include <Arduino.h>

extern float targetX;
extern float targetY;
extern float targetZ;
extern bool eStop; 

// 引入外部定義的角度計算函式
extern void calculateAngles(float x, float y, float z, float &angleBase, float &angle1, float &angle2);

void checkSerial() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        input.toUpperCase(); 

        // 1. 緊急停止指令檢查
        if (input == "STOP" || input == "E") {
            eStop = true;
            Serial.println("!!! EMERGENCY STOP ACTIVATED !!! Motors Locked.");
            return; // 直接中斷解析
        }

        // 2. 解除緊急停止指令
        if (input == "RESET" || input == "R") {
            eStop = false;
            Serial.println("System Reset. Resuming normal operation.");
            return;
        }

        int xIdx = input.indexOf('X');
        int yIdx = input.indexOf('Y');
        int zIdx = input.indexOf('Z');

        if (xIdx != -1 && yIdx != -1 && zIdx != -1 && !eStop) {
            targetX = input.substring(xIdx + 1, yIdx).toFloat();
            targetY = input.substring(yIdx + 1, zIdx).toFloat(); 
            targetZ = input.substring(zIdx + 1).toFloat();

            Serial.printf("Serial Update -> X:%.2f Y:%.2f Z:%.2f\n", targetX, targetY, targetZ);

            // --- 新增：輸出計算出的角度與馬達輸出角度 ---
            float aBase, a1, a2;
            calculateAngles(targetX, targetY, targetZ, aBase, a1, a2);

            // 依照 Main.ino 的馬達補償邏輯計算實際輸出角度
            // out1 = 130 - a1, out2 = a2 - 40
            float out1 = 130.0 - a1;
            float out2 = a2 - 40.0;

            // 檢查是否超出物理範圍 (NaN)
            if (isnan(aBase) || isnan(a1) || isnan(a2)) {
                Serial.println(">>> [Warning] Target out of range! Calculations result in NaN.");
            } else {
                // 輸出 IK 計算出的幾何角度
                Serial.printf(">>> IK Arm Angles  -> Base:%.2f, A1:%.2f, A2:%.2f\n", aBase, a1, a2);
                // 輸出經過補償後發送給馬達的角度
                Serial.printf(">>> Motor PWM Angle -> Base:%.2f, Servo1:%.2f, Servo2:%.2f\n", aBase, out1, out2);
            }
            // ---------------------------------------
        }
    }
}