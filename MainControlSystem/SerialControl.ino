// SerialControl.ino
#include <Arduino.h>

extern float targetX;
extern float targetY;
extern float targetZ;
extern bool eStop; 

extern void calculateAngles(float x, float y, float z, float &angleBase, float &angle1, float &angle2);

void checkSerial() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        input.toUpperCase(); 

        if (input == "STOP" || input == "E") {
            eStop = true;
            Serial.println("!!! EMERGENCY STOP ACTIVATED !!! Motors Locked.");
            return; // 直接中斷解析
        }

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

            float aBase, a1, a2;
            calculateAngles(targetX, targetY, targetZ, aBase, a1, a2);

            float out1 = 130.0 - a1;
            float out2 = a2 - 40.0;

            if (isnan(aBase) || isnan(a1) || isnan(a2)) {
                Serial.println(">>> [Warning] Target out of range! Calculations result in NaN.");
            } else {
                Serial.printf(">>> IK Arm Angles  -> Base:%.2f, A1:%.2f, A2:%.2f\n", aBase, a1, a2);
                Serial.printf(">>> Motor PWM Angle -> Base:%.2f, Servo1:%.2f, Servo2:%.2f\n", aBase, out1, out2);
            }
            // ---------------------------------------
        }
    }
}
