#include "Config.h"

const float PPR = 11;
const float Internal_Gear_Ratio = 46.8;
const float Outer_Gear_Ratio = 3;
const float TICKS_PER_REV = PPR * Internal_Gear_Ratio * Outer_Gear_Ratio;

float Kp = 2.5;  
float Ki = 0.01; 
float Kd = 0.5;  

volatile long currentTicks = 0;
float errorSum = 0;
float lastError = 0;

void readEncoder() {
    int b = digitalRead(ENCB_PIN);
    if (b > 0) currentTicks++;
    else currentTicks--;
}

void setupBaseMotor() {
    pinMode(ENCA_PIN, INPUT_PULLUP);
    pinMode(ENCB_PIN, INPUT_PULLUP);
    pinMode(AIN1_PIN, OUTPUT);
    pinMode(AIN2_PIN, OUTPUT);
    
    // 使用 armpio 的 sm0 來控制 PWMA
    setupPioMotorPWM(armpio, 0, PWMA_PIN);
    attachInterrupt(digitalPinToInterrupt(ENCA_PIN), readEncoder, RISING);
}

void setMotorPower(int power) {
    if (power > 0) {
        digitalWrite(AIN1_PIN, HIGH); digitalWrite(AIN2_PIN, LOW);
    } else if (power < 0) {
        digitalWrite(AIN1_PIN, LOW); digitalWrite(AIN2_PIN, HIGH);
    } else {
        digitalWrite(AIN1_PIN, LOW); digitalWrite(AIN2_PIN, LOW);
    }
    
    int pwmValue = abs(power);
    if (pwmValue > 255) pwmValue = 255;
    setPioMotorPower(armpio, 0, pwmValue);
}

void runBasePID(float targetAngleDegree) {
    long targetTicks = (targetAngleDegree / 360.0) * TICKS_PER_REV;
    long error = targetTicks - currentTicks;
    
    errorSum += error;
    float dError = error - lastError;
    if(errorSum > 1000) errorSum = 1000;
    if(errorSum < -1000) errorSum = -1000;
    float output = (Kp * error) + (Ki * errorSum) + (Kd * dError);
    
    setMotorPower((int)output);
    lastError = error;
}