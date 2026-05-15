#include "Config.h"

void setupWalkerMotors() {
    // 啟用 BTS7960
    pinMode(WALKER_L_EN1, OUTPUT); pinMode(WALKER_L_EN2, OUTPUT);
    pinMode(WALKER_R_EN1, OUTPUT); pinMode(WALKER_R_EN2, OUTPUT);
    digitalWrite(WALKER_L_EN1, HIGH); digitalWrite(WALKER_L_EN2, HIGH);
    digitalWrite(WALKER_R_EN1, HIGH); digitalWrite(WALKER_R_EN2, HIGH);

    // 分配 pio2 的 4 個 SM 給兩顆馬達的正反轉 PWM
    setupPioMotorPWM(Feetpio, 0, WALKER_M1_LPWM);
    setupPioMotorPWM(Feetpio, 1, WALKER_M1_RPWM);
    setupPioMotorPWM(Feetpio, 2, WALKER_M2_LPWM);
    setupPioMotorPWM(Feetpio, 3, WALKER_M2_RPWM);
}

// 傳入數值為 -255 ~ 255
void setWalkerSpeed(int leftSpeed, int rightSpeed) {
    // 左馬達 (M1)
    if (leftSpeed > 0) {
        setPioMotorPower(Feetpio, 0, leftSpeed);
        setPioMotorPower(Feetpio, 1, 0);
    } else if (leftSpeed < 0) {
        setPioMotorPower(Feetpio, 0, 0);
        setPioMotorPower(Feetpio, 1, abs(leftSpeed));
    } else {
        setPioMotorPower(Feetpio, 0, 0);
        setPioMotorPower(Feetpio, 1, 0);
    }

    // 右馬達 (M2)
    if (rightSpeed > 0) {
        setPioMotorPower(Feetpio, 2, rightSpeed);
        setPioMotorPower(Feetpio, 3, 0);
    } else if (rightSpeed < 0) {
        setPioMotorPower(Feetpio, 2, 0);
        setPioMotorPower(Feetpio, 3, abs(rightSpeed));
    } else {
        setPioMotorPower(Feetpio, 2, 0);
        setPioMotorPower(Feetpio, 3, 0);
    }
}