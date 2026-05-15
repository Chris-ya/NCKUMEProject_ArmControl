#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include "hardware/pio.h"
#include "hardware/clocks.h"

// --- 腳位定義 ---
// 手臂與夾爪
#define SERVO1_Arm 11
#define SERVO2_Arm 12
#define SERVO_JAW 13
#define SERVO1_JAW 9
#define SERVO2_JAW 8

// 基座馬達 (Base Motor)
#define ENCA_PIN 6
#define ENCB_PIN 7
#define PWMA_PIN 2
#define AIN1_PIN 3
#define AIN2_PIN 4

// 詹森機構步足馬達 (BTS7960 x 2)
#define WALKER_L_EN1 18 // Left Motor Enables (可將 L_EN 與 R_EN 接在一起)
#define WALKER_L_EN2 19
#define WALKER_M1_LPWM 20
#define WALKER_M1_RPWM 21

#define WALKER_R_EN1 22 // Right Motor Enables
#define WALKER_R_EN2 26
#define WALKER_M2_LPWM 27
#define WALKER_M2_RPWM 28

// --- 全域變數宣告 ---
extern PIO armpio;
extern PIO Jawpio;
extern PIO Feetpio;

extern float targetX;
extern float targetY;
extern float targetZ;
extern bool eStop;
extern bool clawopen;
extern float Gain;

// --- 函式宣告 ---
void setupPioServo(PIO pio, uint sm, uint pin);
void setPioServoAngle(PIO pio, uint sm, float angle, float max_angle);
void setupPioMotorPWM(PIO pio, uint sm, uint pin);
void setPioMotorPower(PIO pio, uint sm, int pwmValue_0_to_255);

void setupBaseMotor();
void runBasePID(float targetAngleDegree);

void setupWalkerMotors();
void setWalkerSpeed(int leftSpeed, int rightSpeed);

void setupBluetooth();
void checkSerial();
void calculateAngles(float x, float y, float z, float &angleBase, float &angle1, float &angle2);

#endif