#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// -------------------------------------------------------------------------
// 【區域一：基座馬達驅動組】
// -------------------------------------------------------------------------
#define PWMA_PIN     26
#define AIN1_PIN     22   
#define AIN2_PIN     2  
#define STBY_PIN     3  

// -------------------------------------------------------------------------
// 【區域二：遙控通訊組】
// -------------------------------------------------------------------------
#define TX1          0  
#define RX1          1  

// -------------------------------------------------------------------------
// 【區域三：基座編碼器組】
// -------------------------------------------------------------------------
#define ENCA_PIN     6  
#define ENCB_PIN     7  

// -------------------------------------------------------------------------
// 【區域四：夾爪與門伺服馬達組】
// -------------------------------------------------------------------------
#define SERVO_JAW    8  
#define SERVO1_JAW   9  
#define SERVO2_JAW  10  
#define SERVO_DOOR  27

// -------------------------------------------------------------------------
// 【區域五：手臂伺服馬達組】
// -------------------------------------------------------------------------
#define SERVO1_Arm  12  // 手臂關節 1
#define SERVO2_Arm  11  // 手臂關節 2



// -------------------------------------------------------------------------
// 【區域七：右步足馬達組 (M2)】
// -------------------------------------------------------------------------
#define WALKER_M2_LPWM 16
#define WALKER_M2_RPWM 17 
#define WALKER_R_EN 18

// -------------------------------------------------------------------------
// 【區域六：左步足馬達組 (M1)】
// -------------------------------------------------------------------------
#define WALKER_M1_LPWM 19 
#define WALKER_M1_RPWM 20
#define WALKER_L_EN 21 
// =========================================================================
// 全域變數宣告
// =========================================================================
extern volatile int MAX_PWM_LIMIT; // 把 255 改成你想要的最高限制 (0~255)
extern volatile int joyX_raw;
extern volatile int joyY_raw;
extern volatile int joyWalkX_raw;
extern volatile int joyWalkY_raw;
extern volatile int btnClaw_raw;
extern volatile int btnDoor_raw;

extern volatile float targetX;
extern volatile float targetY;
extern volatile float targetZ;
extern volatile bool eStop;
extern volatile bool clawopen;
extern volatile bool resetposition;
extern float Gain;

// =========================================================================
// 函式原型宣告
// =========================================================================
void setupBaseMotor();
void runBasePID(float targetAngleDegree);

void setupWalkerMotors();
void setWalkerSpeed(int leftSpeed, int rightSpeed);

void setupUART();
void checkUART();
void checkSerialMonitor();

void calculateAngles(float x, float z, float &angle1, float &angle2);

#endif