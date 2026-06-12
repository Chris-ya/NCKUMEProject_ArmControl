#include "Config.h"
#include <Servo.h> 

// 引用外部 CommUART.ino 內的全域變數
extern size_t bufferIndex;

Servo servoArm1;
Servo servoArm2;
Servo servoJaw;
Servo servoJaw1;
Servo servoJaw2;

volatile float targetX = 168.3;
volatile float targetY = 0;
volatile float targetZ = 69.7;
volatile bool eStop = false;
volatile bool clawopen = false; 
volatile int MAX_PWM_LIMIT = 155;
float Gain = 0.0;
const float MAX_VELOCITY = 1.2;      
const int CONTROL_LOOP_DELAY = 20; 
int walkForward = 0;
int walkTurn = 0;

unsigned long lastPacketTime = 0;

// 🛡️ 安全存檔點
float safeX = 168.3;
float safeY = 0.0;
float safeZ = 69.7;

// ==========================================
// 🚀 3. 手動宣告函式原型 (破解 Arduino IDE 自動產生的 Bug)
// ==========================================
void triggerServo(Servo& servoObj, float angle);

// 實作寫入函式 (保留最高精度的微秒控制)
void triggerServo(Servo& servoObj, float angle) {
    float safeAngle = constrain(angle, 0.0, 180.0);
    uint32_t pulse_us = 500 + (uint32_t)((safeAngle / 180.0) * 2000.0);
    servoObj.writeMicroseconds(pulse_us);
}

// ==========================================
// 核心 0：通訊接收
// ==========================================
void setup() {
    Serial.begin(115200);
    delay(2000); 
    //setupUART();
}

void loop() {
    checkSerialMonitor();
    // checkUART();
    // 斷線安全防護機制 (Fail-safe)
    if (millis() - lastPacketTime > 500) {
        initStates(); // 強制將狀態重設為 0

        // 在 Serial 印出警告
        static unsigned long lastWarningTime = 0;
        if (millis() - lastWarningTime > 2000) { // 改成每兩秒印一次
            Serial.println("⚠️ WARNING: Remote Controller Disconnected! Failsafe Activated (STOP).");
            lastWarningTime = millis();
        }
    }
}

void setup1() {
    delay(1000);
    servoArm1.attach(SERVO1_Arm, 500, 2500);
    servoArm2.attach(SERVO2_Arm, 500, 2500);
    servoJaw.attach(SERVO_JAW, 500, 2500);
    servoJaw1.attach(SERVO1_JAW, 500, 2500);
    servoJaw2.attach(SERVO2_JAW, 500, 2500);
    setupWalkerMotors();
    setupBaseMotor(); 
}

float getTargetVelocity(int rawValue) {
    if (rawValue > 3000) {
        return (float)(rawValue - 3000) / (4000.0 - 3000.0) * MAX_VELOCITY;
    } else if (rawValue < 2000) {
        return (float)(rawValue - 2000) / 2000.0 * MAX_VELOCITY;
    }
    return 0.0; 
}
int getWalkerPWM(int rawValue) {
    if (rawValue > 3000) {
        // 正向：對應 PWM 0 ~ 255
        return map(rawValue, 3000, 4000, 0, MAX_PWM_LIMIT);
    } else if (rawValue < 2000) {
        // 反向：對應 PWM 0 ~ -255 (帶負號表示反轉)
        return map(rawValue, 2000, 0, 0, -MAX_PWM_LIMIT);
    }
    return 0; // 搖桿在 2000~3000 的死區內，輸出 0 停止
}

void loop1() {
    /*float targetVelX = getTargetVelocity(joyX_raw);
    float targetVelY = 0.0;
    float targetVelZ = getTargetVelocity(joyY_raw);

    targetX += targetVelX;
    targetY += targetVelY;
    targetZ += targetVelZ;*/

    clawopen = (btnClaw_raw == 0);

    if (eStop) {
        targetX = safeX;
        targetY = safeY; 
        targetZ = safeZ;
        setWalkerSpeed(0, 0);
        delay(CONTROL_LOOP_DELAY);
        return;
    }

    float aBase, a1, a2;
    calculateAngles(targetX, targetY, targetZ, aBase, a1, a2);

    bool isValid = !isnan(aBase) && !isnan(a1) && !isnan(a2) &&
                   (a1 >= 10 && a1 <= 130) &&
                   ((a2 - a1) >= 45);

    if (isValid) {
        safeX = targetX;
        safeY = targetY;
        safeZ = targetZ;

        float out1 = 150.0 - a1;
        float out2 = a2 - 30.0;

        runBasePID(aBase); 
        triggerServo(servoArm1, out1);
        triggerServo(servoArm2, out2);
        triggerServo(servoJaw, -aBase + Gain);
    } else {
        targetX = safeX;
        targetY = safeY;
        targetZ = safeZ;
    }

    // 控制夾爪開合
    if (clawopen){
        triggerServo(servoJaw1, 0.0);
        triggerServo(servoJaw2, 0.0);
    } else {
        triggerServo(servoJaw1, 50.0);
        triggerServo(servoJaw2, 50.0);
    }
    /*
    int walkForward = getWalkerPWM(joyWalkY_raw); 
    int walkTurn    = getWalkerPWM(joyWalkX_raw); 
    */
    int leftMotorSpeed  = walkForward + 0.5*walkTurn;
    int rightMotorSpeed = walkForward - 0.5*walkTurn;

    leftMotorSpeed  = constrain(leftMotorSpeed, -MAX_PWM_LIMIT, MAX_PWM_LIMIT);
    rightMotorSpeed = constrain(rightMotorSpeed, -MAX_PWM_LIMIT, MAX_PWM_LIMIT)*1.25;

    setWalkerSpeed(leftMotorSpeed, rightMotorSpeed);

    delay(CONTROL_LOOP_DELAY); 
}