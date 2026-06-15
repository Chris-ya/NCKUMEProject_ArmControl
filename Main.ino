#include "Config.h"
#include <Servo.h> 

extern size_t bufferIndex;

Servo servoArm1;
Servo servoArm2;
Servo servoJaw;
Servo servoJaw1;
Servo servoJaw2;
Servo servoDoor;

volatile float targetX = 168.3;
volatile float targetY = 0;
volatile float targetZ = 120;
volatile bool eStop = false;
volatile bool clawopen = false; 
volatile bool dooropen = false;
volatile bool resetposition = false;
volatile int MAX_PWM_LIMIT = 200;
const float GAIN_GAIN = 1.5;
float Gain = 0.0;
const float MAX_VELOCITY = 1.2;      
const int CONTROL_LOOP_DELAY = 20; 
int walkForward = 0;
int walkTurn = 0;
float aBase = 0.0;

unsigned long lastPacketTime = 0;

float safeX = 168.3;
float safeZ = 69.7;

// --- 原點與復位相關設定 ---
const float HOME_X = 168.3;        // 原點 X 座標
const float HOME_Z = 120.0;        // 原點 Z 座標
const float HOME_BASE = 0.0;
const float SAFE_RETRACT_Z = 120.0; // 避開干涉的「安全抬升高度」(請依實機調整)
const float RESET_SPEED = 2.0;     // 復位時的移動速度 (數值越大跑越快)

int resetState = 0; // 狀態機：0=平常模式, 1=抬升到安全高度中, 2=回歸原點中


float JawBaseAngle(float aBase, float& Gain);
void triggerServo(Servo& servoObj, float angle);

void triggerServo(Servo& servoObj, float angle) {
    float safeAngle = constrain(angle, 0.0, 180.0);
    uint32_t pulse_us = 500 + (uint32_t)((safeAngle / 180.0) * 2000.0);
    servoObj.writeMicroseconds(pulse_us);
}
float JawBaseAngle(float aBase, float& Gain){
    float targetJawAngle = 90.0 + aBase + Gain;
    if(targetJawAngle < 1 || targetJawAngle > 179){
        float ans = constrain(targetJawAngle, 1, 179);
        Gain = ans - 90.0 - aBase;
        return ans;

    }else{
        return targetJawAngle;
    }
}

// 核心 0：通訊接收
void setup() {
    Serial.begin(115200);
    delay(2000); 
    setupUART();
}

void loop() {
    // checkSerialMonitor();
    checkUART();
    
    if (millis() - lastPacketTime > 500) {
        initStates();
        joyX_raw = 512;
        joyY_raw = 512;

        static unsigned long lastWarningTime = 0;
        if (millis() - lastWarningTime > 2000) {
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
    servoDoor.attach(SERVO_DOOR, 500, 2500);
    setupWalkerMotors();
    setupBaseMotor(); 
}

/*float getTargetVelocity(int rawValue) {
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
}*/

void loop1() {
    /*float targetVelX = getTargetVelocity(joyX_raw);
    float targetVelY = 0.0;
    float targetVelZ = getTargetVelocity(joyY_raw);

    targetX += targetVelX;
    targetY += targetVelY;
    targetZ += targetVelZ;*/

    clawopen = (btnClaw_raw == 0);
    dooropen = (btnDoor_raw == 0);

    if (eStop) {
        targetX = safeX;
        targetZ = safeZ;
        setWalkerSpeed(0, 0);
        delay(CONTROL_LOOP_DELAY);
        return;
    }

    if (resetposition && resetState == 0) {
        resetState = 1; // 啟動復位流程：進入第一階段
    }

    if (resetState == 1) {
        // 【階段 1】：X 軸保持不動，Z 軸直直往上抬升到安全高度 SAFE_RETRACT_Z
        if (targetZ < SAFE_RETRACT_Z) {
            targetZ += RESET_SPEED; 
            if (targetZ > SAFE_RETRACT_Z) targetZ = SAFE_RETRACT_Z; // 避免過沖
        } else {
            resetState = 2; // 已經到達安全高度，進入第二階段
        }
    } 
    else if (resetState == 2) {
        // 【階段 2】：從安全高度，將 X 與 Z 軸平滑移動到原點
        bool reachedX = false;
        bool reachedZ = false;
        bool reachedBase = false;

        // X 軸朝 HOME_X 逼近
        if (abs(targetX - HOME_X) <= RESET_SPEED) {
            targetX = HOME_X;
            reachedX = true;
        } else {
            targetX += (targetX < HOME_X) ? RESET_SPEED : -RESET_SPEED;
        }

        // Z 軸朝 HOME_Z 逼近
        if (abs(targetZ - HOME_Z) <= RESET_SPEED) {
            targetZ = HOME_Z;
            reachedZ = true;
        } else {
            targetZ += (targetZ < HOME_Z) ? RESET_SPEED : -RESET_SPEED;
        }

        if (abs(aBase - HOME_BASE) <= RESET_SPEED) {
            aBase = HOME_BASE;
            reachedBase = true;
        } else {
            aBase += (aBase < HOME_BASE) ? RESET_SPEED : -RESET_SPEED;
        }

        // 判斷是否完全抵達原點
        if (reachedX && reachedZ && reachedBase) {
            resetState = 0;          // 結束狀態機
            resetposition = false;   // 自動清除復位指令，還原控制權
        }
    }

    float a1, a2;
    calculateAngles(targetX, targetZ, a1, a2);

    bool isValid = !isnan(aBase) && !isnan(a1) && !isnan(a2) &&
                   (a1 >= 10 && a1 <= 130) &&
                   ((a2 - a1) >= 45);

    if (isValid) {
        safeX = targetX;
        safeZ = targetZ;

        float out1 = 150.0 - a1;
        float out2 = a2 - 30.0;

        runBasePID(aBase); 
        triggerServo(servoArm1, out1);
        triggerServo(servoArm2, out2);
        float jawAngle = JawBaseAngle(aBase, Gain);
        triggerServo(servoJaw, jawAngle);
    } else {
        targetX = safeX;
        targetZ = safeZ;
    }

    // 控制夾爪開合
    if (clawopen){
        triggerServo(servoJaw1, 15.0);
        triggerServo(servoJaw2, 15.0);
    } else {
        triggerServo(servoJaw1, 55.0);
        triggerServo(servoJaw2, 55.0);
    }

    if (dooropen){
        triggerServo(servoDoor, 15.0);
    } else {
        triggerServo(servoDoor, 80.0);
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