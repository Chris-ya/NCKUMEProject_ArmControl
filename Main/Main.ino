#include "Config.h"

// PIO 分配
PIO armpio = pio0;   // sm0: Base PWM, sm1: Arm1, sm2: Arm2
PIO Jawpio = pio1;   // sm0: JawBase, sm1: Jaw1, sm2: Jaw2
PIO Feetpio = pio2;  // sm0~3: Walker Motors (BTS7960 LPWM/RPWM)

bool eStop = false;
bool clawopen = false; // [Bug Fix] 修改 flase 為 false
float Gain = 0.0;      // [Bug Fix] 新增 Gain 變數以修復編譯錯誤

// 初始化手臂座標
float targetX = 168.3;
float targetY = 0; 
float targetZ = 69.7; 

void setup() {
    Serial.begin(115200);

    // 基座馬達初始化
    setupBaseMotor();

    // 手臂伺服馬達初始化
    setupPioServo(armpio, 1, SERVO1_Arm);
    setupPioServo(armpio, 2, SERVO2_Arm);

    // 夾爪伺服馬達初始化
    setupPioServo(Jawpio, 0, SERVO_JAW);
    setupPioServo(Jawpio, 1, SERVO1_JAW);
    setupPioServo(Jawpio, 2, SERVO2_JAW);

    // 詹森機構步足馬達初始化
    setupWalkerMotors();

    // setupBluetooth();
    delay(1000);
    Serial.println("System Ready. Use Arrow Keys to move X/Z, 'W'/'S' for Y, 'C' for Claw.");
}

void loop() {
    float oldX = targetX;
    float oldY = targetY;
    float oldZ = targetZ;

    checkSerial();

    // --- 緊急停止 ---
    if (eStop) {
        targetX = oldX;
        targetY = oldY;
        targetZ = oldZ;
        setWalkerSpeed(0, 0); // 停止步足
        delay(20);
        return; 
    }

    float aBase, a1, a2;
    calculateAngles(targetX, targetY, targetZ, aBase, a1, a2);

    bool isValid = !isnan(aBase) && !isnan(a1) && !isnan(a2) &&
                   (a1 >= 10 && a1 <= 130) &&
                   ((a2 - a1) >= 45);

    if (isValid) {
        float out1 = 130.0 - a1;
        float out2 = a2 - 40.0;

        runBasePID(aBase);
        setPioServoAngle(armpio, 1, out1, 180.0);
        setPioServoAngle(armpio, 2, out2, 180.0);
        setPioServoAngle(Jawpio, 0, -aBase + Gain, 180.0);
    } else {
        // 如果運算無效，退回上一個座標
        targetX = oldX;
        targetY = oldY;
        targetZ = oldZ;
    }

    // 夾爪控制 (MG90S 角度限制 0 ~ 55 度)
    if (clawopen){
        setPioServoAngle(Jawpio, 1, 55.0, 180.0);
        setPioServoAngle(Jawpio, 2, 55.0, 180.0);
    } else {
        setPioServoAngle(Jawpio, 1, 0.0, 180.0);
        setPioServoAngle(Jawpio, 2, 0.0, 180.0);
    }

    delay(20); 
}