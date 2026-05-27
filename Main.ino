#include "Config.h"

// PIO 分配
PIO armpio = pio0;   
PIO Jawpio = pio1;
PIO Feetpio = pio2;  

bool eStop = false;
bool clawopen = false; 
float Gain = 0.0;

// 初始化手臂座標
float targetX = 168.3;
float targetY = 0;
float targetZ = 69.7;

const float MAX_VELOCITY = 1.2;      

const int CONTROL_LOOP_DELAY = 20; 

// ==========================================
// 核心 0 (Core 0)：專職通訊接收 (UART + USB)
// ==========================================
void setup() {
    Serial.begin(115200);
    setupUART();
}

void loop() {
    checkUART();          // 聽遙控器
    checkSerialMonitor(); // 聽電腦鍵盤
}

// ==========================================
// 核心 1 (Core 1)：專職進行高頻馬達控制
// ==========================================
void setup1() {
    setupBaseMotor();
    setupPioServo(armpio, 1, SERVO1_Arm);
    setupPioServo(armpio, 2, SERVO2_Arm);
    setupPioServo(Jawpio, 0, SERVO_JAW);
    setupPioServo(Jawpio, 1, SERVO1_JAW);
    setupPioServo(Jawpio, 2, SERVO2_JAW);
    setupWalkerMotors();
    delay(1000);
}

// 搖桿演算法：死區處理並映射至目標速度 (位移量)
float getTargetVelocity(int rawValue) {
    // 假設你採用標準 12-bit ADC 的中心點 2048，死區設為 1700~2300
    if (rawValue > 3000) {
        return (float)(rawValue - 3000) / (4000 - 3000) * MAX_VELOCITY;
    } else if (rawValue < 2000) {
        return (float)(rawValue - 2000) / 2000 * MAX_VELOCITY;
    }
    return 0.0; 
}

bool BtnState(bool fn_state, unit8_t btn, unit8_t last_btn){
    if(btn == 9 && last_btn == 1){
        rerturn fn_state = (fn_state == 0) ? 1 : 0;
    }
}

void loop1() {
    float oldX = targetX;
    float oldY = targetY;
    float oldZ = targetZ;

    float targetVelX = getTargetVelocity(joyX_raw);
    float targetVelY = 0.0; 
    float targetVelZ = getTargetVelocity(joyY_raw);

    targetX += targetVelX;
    targetY += targetVelY;
    targetZ += targetVelZ;

    clawopen = (btnClaw_raw == 0);

    if (eStop) {
        targetX = oldX;
        targetY = oldY; 
        targetZ = oldZ;
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
        float out1 = 130.0 - a1;
        float out2 = a2 - 40.0;

        runBasePID(aBase);
        setPioServoAngle(armpio, 1, out1, 180.0);
        setPioServoAngle(armpio, 2, out2, 180.0);
        setPioServoAngle(Jawpio, 0, -aBase + Gain, 180.0);
    } else {
        targetX = oldX;
        targetY = oldY;
        targetZ = oldZ;
    }

    if (clawopen){
        setPioServoAngle(Jawpio, 1, 55.0, 180.0);
        setPioServoAngle(Jawpio, 2, 55.0, 180.0);
    } else {
        setPioServoAngle(Jawpio, 1, 0.0, 180.0);
        setPioServoAngle(Jawpio, 2, 0.0, 180.0);
    }

    delay(CONTROL_LOOP_DELAY); 
}