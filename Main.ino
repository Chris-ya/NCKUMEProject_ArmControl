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

// 軌跡規劃變數
float currentVelX = 0;
float currentVelY = 0;
float currentVelZ = 0;

const float MAX_VELOCITY = 1.2;      
const float MAX_ACCELERATION = 0.04; 
const int CONTROL_LOOP_DELAY = 2;    

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
// 核心 1 (Core 1)：專職進行高頻馬達控制與插值
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

// 搖桿演算法：加入 2000~3000 雜訊死區，並映射至目標速度
float getTargetVelocity(int rawValue) {
    if (rawValue > 3000) {
        // 大於 3000，正向增加速度
        return (float)(rawValue - 3000) / (4095.0 - 3000.0) * MAX_VELOCITY;
    } else if (rawValue < 2000) {
        // 小於 2000，負向減少速度
        return (float)(rawValue - 2000) / 2000.0 * MAX_VELOCITY;
    }
    return 0.0; // 落在 2000~3000 區間內視為雜訊，馬達靜止
}

// 限制速度的變化率，達成梯形加減速 (Slew Rate Limiting)
void calculateSmoothVelocity(float &currentVel, float targetVel) {
    if (targetVel > currentVel + MAX_ACCELERATION) {
        currentVel += MAX_ACCELERATION;
    } else if (targetVel < currentVel - MAX_ACCELERATION) {
        currentVel -= MAX_ACCELERATION;
    } else {
        currentVel = targetVel;
    }
}

void loop1() {
    // 備份上一次的座標安全點
    float oldX = targetX;
    float oldY = targetY;
    float oldZ = targetZ;

    // 1. 將讀取到的全域搖桿數值轉化為目標速度
    // 備註：因為接收端目前為雙軸搖桿(X, Y)，此處示範 X 控制手臂 X 軸，Y 控制手臂 Z 軸 (高度)
    // 你可以隨時根據實際操作習慣更換映射關係
    float targetVelX = getTargetVelocity(joyX_raw);
    float targetVelY = 0.0; // 若後續增加第二支搖桿，可映射至 joyY2
    float targetVelZ = getTargetVelocity(joyY_raw); 

    // 2. 進行加減速平滑濾波
    calculateSmoothVelocity(currentVelX, targetVelX);
    calculateSmoothVelocity(currentVelY, targetVelY);
    calculateSmoothVelocity(currentVelZ, targetVelZ);

    // 3. 透過速度積分推算新的微小目標座標
    targetX += currentVelX;
    targetY += currentVelY;
    targetZ += currentVelZ;

    // 4. 解析夾爪開合狀態 (通常按鈕按下為 0，放開為 1)
    clawopen = (btnClaw_raw == 0); 

    // 緊急停止安全機制
    if (eStop) {
        targetX = oldX; targetY = oldY; targetZ = oldZ;
        currentVelX = 0; currentVelY = 0; currentVelZ = 0;
        setWalkerSpeed(0, 0);
        delay(CONTROL_LOOP_DELAY);
        return;
    }

    // 5. 逆運動學運算與幾何限制驗證
    float aBase, a1, a2;
    calculateAngles(targetX, targetY, targetZ, aBase, a1, a2);
    
    bool isValid = !isnan(aBase) && !isnan(a1) && !isnan(a2) &&
                   (a1 >= 10 && a1 <= 130) &&
                   ((a2 - a1) >= 45);

    if (isValid) {
        float out1 = 130.0 - a1;
        float out2 = a2 - 40.0;

        // 驅動基座馬達與手臂伺服馬達
        runBasePID(aBase);
        setPioServoAngle(armpio, 1, out1, 180.0);
        setPioServoAngle(armpio, 2, out2, 180.0);
        setPioServoAngle(Jawpio, 0, -aBase + Gain, 180.0);
    } else {
        // 若運算超出物理死角，退回上一點並將速度清空，防止過衝抽動
        targetX = oldX;
        targetY = oldY;
        targetZ = oldZ;
        currentVelX = 0; currentVelY = 0; currentVelZ = 0;
    }

    // 6. 夾爪控制 (MG90S 角度限制 0 ~ 55 度)
    if (clawopen){
        setPioServoAngle(Jawpio, 1, 55.0, 180.0);
        setPioServoAngle(Jawpio, 2, 55.0, 180.0);
    } else {
        setPioServoAngle(Jawpio, 1, 0.0, 180.0);
        setPioServoAngle(Jawpio, 2, 0.0, 180.0);
    }

    // 控制核心嚴格維持 2ms 的高頻更新，讓物理慣性自動平滑軌跡
    delay(CONTROL_LOOP_DELAY); 
}