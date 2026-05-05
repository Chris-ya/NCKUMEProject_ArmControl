// Main.ino
#include "hardware/pio.h"
#include "hardware/clocks.h"

#define SERVO_BASE_PIN 10   // Base Servo
#define SERVO1_PIN 11       // First Arm
#define SERVO2_PIN 12       // Second Arm
#define SERVO_JAW 13        // Jaw Angle

bool eStop = false;

float targetX = 168.3; 
float targetY = 0; 
float targetZ = 69.7; 

extern void setupBluetooth();
extern void checkBluetooth();
extern void calculateAngles(float x, float y, float z, float &angleBase, float &angle1, float &angle2);
extern void checkSerial();

PIO pio = pio0;
uint sm_base = 0;
uint sm1 = 1;
uint sm2 = 2;

static const uint16_t pio_servo_instructions[] = {
    0x8080, 0xa027, 0xa046, 0x00a5, 0xe000, 0x0083, 0xe001
};

static const struct pio_program pio_servo_program = {
    .instructions = pio_servo_instructions,
    .length = 7,
    .origin = -1,
};

void setupPioServo(PIO pio, uint sm, uint pin) {
    uint offset = pio_add_program(pio, &pio_servo_program);
    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
    
    pio_sm_config c = pio_get_default_sm_config();
    
    sm_config_set_wrap(&c, offset, offset + 6);
    sm_config_set_set_pins(&c, pin, 1);
    
    sm_config_set_clkdiv(&c, (float)clock_get_hz(clk_sys) / 2000000.0f);
    
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
    
    pio_sm_put_blocking(pio, sm, 20000);
    pio_sm_exec(pio, sm, pio_encode_pull(false, false));
    pio_sm_exec(pio, sm, pio_encode_mov(pio_isr, pio_osr));
}

void setPioServoAngle(PIO pio, uint sm, float angle, float max_angle) {
    float safeAngle = (angle < 0) ? 0 : (angle > max_angle ? max_angle : angle);
    uint32_t on_time_us = 500 + (uint32_t)((safeAngle / max_angle) * 2000.0);
    uint32_t duty_val = 20000 - on_time_us; 
    pio_sm_put_blocking(pio, sm, duty_val);
}

void setup() {
    Serial.begin(115200);
    
    // 初始化三個軸的 PIO 控制
    setupPioServo(pio, sm_base, SERVO_BASE_PIN);
    setupPioServo(pio, sm1, SERVO1_PIN);
    setupPioServo(pio, sm2, SERVO2_PIN);
    
    // setupBluetooth();
    delay(1000);
}

void loop() {
    float oldX = targetX;
    float oldY = targetY;
    float oldZ = targetZ;

    checkSerial();
    // checkBluetooth(); 

    // --- 緊急停止攔截 ---
    if (eStop) {
        // 放棄新的輸入，鎖死在最後的安全座標
        targetX = oldX;
        targetY = oldY;
        targetZ = oldZ;
        delay(20);
        return; // 直接跳出這次 loop，不更新馬達
    }

    float aBase, a1, a2;
    // 逆向運動學計算
    calculateAngles(targetX, targetY, targetZ, aBase, a1, a2);

    // 檢查角度是否在安全範圍內，避免機構碰撞
    bool isValid = !isnan(aBase) && !isnan(a1) && !isnan(a2) &&
                   (a1 >= 10 && a1 <= 130) &&
                   ((a2 - a1) >= 45); 

    if (isValid) {
        // 根據硬體組裝方向調整實際輸出的角度
        float out1 = 130.0 - a1;
        float out2 = a2 - 40.0;

        // 驅動三軸馬達
        setPioServoAngle(pio, sm_base, aBase, 180.0);
        setPioServoAngle(pio, sm1, out1, 180.0);
        setPioServoAngle(pio, sm2, out2, 180.0);
    } else {
        // 若角度不合法，退回上一個安全座標
        targetX = oldX;
        targetY = oldY;
        targetZ = oldZ;
    }

    delay(20); 
}