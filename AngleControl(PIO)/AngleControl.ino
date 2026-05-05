#include <Arduino.h>
#include "hardware/pio.h"
#include "hardware/clocks.h"

const int servoPin = 13; // 伺服馬達腳位

PIO pio = pio0;
uint sm = 0; 

// --- 1. 修正後的 PIO 機器碼 ---
static const uint16_t pio_servo_instructions[] = {
    0x8080, // 0: pull noblock 
    0xa027, // 1: mov x, osr   
    0xa046, // 2: mov y, isr   
    0x00a5, // 3: jmp x!=y, 5  
    0xe000, // 4: set pins, 0  
    0x0083, // 5: jmp y--, 3   
    0xe001  // 6: set pins, 1  
};

static const struct pio_program pio_servo_program = {
    .instructions = pio_servo_instructions,
    .length = 7, 
    .origin = -1,
};

// --- 2. PIO 初始化 ---
void setupPioServo(PIO pio, uint sm, uint pin) {
    uint offset = pio_add_program(pio, &pio_servo_program);
    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
    
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset, offset + 6);
    sm_config_set_set_pins(&c, pin, 1);
    
    // 【關鍵修正】將 PIO 時脈設為 2MHz，抵消迴圈的 2 個指令週期！
    sm_config_set_clkdiv(&c, (float)clock_get_hz(clk_sys) / 2000000.0f); 
    
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
    
    pio_sm_put_blocking(pio, sm, 20000);
    pio_sm_exec(pio, sm, pio_encode_pull(false, false));
    pio_sm_exec(pio, sm, pio_encode_mov(pio_isr, pio_osr));
}

// --- 3. 角度計算與寫入 ---
void setPioServoAngle(PIO pio, uint sm, float angle, float max_angle) {
    float safeAngle = (angle < 0) ? 0 : (angle > max_angle ? max_angle : angle); 
    
    // 現在這裡的數學計算會 100% 準確反映在馬達上了！
    uint32_t on_time_us = 500 + (uint32_t)((safeAngle / max_angle) * 2000.0);
    uint32_t duty_val = 20000 - on_time_us; 
    
    pio_sm_put_blocking(pio, sm, duty_val);
}

void setup() {
    Serial.begin(9600); 
    setupPioServo(pio, sm, servoPin);
    
    // 預設轉到 90 度
    setPioServoAngle(pio, sm, 180, 180.0);
    
    Serial.println("=== 🟢 完美 PIO 伺服控制系統 ===");
    Serial.println("請輸入 0 到 180 的角度：");
}

void loop() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input.length() > 0) {
            int angle = input.toInt(); 
            
            if (angle >= 0 && angle <= 180) {
                setPioServoAngle(pio, sm, angle, 180.0);
                
                Serial.print("🎯 成功轉至: ");
                Serial.print(angle);
                Serial.println(" 度");
            } else {
                Serial.println("❌ 錯誤：請輸入 0 到 180 之間的數字！");
            }
        }
    }
}