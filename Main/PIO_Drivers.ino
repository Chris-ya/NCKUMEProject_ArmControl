#include "Config.h"

// ==========================================
// 1. PIO Servo Driver (伺服馬達驅動)
// ==========================================
static const uint16_t pio_servo_instructions[] = {
    0x8080, 0xa027, 0xa046, 0x00a5, 0xe000, 0x0083, 0xe001
};

static const struct pio_program pio_servo_program = {
    .instructions = pio_servo_instructions,
    .length = 7,
    .origin = -1,
};

void setupPioServo(PIO pio, uint sm, uint pin) {
    if (!pio_sm_is_claimed(pio, sm)) {
        pio_sm_claim(pio, sm);
    }

    static uint offset_servo[3];
    static bool programLoaded_servo[3] = {false, false, false};
    
    uint pio_idx = pio_get_index(pio);
    if (!programLoaded_servo[pio_idx]) {
        offset_servo[pio_idx] = pio_add_program(pio, &pio_servo_program);
        programLoaded_servo[pio_idx] = true;
    }
    
    uint offset = offset_servo[pio_idx];

    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
    
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset, offset + 6);
    sm_config_set_set_pins(&c, pin, 1);
    
    // 設定時脈，產生 20ms 週期
    sm_config_set_clkdiv(&c, (float)clock_get_hz(clk_sys) / 2000000.0f);
    
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);

    // 🚀 初始化週期與 OSR (完全比照 Test.ino 的完美邏輯)
    pio_sm_put_blocking(pio, sm, 20000);
    pio_sm_exec(pio, sm, pio_encode_pull(false, false));
    pio_sm_exec(pio, sm, pio_encode_mov(pio_isr, pio_osr));
}

void setPioServoAngle(PIO pio, uint sm, float angle, float max_angle) {
    // 軟體限幅防爆衝
    if (angle < 0.0) angle = 0.0;
    if (angle > max_angle) angle = max_angle;
    
    // 計算脈衝寬度 (500us ~ 2500us)
    float pulse_us = 500.0 + (angle / max_angle) * 2000.0;
    
    uint32_t compare_value = 20000 - (uint32_t)pulse_us;
    
    // 🚀 關鍵修正：改用 blocking，讓硬體時鐘完美接管主迴圈節奏
    pio_sm_put_blocking(pio, sm, compare_value);
}

// ==========================================
// 2. PIO PWM Driver (直流馬達驅動)
// ==========================================
static const uint16_t pio_pwm_instructions[] = {
    0x8080, 0xa027, 0xa046, 0x00a5, 0xe000, 0x0083, 0xe001
};

static const struct pio_program pio_pwm_program = {
    .instructions = pio_pwm_instructions,
    .length = 7,
    .origin = -1,
};

void setupPioMotorPWM(PIO pio, uint sm, uint pin) {
    if (!pio_sm_is_claimed(pio, sm)) {
        pio_sm_claim(pio, sm);
    }

    static uint offset_pwm[3];
    static bool programLoaded_pwm[3] = {false, false, false};
    
    uint pio_idx = pio_get_index(pio);
    if (!programLoaded_pwm[pio_idx]) {
        offset_pwm[pio_idx] = pio_add_program(pio, &pio_pwm_program);
        programLoaded_pwm[pio_idx] = true;
    }
    
    uint offset = offset_pwm[pio_idx];

    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
    
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset, offset + 6);
    sm_config_set_set_pins(&c, pin, 1);
    
    sm_config_set_clkdiv(&c, (float)clock_get_hz(clk_sys) / 2000000.0f);
    
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);

    // 🚀 直流馬達也比照 Test.ino 的邏輯，預防 PWM 雜訊
    pio_sm_put_blocking(pio, sm, 20000);
    pio_sm_exec(pio, sm, pio_encode_pull(false, false));
    pio_sm_exec(pio, sm, pio_encode_mov(pio_isr, pio_osr));
}

void setPioMotorPower(PIO pio, uint sm, int pwmValue_0_to_255) {
    if (pwmValue_0_to_255 < 0) pwmValue_0_to_255 = 0;
    if (pwmValue_0_to_255 > 255) pwmValue_0_to_255 = 255;
    
    uint32_t pulse_us = (pwmValue_0_to_255 * 20000) / 255;
    uint32_t compare_value = 20000 - pulse_us;
    
    // 🚀 關鍵修正：改用 blocking
    pio_sm_put_blocking(pio, sm, compare_value);
}