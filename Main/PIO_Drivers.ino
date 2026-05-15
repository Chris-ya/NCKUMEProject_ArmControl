#include "Config.h"

// --- PIO Servo Driver ---
static const uint16_t pio_servo_instructions[] = {
    0x8080, 0xa027, 0xa046, 0x00a5, 0xe000, 0x0083, 0xe001
};
static const struct pio_program pio_servo_program = {
    .instructions = pio_servo_instructions,
    .length = 7,
    .origin = -1,
};

void setupPioServo(PIO pio, uint sm, uint pin) {
    static uint offset_servo = 0;
    static bool programLoaded_servo = false;
    if (!programLoaded_servo) {
        offset_servo = pio_add_program(pio, &pio_servo_program);
        programLoaded_servo = true;
    }
    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset_servo, offset_servo + 6);
    sm_config_set_set_pins(&c, pin, 1);
    sm_config_set_clkdiv(&c, (float)clock_get_hz(clk_sys) / 2000000.0f);
    pio_sm_init(pio, sm, offset_servo, &c);
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

// --- PIO DC Motor PWM Driver ---
const uint32_t PIO_PWM_PERIOD = 2000;
static const uint16_t pio_pwm_instructions[] = {
    0x8080, 0xa027, 0xa046, 0x00a5, 0xe000, 0x0083, 0xe001
};
static const struct pio_program pio_pwm_program = {
    .instructions = pio_pwm_instructions,
    .length = 7,
    .origin = -1,
};

void setupPioMotorPWM(PIO pio, uint sm, uint pin) {
    static uint offset_pwm = 0;
    static bool programLoaded_pwm = false;
    if(!programLoaded_pwm) {
        offset_pwm = pio_add_program(pio, &pio_pwm_program);
        programLoaded_pwm = true;
    }
    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset_pwm, offset_pwm + 6);
    sm_config_set_set_pins(&c, pin, 1);
    sm_config_set_clkdiv(&c, (float)clock_get_hz(clk_sys) / 2000000.0f);
    pio_sm_init(pio, sm, offset_pwm, &c);
    pio_sm_set_enabled(pio, sm, true);
    pio_sm_put_blocking(pio, sm, PIO_PWM_PERIOD);
    pio_sm_exec(pio, sm, pio_encode_pull(false, false));
    pio_sm_exec(pio, sm, pio_encode_mov(pio_isr, pio_osr));
}

void setPioMotorPower(PIO pio, uint sm, int pwmValue_0_to_255) {
    uint32_t on_time = (pwmValue_0_to_255 * PIO_PWM_PERIOD) / 255;
    uint32_t duty_val = PIO_PWM_PERIOD - on_time;     
    pio_sm_put_blocking(pio, sm, duty_val);
}