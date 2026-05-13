#include <Arduino.h>

#define ENCA_PIN 6
#define ENCB_PIN 7
#define PWMA_PIN 2
#define AIN1_PIN 3
#define AIN2_PIN 4

const float PPR = 11;
const float Internal_Gear_Ratio = 46.8;
const float Outer_Gear_Ratio = 3;
const float TICKS_PER_REV = PPR * Internal_Gear_Ratio * Outer_Gear_Ratio; 

float Kp = 2.5;  
float Ki = 0.01; 
float Kd = 0.5;  

volatile long currentTicks = 0; 
float errorSum = 0;
float lastError = 0;

       
const uint32_t PIO_PWM_PERIOD = 2000; // 設定週期為 2000 (約等於 500Hz，適合直流馬達)

static const uint16_t pio_pwm_instructions[] = {
    0x8080, 0xa027, 0xa046, 0x00a5, 0xe000, 0x0083, 0xe001
};

static const struct pio_program pio_pwm_program = {
    .instructions = pio_pwm_instructions,
    .length = 7,
    .origin = -1,
};

void setupPioMotorPWM(PIO pio, uint sm, uint pin) {
    uint offset = pio_add_program(pio, &pio_pwm_program);
    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
    
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset, offset + 6);
    sm_config_set_set_pins(&c, pin, 1);
    
    sm_config_set_clkdiv(&c, (float)clock_get_hz(clk_sys) / 2000000.0f);
    
    pio_sm_init(pio, sm, offset, &c);
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

void readEncoder() {
    int b = digitalRead(ENCB_PIN);
    if (b > 0) {
        currentTicks++;
    } else {
        currentTicks--;
    }
}

void setupBaseMotor() {
    pinMode(ENCA_PIN, INPUT_PULLUP);
    pinMode(ENCB_PIN, INPUT_PULLUP);
    pinMode(AIN1_PIN, OUTPUT);
    pinMode(AIN2_PIN, OUTPUT);
    setupPioMotorPWM(base_pio, base_sm, PWMA_PIN);

    attachInterrupt(digitalPinToInterrupt(ENCA_PIN), readEncoder, RISING);
}

void setMotorPower(int power) {
    if (power > 0) {
        digitalWrite(AIN1_PIN, HIGH);
        digitalWrite(AIN2_PIN, LOW);
    } else if (power < 0) {
        digitalWrite(AIN1_PIN, LOW);
        digitalWrite(AIN2_PIN, HIGH);
    } else {
        digitalWrite(AIN1_PIN, LOW);
        digitalWrite(AIN2_PIN, LOW);
    }
    
    int pwmValue = abs(power);
    if (pwmValue > 255) pwmValue = 255;
    setPioMotorPower(base_pio, base_sm, pwmValue);
}

void runBasePID(float targetAngleDegree) {
    
    long targetTicks = (targetAngleDegree / 360.0) * TICKS_PER_REV;
    
    long error = targetTicks - currentTicks;
    
    errorSum += error;
    float dError = error - lastError;
    
    if(errorSum > 1000) errorSum = 1000;
    if(errorSum < -1000) errorSum = -1000;

    float output = (Kp * error) + (Ki * errorSum) + (Kd * dError);
    
    setMotorPower((int)output);
    
    lastError = error;
}
