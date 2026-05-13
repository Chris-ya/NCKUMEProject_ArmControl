// Main.ino
#include "hardware/pio.h"
#include "hardware/clocks.h"

#define SERVO1_Arm 11       // First Arm
#define SERVO2_Arm 12       // Second Arm
#define SERVO_JAW 13        // Jaw Angle
#define SERVO1_JAW 9
#define SERVO2_JAW 8

// PIO State Machine
PIO armpio = pio0;
PIO Jawpio = pio1;
PIO Feetpio = pio2;
uint sm0 = 0;
uint sm1 = 1; 
uint sm2 = 2; 
uint sm3 = 3; 

bool eStop = false;
// Arm Coordinate
float targetX = 168.3; 
float targetY = 0; 
float targetZ = 69.7; 

extern void setupBluetooth();
extern void checkBluetooth();
extern void calculateAngles(float x, float y, float z, float &angleBase, float &angle1, float &angle2);
extern void checkSerial();
extern void setupPioServo(PIO pio, uint sm, uint pin);
extern void setPioServoAngle(PIO pio, uint sm, float angle, float max_angle);


void setup() {
    Serial.begin(115200);

    setupBaseMotor();
    setupPioServo(armpio, sm1, SERVO1_Arm);
    setupPioServo(armpio, sm2, SERVO2_Arm);

    setupPioServo(Jawpio, sm0, SERVO_JAW);
    setupPioServo(Jawpio, sm1, SERVO1_JAW);
    setupPioServo(Jawpio, sm2, SERVO2_JAW);

    // setupBluetooth();
    delay(1000);
}

void loop() {
    float oldX = targetX;
    float oldY = targetY;
    float oldZ = targetZ;

    checkSerial();
    // checkBluetooth(); 

    // --- Emergency Stop ---
    if (eStop) {
        targetX = oldX;
        targetY = oldY;
        targetZ = oldZ;
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
        setPioServoAngle(armpio, sm1, out1, 180.0);
        setPioServoAngle(armpio, sm2, out2, 180.0);

        setPioServoAngle(Jawpio, sm0, -aBase, 180.0);
        //setPioServoAngle(Jawpio, sm1, -aBase, 180.0);
        //setPioServoAngle(Jawpio, sm2, -aBase, 180.0);

    } else {
        targetX = oldX;
        targetY = oldY;
        targetZ = oldZ;
    }

    delay(20); 
}
