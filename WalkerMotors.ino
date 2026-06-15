#include "Config.h"

void setupWalkerMotors() {
    pinMode(WALKER_L_EN, OUTPUT); 
    pinMode(WALKER_R_EN, OUTPUT); 
    
    digitalWrite(WALKER_L_EN, HIGH);
    digitalWrite(WALKER_R_EN, HIGH); 
    
    pinMode(WALKER_M1_LPWM, OUTPUT);
    pinMode(WALKER_M1_RPWM, OUTPUT);
    pinMode(WALKER_M2_LPWM, OUTPUT);
    pinMode(WALKER_M2_RPWM, OUTPUT);
}

void setWalkerSpeed(int leftSpeed, int rightSpeed) {
    if (leftSpeed > 0) {
        analogWrite(WALKER_M1_LPWM, leftSpeed);
        analogWrite(WALKER_M1_RPWM, 0);
    } else if (leftSpeed < 0) {
        analogWrite(WALKER_M1_LPWM, 0);
        analogWrite(WALKER_M1_RPWM, abs(leftSpeed));
    } else {
        analogWrite(WALKER_M1_LPWM, 0);
        analogWrite(WALKER_M1_RPWM, 0);
    }

    if (rightSpeed > 0) {
        analogWrite(WALKER_M2_LPWM, rightSpeed);
        analogWrite(WALKER_M2_RPWM, 0);
    } else if (rightSpeed < 0) {
        analogWrite(WALKER_M2_LPWM, 0);
        analogWrite(WALKER_M2_RPWM, abs(rightSpeed));
    } else {
        analogWrite(WALKER_M2_LPWM, 0);
        analogWrite(WALKER_M2_RPWM, 0);
    }
}