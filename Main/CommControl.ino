#include "Config.h"
#include <SerialBT.h>

void checkSerial() {
    if (Serial.available() > 0) {
        int c = Serial.read();

        // 捕捉 ANSI 方向鍵 (格式通常為 ESC [ A)
        if (c == 27) { 
            delay(5);
            if (Serial.available() && Serial.read() == '[') {
                delay(5);
                if (Serial.available()) {
                    int dir = Serial.read();
                    switch(dir) {
                        case 'A': targetZ += 2.0; Serial.println("Up -> Z+"); break;
                        case 'B': targetZ -= 2.0; Serial.println("Down -> Z-"); break;
                        case 'C': targetX += 2.0; Serial.println("Right -> X+"); break;
                        case 'D': targetX -= 2.0; Serial.println("Left -> X-"); break;
                    }
                }
            }
        } 
        // 捕捉一般字母操作
        else {
            char cmd = toupper((char)c);
            if (cmd == 'W') { targetY += 2.0; Serial.println("W -> Y+"); }
            else if (cmd == 'S') { targetY -= 2.0; Serial.println("S -> Y-"); }
            else if (cmd == 'C') { 
                clawopen = !clawopen; 
                Serial.print("Claw toggled: "); Serial.println(clawopen ? "OPEN (55 deg)" : "CLOSE (0 deg)"); 
            }
            else if (cmd == 'E') {
                eStop = true;
                Serial.println("!!! EMERGENCY STOP ACTIVATED !!!");
            }
            else if (cmd == 'R') {
                eStop = false;
                Serial.println("System Reset. Resuming normal operation.");
            }
        }
    }
}

void setupBluetooth() {
    SerialBT.begin(115200);
    Serial.println("Bluetooth initialized.");
}