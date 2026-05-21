#include "Config.h"

#define TX2 4
#define RX2 5
#define SET_PIN 22 

String rxString = "";

// 初始化搖桿預設值 (中間值 1867)
volatile int joyX_raw = 1867; 
volatile int joyY_raw = 1867; 
volatile int btnClaw_raw = 1;

void setupUART() {
    Serial2.setTX(TX2);  
    Serial2.setRX(RX2);
    Serial2.begin(9600); 

    pinMode(SET_PIN, OUTPUT);
    digitalWrite(SET_PIN, HIGH); 
    
    Serial.println("Core 0: UART & Serial Monitor Ready.");
    Serial.println("Use Arrow Keys to move X/Z, 'W'/'S' for Y, 'C' for Claw.");
}

// -----------------------------------------
// 1. 遙控器 UART 接收邏輯
// -----------------------------------------
void parseUARTData(String data) {
    data.trim();
    int firstComma = data.indexOf(',');
    int secondComma = data.lastIndexOf(',');
    
    if (firstComma != -1 && secondComma != -1 && firstComma != secondComma) {
        String xStr = data.substring(0, firstComma);
        String yStr = data.substring(firstComma + 1, secondComma);
        String btnStr = data.substring(secondComma + 1);
        
        joyX_raw = xStr.toInt();
        joyY_raw = yStr.toInt();
        btnClaw_raw = btnStr.toInt();
    }
}

void checkUART() {
    while (Serial2.available()) {        
        char c = Serial2.read();
        if (c == '<') {       
            rxString = "";    
        } else if (c == '>') {  
            parseUARTData(rxString); 
            rxString = "";
        } else {
            if (isDigit(c) || c == ',') {
                rxString += c;
            }
        }
    }
}

// -----------------------------------------
// 2. 電腦鍵盤 Serial Monitor 接收邏輯
// -----------------------------------------
void checkSerialMonitor() {
    if (Serial.available() > 0) {
        int c = Serial.read();
        
        // 捕捉 ANSI 方向鍵 (格式通常為 ESC [ A)
        if (c == 27) { 
            delay(2); // 縮短 delay 以免阻礙 UART 接收
            if (Serial.available() && Serial.read() == '[') {
                delay(2);
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
                // 直接切換搖桿變數的狀態，確保能與遙控器同步
                btnClaw_raw = (btnClaw_raw == 0) ? 1 : 0;
                Serial.print("Claw toggled via Serial: "); 
                Serial.println((btnClaw_raw == 0) ? "CLOSE" : "OPEN");
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