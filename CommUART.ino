#include "Config.h"

#define TX2 4
#define RX2 5
//#define SET_PIN 22 

float CoorSen = 1;
String rxString = "";

// 初始化搖桿預設值 
volatile int joyX_raw = 2467; 
volatile int joyY_raw = 2467; 
volatile int btnClaw_raw = 1;

uint8_t lastClawBtnState = 1;
uint8_t lastStopBtnState = 1;

uint8_t currentClawBtnState = 0;


struct __attribute__((__packed__)) RobotControlPacket {
  char startMarker;           
  uint16_t analogSignals[6];  
  uint8_t btnState[8];
  char endMarker;             
};

RobotControlPacket rxPacket;
// bool btnState[8] = {0};

bool isButtonPressed(uint8_t currentState, uint8_t &lastState) {
    bool justPressed = (currentState == 0 && lastState == 1);
    lastState = currentState;
    return justPressed;
}

void setupUART() {
    Serial.begin(115200);
    
    Serial2.setTX(TX2);  
    Serial2.setRX(RX2);
    Serial2.begin(9600); 

    /*pinMode(SET_PIN, OUTPUT);
    digitalWrite(SET_PIN, HIGH);
    */
    
    Serial.println("Waiting for remote controller..."); // 修正拼寫
    pinMode(LED_BUILTIN, OUTPUT);
}

// -----------------------------------------
// 1. 遙控器 UART 接收邏輯
// -----------------------------------------

void checkUART() {
    if (Serial2.available() >= sizeof(RobotControlPacket)) {
        
        // 修正 3：檢查開頭，若不是 '<'，則從 Serial2 讀掉一個垃圾字元
        if (Serial2.peek() != '<') {
            Serial2.read(); 
            return; // 提早結束，讓下一個 loop 繼續清垃圾
        }

        // 讀取整個封包 
        Serial2.readBytes((uint8_t*)&rxPacket, sizeof(RobotControlPacket));

        // 修正 4：確保封包完整性 
        if (rxPacket.startMarker == '<' && rxPacket.endMarker == '>') {
            
            
            joyX_raw = rxPacket.analogSignals[0];
            joyY_raw = rxPacket.analogSignals[1];
            if (isButtonPressed(bitRead(rxPacket.digitalBits, 0), lastClawBtnState)) {
                btnClaw_raw = (btnClaw_raw == 0) ? 1 : 0; 
                Serial.print("Claw toggled! State: ");
                Serial.println(btnClaw_raw);
            }
            if (isButtonPressed(bitRead(rxPacket.digitalBits, 1), lastStopBtnState)) {
                eStop = !eStop; 
                Serial.print("E-Stop toggled! State: ");
                Serial.println(eStop);
            }
        }
    }
}

// -----------------------------------------
// 2. 電腦鍵盤 Serial Monitor 接收邏輯 (這部分邏輯沒問題，保留原樣)
// -----------------------------------------
void checkSerialMonitor() {
    if (Serial.available() > 0) {
        int c = Serial.read();
        // 捕捉 ANSI 方向鍵 (格式通常為 ESC [ A)
        if (c == 27) { 
            delay(2);
            // 縮短 delay 以免阻礙 UART 接收
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
