#include "Config.h"

float CoorSen = 1;
String rxString = "";
float stepSize = 10.0;
// 初始化搖桿預設值 (置中)
volatile int joyX_raw = 2467; 
volatile int joyY_raw = 2467;
volatile int joyWalkX_raw = 2467; // 負責步足左右轉向 (預設置中)
volatile int joyWalkY_raw = 2467; // 負責步足前進後退 (預設置中)
volatile int btnClaw_raw = 0;
volatile int btnDoor_raw = 0;
uint8_t lastClawBtnState = 1;
uint8_t lastStopBtnState = 1;
uint8_t currentClawBtnState = 0;

// ==========================================
// 無線通訊封包結構與緩衝區定義
// ==========================================
struct __attribute__((__packed__)) RobotControlPacket {
  char startMarker;           // 1 Byte
  uint8_t _padding1;          // 1 Byte (對齊填充)
  uint16_t analogSignals[6];  // 12 Bytes 
  uint8_t btnState[8];        // 8 Bytes 
  char endMarker;             // 1 Byte
  uint8_t _padding2;          // 1 Byte (湊滿 24 Bytes)
};

struct __attribute__((__packed__)) RobotFeedbackPacket {
  char startMarker = '[';     // 1 Byte (回傳開頭標記)
  int16_t robotArmX;          // 2 Bytes (回傳手臂實體 targetX)
  int16_t robotArmY;          // 2 Bytes (回傳手臂實體 targetY)
  char endMarker = ']';       // 1 Byte (回傳結尾標記)
  uint16_t _padding = 0;      // 2 Bytes (記憶體對齊填充)
};

RobotControlPacket rxPacket;
const size_t packetSize = sizeof(RobotControlPacket);

uint8_t rxBuffer[24];                                 
size_t bufferIndex = 0;

RobotFeedbackPacket txFeedbackPacket;

unsigned long lastSuccessTime = 0; 
unsigned long packetInterval = 0;

// ==========================================
// 輔助函式
// ==========================================
bool isButtonPressed(uint8_t currentState, uint8_t &lastState) {
    bool justPressed = (currentState == 0 && lastState == 1);
    lastState = currentState;
    return justPressed;
}

void setupUART() {
    Serial.begin(115200);

    Serial1.setTX(TX1);  
    Serial1.setRX(RX1);
    Serial1.begin(9600);

    Serial.println("Serial Launch!");
}

// ==========================================
// 解析來自無線搖桿的訊號 (UART1)
// ==========================================
int signalStates[6] = {0, 0, 0, 0, 0, 0};
uint8_t lastBtnStates[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int decodedBtnOutputs[8] = {0, 0, 0, 0, 0, 0, 0, 0};

unsigned long lastByteIncomingTime = 0;

void checkUART() {
    if (bufferIndex > 0 && (millis() - lastByteIncomingTime > 100)) {
        bufferIndex = 0;
        while(Serial1.available() > 0) { Serial1.read(); }
        Serial.println("♻️ Packet assembly timeout! Serial buffer auto-flushed.");
    }

    while (Serial1.available() > 0) {
        uint8_t incomingByte = Serial1.read();
        lastByteIncomingTime = millis(); 

        if (bufferIndex == 0 && incomingByte != '<') {
            continue;
        }

        rxBuffer[bufferIndex++] = incomingByte;

        if (bufferIndex >= 24) {
            memcpy(&rxPacket, rxBuffer, packetSize);

            if (rxPacket.startMarker == '<' && rxPacket.endMarker == '>') {
                unsigned long currentTime = millis();
                lastPacketTime = currentTime; 

                lastSuccessTime = currentTime;

                for (int i = 0; i < 6; i++) {
                    if ((int)rxPacket.analogSignals[i] >= 500 && (int)rxPacket.analogSignals[i] <= 700) {
                        signalStates[i] = 0;
                    } 
                    else if ((int)rxPacket.analogSignals[i] > 700) {
                        signalStates[i] = 1;
                    } 
                    else {
                        signalStates[i] = -1;
                    }
                }

                joyX_raw = rxPacket.analogSignals[0];
                joyY_raw = rxPacket.analogSignals[1];
                
                for (int i = 0; i < 8; i++) {
                    uint8_t currentBtnState = rxPacket.btnState[i];

                    if (currentBtnState == 0 && lastBtnStates[i] == 1) {
                        
                        if (i == 0) {
                            btnClaw_raw = (btnClaw_raw == 0) ? 1 : 0;
                            decodedBtnOutputs[0] = btnClaw_raw; 
                        }
                        else if (i == 3) {
                            btnDoor_raw = (btnDoor_raw == 0) ? 1 : 0;
                            decodedBtnOutputs[3] = btnDoor_raw;

                            if (btnDoor_raw == 1) {
                                startHaruhikage();
                            }
                        } 
                        else {
                            decodedBtnOutputs[i] = (decodedBtnOutputs[i] == 0) ? 1 : 0;
                        }
                    }
                    lastBtnStates[i] = currentBtnState;
                }

                // 1. 歸位按鈕 (按鈕 2)：觸發歸位並「儲存」當下座標
                if (decodedBtnOutputs[1] == 1) {
                    decodedBtnOutputs[1] = 0;
                    
                    // 在進入歸位前，儲存當下座標並賦予復原資格
                    memX = targetX;
                    memZ = targetZ;
                    memBase = aBase;
                    memGain = Gain;
                    canReturnToPrev = true; 
                    
                    resetposition = true;
                    restorePosition = false; // 若原本正在復原，強制中斷切換為歸位
                }

                // 按鈕 3 (索引 2)：設定 0 度原點
                if (decodedBtnOutputs[2] == 1) {
                    decodedBtnOutputs[2] = 0; 
                    
                    noInterrupts();
                    baseTicks = 0; 
                    interrupts();
                    
                    aBase = 0.0;
                    Gain = 0.0;    
                    
                    Serial.println("🎯 遙控器觸發：基座 0 度原點已重新校正！");
                }

                // 2. 復原按鈕 (按鈕 4)：觸發兩段式平滑返回
                if (decodedBtnOutputs[6] == 1) {
                    decodedBtnOutputs[6] = 0;
                    
                    if (canReturnToPrev && !resetposition) {
                        restorePosition = true;  // 啟動 Main.ino 的復原狀態機
                        canReturnToPrev = false; // 資格用過一次即失效
                        Serial.println("🔙 遙控器觸發：啟動兩段式平滑返回！");
                    }
                }

                // -----------------------------------------------------------
                // 執行動作控制分配與「搖桿介入失效機制」
                // -----------------------------------------------------------
                walkForward = MAX_PWM_LIMIT * signalStates[5];
                walkTurn = MAX_PWM_LIMIT * signalStates[4];
                
                // 偵測搖桿是否有動作 (代表人類想手動接管控制)
                bool joystickActive = (signalStates[0] != 0 || signalStates[1] != 0 || 
                                       signalStates[2] != 0 || signalStates[3] != 0);

                if (joystickActive) {
                    canReturnToPrev = false;  // 只要手動介入，復原資格立刻失效
                    restorePosition = false;  // 如果正在自動復原中，立刻煞車停住
                    
                    // 🎯 新增：讓搖桿也能打斷「歸位 (resetposition)」過程
                    resetposition = false;    
                }

                // 只有在「沒有自動歸位」且「沒有自動復原」的情況下，搖桿才能改寫座標
                if (!resetposition && !restorePosition) {
                    targetX += MAX_VELOCITY * signalStates[0] * 4;
                    targetZ -= MAX_VELOCITY * signalStates[3] * 4;
                    
                    aBase -= GAIN_GAIN * signalStates[1];
                    while (aBase > 180.0)  aBase -= 360.0;
                    while (aBase < -180.0) aBase += 360.0;
                    
                    Gain -= GAIN_GAIN * signalStates[2];
                }

                txFeedbackPacket.robotArmX = (int16_t)targetX;
                txFeedbackPacket.robotArmY = (int16_t)targetY;

                Serial.print("🤖 Joysticks: ");
                for (int i = 0; i < 6; i++) { 
                    Serial.print("A");
                    Serial.print(i+1); Serial.print(":"); Serial.print(signalStates[i]); Serial.print(" "); 
                }
                
                Serial.print(" | 🔘 Buttons: ");
                for (int i = 0; i < 8; i++) {
                    Serial.print("B");
                    Serial.print(i+1); Serial.print(":"); Serial.print(decodedBtnOutputs[i]); Serial.print(" ");
                }

                Serial.print(" | Claw: ");
                Serial.print((btnClaw_raw == 0)? "open" : "close");
                Serial.print(" | Door: ");
                Serial.print((btnDoor_raw == 0)? "open" : "close");

                Serial.println("\n-------------------------------------------------------");
            } else {
                Serial.print("❌ Frame marker error! Head: ");
                Serial.print((char)rxBuffer[0]);
                Serial.print(" Tail: ");
                Serial.println((char)rxBuffer[23]); 

                bufferIndex = 0;
                while(Serial1.available() > 0) { Serial1.read(); } 
            }

            bufferIndex = 0;
        }
    }
}

// ==========================================
// 解析來自電腦鍵盤的指令 (Serial Monitor)
// ==========================================
void checkSerialMonitor() {
    if (Serial.available() > 0) {
        int c = Serial.read();
        if (c == '\n' || c == '\r' || c == ' ') {
            return;
        }

        if ((char)c == '+') {
            MAX_PWM_LIMIT += 10;
            if (MAX_PWM_LIMIT > 255) MAX_PWM_LIMIT = 255; 
            Serial.print("➕ 提高馬達最高速限制！目前 MAX_PWM_LIMIT = ");
            Serial.println(MAX_PWM_LIMIT);
            return;
        }
        else if ((char)c == '-') {
            MAX_PWM_LIMIT -= 10;
            if (MAX_PWM_LIMIT < 0) MAX_PWM_LIMIT = 0;     
            Serial.print("➖ 降低馬達最高速限制！目前 MAX_PWM_LIMIT = ");
            Serial.println(MAX_PWM_LIMIT);
            return;
        }

        char cmd = toupper((char)c);

        if (cmd == 'W') { targetZ += stepSize;
            Serial.println("W -> Z軸 上升 (Z+)"); }
        else if (cmd == 'S') { targetZ -= stepSize;
            Serial.println("S -> Z軸 下降 (Z-)"); }
        else if (cmd == 'A') { targetX -= stepSize;
            Serial.println("A -> X軸 內收 (X-)"); }
        else if (cmd == 'D') { targetX += stepSize;
            Serial.println("D -> X軸 外伸 (X+)"); }
        
        else if (cmd == 'Q') { targetY += stepSize;
            Serial.println("Q -> 基座向左轉 (Y+)"); }
        else if (cmd == 'E') { targetY -= stepSize;
            Serial.println("E -> 基座向右轉 (Y-)"); }
        
        else if (cmd == 'I') { 
            joyWalkY_raw = 4000;
            joyWalkX_raw = 2467; 
            Serial.println("I 🐾 -> 步足：前進"); 
        }
        else if (cmd == 'K') { 
            joyWalkY_raw = 0;
            joyWalkX_raw = 2467; 
            Serial.println("K 🐾 -> 步足：後退"); 
        }
        else if (cmd == 'J') { 
            joyWalkY_raw = 2467;
            joyWalkX_raw = 0;    
            Serial.println("J 🐾 -> 步足：左轉"); 
        }
        else if (cmd == 'L') { 
            joyWalkY_raw = 2467;
            joyWalkX_raw = 4000; 
            Serial.println("L 🐾 -> 步足：右轉"); 
        }
        else if (cmd == 'M') { 
            joyWalkY_raw = 2467;
            joyWalkX_raw = 2467; 
            Serial.println("M 🛑 -> 步足：停止"); 
        }

        if (cmd == 'C') { 
            btnClaw_raw = (btnClaw_raw == 0) ? 1 : 0;
            Serial.print("夾爪狀態切換: "); 
            Serial.println((btnClaw_raw == 0) ? "CLOSE" : "OPEN");
        }
        else if (cmd == 'X') { 
            eStop = true;
            Serial.println("!!! 🚨 緊急停止 🚨 !!!");
        }
        else if (cmd == 'R') { 
            eStop = false;
            Serial.println("♻️ 系統重置，恢復正常運作。");
        }
        else {
            Serial.print("未知的指令: ");
            Serial.println(cmd);
        }
    }
}

void initStates(){
    for (int i = 0; i < 6; i++) {
        signalStates[i] = 0;
    }
    // 同步清空運動增量
    walkForward = 0;
    walkTurn = 0;
}