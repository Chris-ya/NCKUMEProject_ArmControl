#include "Config.h"

// ==========================================
// UART 腳位與全域變數定義
// ==========================================
#define TX1 0
#define RX1 1

float CoorSen = 1;
String rxString = "";
float stepSize = 10.0;
// 初始化搖桿預設值 (置中)
volatile int joyX_raw = 2467; 
volatile int joyY_raw = 2467; 
volatile int joyWalkX_raw = 2467; // 負責步足左右轉向 (預設置中)
volatile int joyWalkY_raw = 2467; // 負責步足前進後退 (預設置中)
volatile int btnClaw_raw = 1;

uint8_t lastClawBtnState = 1;
uint8_t lastStopBtnState = 1;
uint8_t currentClawBtnState = 0;

// ==========================================
// 無線通訊封包結構與緩衝區定義
// ==========================================
struct __attribute__((__packed__)) RobotControlPacket {
  char startMarker;           
  uint16_t analogSignals[6];  
  uint8_t btnState[8];        
  char endMarker;             
};

RobotControlPacket rxPacket;
const size_t packetSize = sizeof(RobotControlPacket); // 22 Bytes
uint8_t rxBuffer[22];                                // 接收實體緩衝區
size_t bufferIndex = 0;                              // 緩衝區索引指針

// 頻率量測專用變數
unsigned long lastSuccessTime = 0; // 記錄上一次成功解包的時間 (ms)
unsigned long packetInterval = 0;  // 兩次封包之間的時間差 (ms)

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
    
    // 啟動與搖桿通訊的 UART1 (波特率 9600)
    Serial1.setTX(TX1);  
    Serial1.setRX(RX1);
    Serial1.begin(9600); 
}

// ==========================================
// 解析來自無線搖桿的訊號 (UART1)
// ==========================================
int signalStates[6] = {0, 0, 0, 0, 0, 0};

unsigned long lastByteIncomingTime = 0; // 新增：記錄上一次實體字元進來的時間

void checkUART() {
    // 【防卡死安全鎖】超過 100ms 沒拼完一包就重置
    if (bufferIndex > 0 && (millis() - lastByteIncomingTime > 100)) {
        bufferIndex = 0;
        while(Serial1.available() > 0) { Serial1.read(); }
        Serial.println("♻️ Packet assembly timeout! Serial buffer auto-flushed.");
    }

    while (Serial1.available() > 0) {
        uint8_t incomingByte = Serial1.read();
        lastByteIncomingTime = millis(); 

        // 強效對齊：第一根 Byte 必須是 '<'
        if (bufferIndex == 0 && incomingByte != '<') {
            continue; 
        }

        rxBuffer[bufferIndex++] = incomingByte;

        // 當收集滿了 22 Bytes
        if (bufferIndex >= 22) {
            
            // 【核心修正】改用絕對陣列索引校驗頭尾標記
            // rxBuffer[0] 是開頭，rxBuffer[21] 是第 22 個 Byte (結尾)
            if (rxBuffer[0] == '<' && rxBuffer[21] == '>') {
                
                unsigned long currentTime = millis();
                lastPacketTime = currentTime; // 刷新手臂與馬達活著的時間

                // -----------------------------------------------------------
                // 🛠️ 絕對陣列手動解包 (22 Bytes 精準拆解)
                // -----------------------------------------------------------
                // 1. 拆解 6 個 analogSignals (每個 2 Bytes，高低位元組合)
                // 根據結構：rxBuffer[0] 是 '<'
                // A1: rxBuffer[1](低) + rxBuffer[2](高)
                // A2: rxBuffer[3](低) + rxBuffer[4](高) ... 依此類推
                for (int i = 0; i < 6; i++) {
                    int baseIdx = 1 + (i * 2);
                    int currentVal = rxBuffer[baseIdx] | (rxBuffer[baseIdx + 1] << 8);

                    // 直接進行死區與訊號狀態判斷 (500 ~ 700)
                    if (currentVal >= 500 && currentVal <= 700) {
                        signalStates[i] = 0; 
                    } else if (currentVal > 700) {
                        signalStates[i] = 1; 
                    } else {
                        signalStates[i] = -1; 
                    }
                }

                // 2. 拆解原本的 joyX_raw 與 joyY_raw (相容舊程式)
                joyX_raw = rxBuffer[1] | (rxBuffer[2] << 8);
                joyY_raw = rxBuffer[3] | (rxBuffer[4] << 8);

                // 3. 拆解按鈕狀態 btnState[8] (從第 13 個 Byte 開始，共 8 個 Byte)
                // rxBuffer[13] 是 btnState[0]
                currentClawBtnState = rxBuffer[13];
                if (isButtonPressed(currentClawBtnState, lastClawBtnState)) {
                    btnClaw_raw = (btnClaw_raw == 0) ? 1 : 0;
                }

                // -----------------------------------------------------------
                // 執行動作控制分配 (與你原本的控制邏輯相同)
                // -----------------------------------------------------------
                walkForward = MAX_PWM_LIMIT * signalStates[5];
                walkTurn = MAX_PWM_LIMIT * signalStates[4];
                targetX += MAX_VELOCITY * signalStates[1] * 4;
                targetY += MAX_VELOCITY * signalStates[0] * 4;
                targetZ -= MAX_VELOCITY * signalStates[3] * 4;

                // 頻率 Debug 輸出 (如果需要看時間差，可以解除註解)
                // Serial.println(millis() - lastPacketTime);

            } else {
                // 如果頭尾不對，印出收到的錯誤標記到底是甚麼，方便 debug
                Serial.print("❌ Frame marker error! Head: ");
                Serial.print((char)rxBuffer[0]);
                Serial.print(" Tail: ");
                Serial.println((char)rxBuffer[21]);

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

        if (cmd == 'W') { targetZ += stepSize; Serial.println("W -> Z軸 上升 (Z+)"); }
        else if (cmd == 'S') { targetZ -= stepSize; Serial.println("S -> Z軸 下降 (Z-)"); }
        else if (cmd == 'A') { targetX -= stepSize; Serial.println("A -> X軸 內收 (X-)"); }
        else if (cmd == 'D') { targetX += stepSize; Serial.println("D -> X軸 外伸 (X+)"); }
        
        else if (cmd == 'Q') { targetY += stepSize; Serial.println("Q -> 基座向左轉 (Y+)"); }
        else if (cmd == 'E') { targetY -= stepSize; Serial.println("E -> 基座向右轉 (Y-)"); }
        
        else if (cmd == 'I') { 
            joyWalkY_raw = 4000; joyWalkX_raw = 2467; 
            Serial.println("I 🐾 -> 步足：前進"); 
        }
        else if (cmd == 'K') { 
            joyWalkY_raw = 0;    joyWalkX_raw = 2467; 
            Serial.println("K 🐾 -> 步足：後退"); 
        }
        else if (cmd == 'J') { 
            joyWalkY_raw = 2467; joyWalkX_raw = 0;    
            Serial.println("J 🐾 -> 步足：左轉"); 
        }
        else if (cmd == 'L') { 
            joyWalkY_raw = 2467; joyWalkX_raw = 4000; 
            Serial.println("L 🐾 -> 步足：右轉"); 
        }
        else if (cmd == 'M') { 
            joyWalkY_raw = 2467; joyWalkX_raw = 2467; 
            Serial.println("M 🛑 -> 步足：停止"); 
        }

        else if (cmd == 'C') { 
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