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
// 🎯 完美對齊遙控器端：更新為 24 位元組二進位控制結構體
struct __attribute__((__packed__)) RobotControlPacket {
  char startMarker;           // 1 Byte
  uint8_t _padding1;          // 1 Byte (對齊填充)
  uint16_t analogSignals[6];  // 12 Bytes 
  uint8_t btnState[8];        // 8 Bytes 
  char endMarker;             // 1 Byte
  uint8_t _padding2;          // 1 Byte (湊滿 24 Bytes)
};

// 🎯 新增：發送給遙控器專用的 8 位元組回傳結構體
struct __attribute__((__packed__)) RobotFeedbackPacket {
  char startMarker = '[';     // 1 Byte (回傳開頭標記)
  int16_t robotArmX;          // 2 Bytes (回傳手臂實體 targetX)
  int16_t robotArmY;          // 2 Bytes (回傳手臂實體 targetY)
  char endMarker = ']';       // 1 Byte (回傳結尾標記)
  uint16_t _padding = 0;      // 2 Bytes (記憶體對齊填充)
};

RobotControlPacket rxPacket;
const size_t packetSize = sizeof(RobotControlPacket); // 💡 自動更新為 24 Bytes
uint8_t rxBuffer[24];                                 // 💡 實體緩衝區同步改為 24 Bytes
size_t bufferIndex = 0;

RobotFeedbackPacket txFeedbackPacket;

// 頻率量測專用變數
unsigned long lastSuccessTime = 0; // 記錄上一次成功解包的時間 (ms)
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
    // 啟動與搖桿通訊的 UART1 (波特率 9600)
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
        
        // 💡 當收集滿了 24 Bytes
        if (bufferIndex >= 24) {
            
            // 將緩衝區的記憶體安全複製到結構體物件中
            memcpy(&rxPacket, rxBuffer, packetSize);

            // 💡 配合 24 節奏，校驗頭（rxBuffer[0]）與尾（rxBuffer[23]）
            if (rxPacket.startMarker == '<' && rxPacket.endMarker == '>') {
                
                unsigned long currentTime = millis();
                lastPacketTime = currentTime; // 刷新手臂與馬達活著的時間
                /*
                // -----------------------------------------------------------
                // 📊 計算並印出實際發射頻率
                // -----------------------------------------------------------
                if (lastSuccessTime > 0) { 
                packetInterval = currentTime - lastSuccessTime;
                Serial.print("📡 [TEST] Packet Interval: ");
                Serial.print(packetInterval);
                Serial.print(" ms (實際發射頻率: ");
                Serial.print(1000.0 / packetInterval, 1);
                Serial.println(" Hz)");
                }*/
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

                // 2. 拆解原本的 joyX_raw 與 joyY_raw (對齊新位址相容舊程式)
                joyX_raw = rxPacket.analogSignals[0];
                joyY_raw = rxPacket.analogSignals[1];
                
                for (int i = 0; i < 8; i++) {
                    uint8_t currentBtnState = rxPacket.btnState[i];

                    // 偵測「按下瞬間」（從放開 1 變成 按下 0）
                    if (currentBtnState == 0 && lastBtnStates[i] == 1) {
                        // 觸發狀態反轉 (0 變 1, 1 變 0)
                        decodedBtnOutputs[i] = (decodedBtnOutputs[i] == 0) ? 1 : 0;
                        
                        // 特殊相容：如果是第一個按鈕 (索引0)，同步更新你的舊變數 btnClaw_raw
                        if (i == 0) {
                            btnClaw_raw = decodedBtnOutputs[0];
                            }
                        else if (i == 4) {
                            btnDoor_raw = decodedBtnOutputs[4];
                            } 
                    }
                    
                // 更新歷史狀態，留給下一次循環比較
                lastBtnStates[i] = currentBtnState;
                }

                if (decodedBtnOutputs[1] == 1) {
                    decodedBtnOutputs[1] = 0;
                    resetposition = true;
                }

                // -----------------------------------------------------------
                // 執行動作控制分配
                // -----------------------------------------------------------
                walkForward = MAX_PWM_LIMIT * signalStates[5];
                walkTurn = MAX_PWM_LIMIT * signalStates[4];
                targetX += MAX_VELOCITY * signalStates[0] * 4;
                targetZ -= MAX_VELOCITY * signalStates[3] * 4;
                aBase -= GAIN_GAIN * signalStates[1];
                Gain -= GAIN_GAIN * signalStates[2];

                // -----------------------------------------------------------
                // 🎯 新增：解包成功後，閃電般非同步回傳主機座標與心跳給遙控器
                // -----------------------------------------------------------
                txFeedbackPacket.robotArmX = (int16_t)targetX; // 讀取全域變數並強制轉型
                txFeedbackPacket.robotArmY = (int16_t)targetY; // 讀取全域變數並強制轉型
                
                // 噴出 8 數位組二進位資料回 HC-12
                // Serial1.write((uint8_t*)&txFeedbackPacket, sizeof(txFeedbackPacket));

                // -----------------------------------------------------------
                // 🛠️ 乾淨完整的 Debug 狀態輸出 (包含 6搖桿 + 8按鈕)
                // -----------------------------------------------------------
                Serial.print("🤖 Joysticks: ");
                for (int i = 0; i < 6; i++) { 
                Serial.print("A"); Serial.print(i+1); Serial.print(":"); Serial.print(signalStates[i]); Serial.print(" "); 
                }
                
                Serial.print(" | 🔘 Buttons: ");
                for (int i = 0; i < 8; i++) {
                Serial.print("B"); Serial.print(i+1); Serial.print(":"); Serial.print(decodedBtnOutputs[i]); Serial.print(" ");
                }

                Serial.print(" | Claw: ");
                Serial.print((btnClaw_raw == 0)? "open" : "close");
                Serial.print(" | Door: ");
                Serial.print((btnDoor_raw == 0)? "open" : "close");

                Serial.println("\n-------------------------------------------------------");

            } else {
                // 如果頭尾不對，印出收到的錯誤標記到底是甚麼，方便 debug
                Serial.print("❌ Frame marker error! Head: ");
                Serial.print((char)rxBuffer[0]);
                Serial.print(" Tail: ");
                Serial.println((char)rxBuffer[23]); // 💡 同步更新為索引 23

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

        char bufferText[12];
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