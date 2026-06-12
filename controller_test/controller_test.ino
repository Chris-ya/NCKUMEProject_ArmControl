#include <Arduino.h>

#define TX1 0
#define RX1 1

// ==========================================
// 📡 雙向通訊結構體定義 (與遙控器端完美對齊)
// ==========================================
struct __attribute__((__packed__)) RobotControlPacket {
  char startMarker;           // 1 Byte
  uint8_t _padding1;          // 1 Byte
  uint16_t analogSignals[6];  // 12 Bytes 
  uint8_t btnState[8];        // 8 Bytes 
  char endMarker;             // 1 Byte
  uint8_t _padding2;          // 1 Byte
};

struct __attribute__((__packed__)) RobotFeedbackPacket {
  char startMarker = '[';     // 1 Byte
  int16_t robotArmX;          // 2 Bytes
  int16_t robotArmY;          // 2 Bytes
  char endMarker = ']';       // 1 Byte
  uint16_t _padding = 0;      // 2 Bytes
};

RobotControlPacket rxPacket;
RobotFeedbackPacket txFeedbackPacket;

// ==========================================
// 通訊緩衝區與計時器變數
// ==========================================
uint8_t rxBuffer[24];
size_t bufferIndex = 0;

unsigned long lastByteIncomingTime = 0; 
unsigned long lastSuccessTime = 0;      // 用於計算 Hz 與檢測斷線

// 獨立出來的回傳封包定時器 (20 Hz)
unsigned long lastTxFeedbackTime = 0;
const unsigned long FEEDBACK_INTERVAL = 50; 

// 🎯 斷線提示專用狀態旗標
bool remoteAlive = false;

void setup() {
  Serial.begin(115200); 
  
  Serial1.setTX(TX1);  
  Serial1.setRX(RX1);
  Serial1.begin(9600);

  Serial.println("=============================================");
  Serial.println("🤖 Pico 2W 終極滑動對齊與雙向通訊測試診斷程式 🚀");
  Serial.println("=============================================");
}

void loop() {
  unsigned long currentTime = millis();

  // -----------------------------------------------------------------------
  // ⚡ 任務 1：接收與強效對齊解析遙控器訊號
  // -----------------------------------------------------------------------
  while (Serial1.available() > 0) {
    uint8_t incomingByte = Serial1.read();
    lastByteIncomingTime = currentTime;

    // 將收到的字元依序塞入緩衝區
    if (bufferIndex < 24) {
      rxBuffer[bufferIndex++] = incomingByte;
    }

    // 當收集滿 24 Bytes，啟動強效對齊檢驗
    if (bufferIndex >= 24) {
      
      // 💡 只有當頭尾同時完全符合，才算真正解包成功！
      if (rxBuffer[0] == '<' && rxBuffer[23] == '>') {
        
        // 精準頻率計算
        if (lastSuccessTime > 0) {
          unsigned long packetInterval = currentTime - lastSuccessTime;
          float frequency = 1000.0 / packetInterval;
          Serial.print("📊 遙控器發射頻率: ");
          Serial.print(frequency, 1);
          Serial.print(" Hz | ");
        } else {
          Serial.print("✨ 首次連線成功! | ");
        }
        
        lastSuccessTime = currentTime; // 刷新成功解包時間點
        remoteAlive = true;            // 標記遙控器目前活著

        // 二進位精準映射至結構體欄位
        for (int i = 0; i < 6; i++) {
          int baseIdx = 2 + (i * 2);
          rxPacket.analogSignals[i] = rxBuffer[baseIdx] | (rxBuffer[baseIdx + 1] << 8);
        }
        for (int i = 0; i < 8; i++) {
          rxPacket.btnState[i] = rxBuffer[14 + i];
        }

        // 輸出乾淨數據供開發排查
        Serial.print("A1~A6:[");
        for(int i=0; i<6; i++) {
          Serial.print(rxPacket.analogSignals[i]);
          if(i<5) Serial.print("/");
        }
        Serial.print("] BTN:[");
        for(int i=0; i<3; i++) { 
          Serial.print(rxPacket.btnState[i]);
          if(i<2) Serial.print(",");
        }
        Serial.println("]");

        bufferIndex = 0; // 完美解包，清空緩衝區等待下一包

      } else {
        // 💡 【核心修正：滑動視窗對齊】
        // 如果頭尾不吻合，代表我們抓錯開頭了！
        // 將整個緩衝區的資料集體往前挪移 1 Byte，拋棄最前面的髒字元，尋找真正的對齊點
        for (size_t i = 1; i < 24; i++) {
          rxBuffer[i - 1] = rxBuffer[i];
        }
        bufferIndex = 23; // 指標退回 23，讓 loop 下一次進來的 Byte 填補在最後一個位置
      }
    }
  }

  // -----------------------------------------------------------------------
  // 🚨 🎯 新增：超時斷線提示功能 (心跳超時檢測)
  // -----------------------------------------------------------------------
  // 如果原本是連線狀態，但超過 500ms 沒能成功解開任何一包正確封包
  if (remoteAlive && (currentTime - lastSuccessTime > 500)) {
    remoteAlive = false; // 變更狀態為斷線
    Serial.println("\n🚨 [WARNING] 遙控器連線中斷！請檢查發射端電源或 HC-12 線路。");
  }

  // -----------------------------------------------------------------------
  // 📡 任務 2：定時回傳 8 Bytes 測試封包給遙控器 (20 Hz 非阻塞)
  // -----------------------------------------------------------------------
  if (currentTime - lastTxFeedbackTime >= FEEDBACK_INTERVAL) {
    lastTxFeedbackTime = currentTime;

    // 固定回傳測試座標
    txFeedbackPacket.robotArmX = 555;  
    txFeedbackPacket.robotArmY = -666; 

    Serial1.write((uint8_t*)&txFeedbackPacket, sizeof(txFeedbackPacket));
  }
}