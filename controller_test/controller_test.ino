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

// 斷線提示專用狀態旗標
bool remoteAlive = false;

// 用于偵測按鈕邊緣觸發的狀態追蹤陣列
uint8_t lastBtnStates[8] = {0, 0, 0, 0, 0, 0, 0, 0};

void setup() {
  Serial.begin(115200); 
  
  Serial1.setTX(TX1);  
  Serial1.setRX(RX1);
  Serial1.begin(9600);

  Serial.println("=============================================");
  Serial.println("🤖 Pico 2W 按鈕偵測與固定座標回傳測試程式  🚀");
  Serial.println("=============================================");
}

void loop() {
  unsigned long currentTime = millis();

  // -----------------------------------------------------------------------
  // ⚡ 任務 1：全非同步接收與滑動對齊解析遙控器訊號
  // -----------------------------------------------------------------------
  while (Serial1.available() > 0) {
    uint8_t incomingByte = Serial1.read();
    lastByteIncomingTime = currentTime;

    if (bufferIndex < 24) {
      rxBuffer[bufferIndex++] = incomingByte;
    }

    if (bufferIndex >= 24) {
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
        
        lastSuccessTime = currentTime; 
        remoteAlive = true;            

        // 二進位精準映射
        for (int i = 0; i < 6; i++) {
          int baseIdx = 2 + (i * 2);
          rxPacket.analogSignals[i] = rxBuffer[baseIdx] | (rxBuffer[baseIdx + 1] << 8);
        }
        for (int i = 0; i < 8; i++) {
          rxPacket.btnState[i] = rxBuffer[14 + i];
        }

        // 實時按鈕邊緣觸發偵測
        for (int i = 0; i < 7; i++) {
          if (rxPacket.btnState[i] == 1 && lastBtnStates[i] == 0) {
            Serial.print(" - [按鈕提示] 實體按鈕 B");
            Serial.print(i + 1);
            Serial.println(" 被按下了！💥");
          }
          lastBtnStates[i] = rxPacket.btnState[i];
        }

        // 輸出數據
        Serial.print("A1~A6:[");
        for(int i=0; i<6; i++) {
          Serial.print(rxPacket.analogSignals[i]);
          if(i<5) Serial.print("/");
        }
        Serial.print("] BTN:[");
        for(int i=0; i<7; i++) { 
          Serial.print(rxPacket.btnState[i]);
          if(i<6) Serial.print(",");
        }
        Serial.println("]");

        bufferIndex = 0; 

      } else {
        for (size_t i = 1; i < 24; i++) {
          rxBuffer[i - 1] = rxBuffer[i];
        }
        bufferIndex = 23; 
      }
    }
  }

  // -----------------------------------------------------------------------
  // 🚨 超時斷線提示功能 (心跳超時檢測)
  // -----------------------------------------------------------------------
  if (remoteAlive && (currentTime - lastSuccessTime > 500)) {
    remoteAlive = false; 
    Serial.println("\n🚨 [WARNING] 遙控器連線中斷！請檢查發射端電源或 HC-12 線路。");
    for (int i = 0; i < 8; i++) lastBtnStates[i] = 0;
  }

  // -----------------------------------------------------------------------
  // 📡 任務 2：定時回傳固定封包給遙控器 (20 Hz 非阻塞)
  // -----------------------------------------------------------------------
  if (currentTime - lastTxFeedbackTime >= FEEDBACK_INTERVAL) {
    lastTxFeedbackTime = currentTime;

    // 🎯 拿掉隨機，改回最安全的固定常數測試
    txFeedbackPacket.robotArmX = 555;  
    txFeedbackPacket.robotArmY = -666; 

    Serial1.write((uint8_t*)&txFeedbackPacket, sizeof(txFeedbackPacket));
  }
}
