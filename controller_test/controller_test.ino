#include <Arduino.h>

// ==========================================
// UART 腳位定義 (Pico 2W 的 Serial1)
// ==========================================
#define TX1 0
#define RX1 1

// ==========================================
// 📡 雙向通訊結構體定義 (與遙控器端完美對齊)
// ==========================================
// 接收端：24 位元組無線控制結構體
struct __attribute__((__packed__)) RobotControlPacket {
  char startMarker;           // 1 Byte
  uint8_t _padding1;          // 1 Byte
  uint16_t analogSignals[6];  // 12 Bytes 
  uint8_t btnState[8];        // 8 Bytes 
  char endMarker;             // 1 Byte
  uint8_t _padding2;          // 1 Byte
};

// 發送端：8 位元組回傳測試結構體
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

unsigned long lastByteIncomingTime = 0; // 斷線與超時防卡死計時
unsigned long lastSuccessTime = 0;      // 💡 用於精準計算遙控器發射頻率 (Hz)

// 獨立出來的回傳封包定時器 (半雙工分流：避免塞爆航道)
unsigned long lastTxFeedbackTime = 0;
const unsigned long FEEDBACK_INTERVAL = 50; // 50ms 對應 20 Hz 回傳頻率

void setup() {
  Serial.begin(115200); // 電腦除錯監控
  
  // 啟動與遙控器 HC-12 通訊的 UART1
  Serial1.setTX(TX1);  
  Serial1.setRX(RX1);
  Serial1.begin(9600);

  Serial.println("=============================================");
  Serial.println("🤖 Raspberry Pi Pico 2W 無線訊號測試診斷程式 🚀");
  Serial.println("=============================================");
}

void loop() {
  unsigned long currentTime = millis();

  // -----------------------------------------------------------------------
  // ⚡ 任務 1：接收與解析遙控器訊號，並精準計算發射頻率
  // -----------------------------------------------------------------------
  if (bufferIndex > 0 && (currentTime - lastByteIncomingTime > 100)) {
    bufferIndex = 0;
    while(Serial1.available() > 0) { Serial1.read(); }
    Serial.println("♻️ 拼包超時！已自動清空接收緩衝區。");
  }

  while (Serial1.available() > 0) {
    uint8_t incomingByte = Serial1.read();
    lastByteIncomingTime = currentTime; 

    // 強效對齊
    if (bufferIndex == 0 && incomingByte != '<') {
      continue;
    }

    rxBuffer[bufferIndex++] = incomingByte;
    
    // 當收滿 24 Bytes 進行解包與頻率計算
    if (bufferIndex >= 24) {
      if (rxBuffer[0] == '<' && rxBuffer[23] == '>') {
        
        // 💡 【精準頻率計算核心】
        if (lastSuccessTime > 0) {
          unsigned long packetInterval = currentTime - lastSuccessTime; // 計算兩次成功解包的時間差(ms)
          float frequency = 1000.0 / packetInterval;                    // 轉換為標準 Hz 頻率
          
          Serial.print("📊 遙控器發射頻率: ");
          Serial.print(frequency, 1);
          Serial.print(" Hz (間隔: ");
          Serial.print(packetInterval);
          Serial.print(" ms) | ");
        } else {
          Serial.print("✨ 成功接收第一筆封包! | ");
        }
        lastSuccessTime = currentTime; // 更新成功時間點

        // 🛠️ 記憶體對齊拆解 (24 Bytes 精準映射)
        for (int i = 0; i < 6; i++) {
          int baseIdx = 2 + (i * 2);
          rxPacket.analogSignals[i] = rxBuffer[baseIdx] | (rxBuffer[baseIdx + 1] << 8);
        }
        for (int i = 0; i < 8; i++) {
          rxPacket.btnState[i] = rxBuffer[14 + i];
        }

        // 印出實時數據驗證是否有錯位亂碼
        Serial.print("A1~A6:[");
        for(int i=0; i<6; i++) {
          Serial.print(rxPacket.analogSignals[i]);
          if(i<5) Serial.print("/");
        }
        Serial.print("] BTN:[");
        for(int i=0; i<3; i++) { // 先看前 3 顆接好的按鈕
          Serial.print(rxPacket.btnState[i]);
          if(i<2) Serial.print(",");
        }
        Serial.println("]");

      } else {
        Serial.print("❌ 封包頭尾校驗失敗! Head: ");
        Serial.print((char)rxBuffer[0]);
        Serial.print(" Tail: ");
        Serial.println((char)rxBuffer[23]);

        bufferIndex = 0;
        while(Serial1.available() > 0) { Serial1.read(); } 
      }

      bufferIndex = 0;
    }
  }

  // -----------------------------------------------------------------------
  // 📡 任務 2：定時回傳 8 Bytes 測試封包給遙控器 (20 Hz 非阻塞)
  // -----------------------------------------------------------------------
  if (currentTime - lastTxFeedbackTime >= FEEDBACK_INTERVAL) {
    lastTxFeedbackTime = currentTime;

    // 填入固定的測試用座標數據 (若通訊成功，遙控器螢幕會恆定顯示這組數值)
    txFeedbackPacket.robotArmX = 555;  
    txFeedbackPacket.robotArmY = -666; 

    // 噴出 8 位元組二進位資料回傳給遙控器
    Serial1.write((uint8_t*)&txFeedbackPacket, sizeof(txFeedbackPacket));
  }
}