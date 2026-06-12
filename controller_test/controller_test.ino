#include <Wire.h>

// ===================================================================
// 1. 定義與發送端完全相同的緊密結構體 (Packed Structure)
// ===================================================================
struct __attribute__((__packed__)) RobotControlPacket {
  char startMarker;           // 起始標記 (預設應為 '<')
  uint8_t _padding1;
  uint16_t analogSignals[6];  // 6個類比訊號 (每個 2 Byte)
  uint8_t btnState[8];        // 8個按鈕狀態 (每個 1 Byte)
  char endMarker;             // 結束標記 (預設應為 '>')
  uint8_t _padding2;
};

// ===================================================================
// 2. 全域變數宣告
// ===================================================================
RobotControlPacket rxPacket;
const size_t packetSize = sizeof(RobotControlPacket); // 固定的封包總長度：22 Bytes
uint8_t buffer[24];
size_t bufferIndex = 0;

// 解包後的數據儲存區
int decodedSignals[6];                                       // 儲存 6 通道類比原始值 (int)
char armSignalStates[6] = {'O', 'O', 'O', 'O', 'O', 'O'};   // 儲存 6 通道死區狀態 ('O', '+', '-')

// 舊邏輯相容變數
int joyX_raw = 512;
int joyY_raw = 512;
uint8_t currentClawBtnState = 0;
uint8_t lastClawBtnState = 0;
int btnClaw_raw = 0;

// 頻率量測與安全防護變數
unsigned long lastSuccessTime = 0; // 記錄上一次成功解包的時間 (ms)，用來算頻率
unsigned long packetInterval = 0;  // 兩次封包之間的時間差 (ms)
unsigned long lastPacketTime = 0;  // 供 Failsafe 斷線防護計時使用 (ms)

// ===================================================================
// 3. 初始化設定 (Setup)
// ===================================================================
void setup() {
  // 初始化 USB 序列埠（電腦序列埠監視器）
  Serial.begin(115200);
  
  // 強制等待，直到你在電腦上打開序列埠監視器，確保開頭提示與數據不會漏看
  while (!Serial) { 
    delay(10); 
  }
  delay(500);

  Serial.println("\n=======================================================");
  Serial.println("--- Pico 2 W Ready: Packet Decoder & Frequency Tester ---");
  Serial.print("-> Expected Packet Size: "); Serial.print(packetSize); Serial.println(" bytes");
  Serial.println("-> Listening on UART0 (Serial1): TX=GPIO0, RX=GPIO1");
  Serial.println("=======================================================\n");

  // 初始化 HC-12 通訊埠 (強制宣告使用 Pico 2 W 的 UART0 穩定腳位)
  Serial1.setTX(0);
  Serial1.setRX(1);
  Serial1.begin(9600); // 速率需與您的 HC-12 通訊速率對齊
}

// ===================================================================
// 4. 主程式循環 (Loop)
// ===================================================================
void loop() {
  // 呼叫序列埠檢查、防錯對齊與解包
  checkUART();

  // -----------------------------------------------------------------
  // 斷線安全防護機制 (Fail-safe)
  // 如果當前時間距離上次成功收到封包的時間超過 500ms，代表遙控器關機或斷訊
  // -----------------------------------------------------------------
  if (millis() - lastPacketTime > 500) {
    // 強制將 6 個搖桿狀態全部重設為 'O' (死區/置中/停止)
    for (int i = 0; i < 6; i++) {
      armSignalStates[i] = 'O';
    }
    // 原始值重置
    joyX_raw = 512;
    joyY_raw = 512;

    // 每秒在畫面上印出一次警告，提醒目前處於斷線剎車狀態
    static unsigned long lastWarningTime = 0;
    if (millis() - lastWarningTime > 1000) {
      Serial.println("⚠️ WARNING: Remote Controller Disconnected! Failsafe Activated (STOP).");
      lastWarningTime = millis();
    }
  }

  // -----------------------------------------------------------------
  // 🤖 驅動馬達或機器人動作的程式碼請寫在下方：
  // 這裡可以直接讀取 armSignalStates[0] 到 armSignalStates[5]
  // 範例：
  // if (armSignalStates[5] == '+') { 前進(); }
  // -----------------------------------------------------------------
}

// ===================================================================
// 5. 序列埠讀取、強效校正與解包函式
// ===================================================================
void checkUART() {
  // 當 Serial1 有任何無線位元組進來時
  while (Serial1.available() > 0) {
    uint8_t incomingByte = Serial1.read();

    // 【強效對齊修正】如果緩衝區目前是空的，進來的第一個位元組必須是起始標記 '<'
    if (bufferIndex == 0 && incomingByte != '<') {
      continue; // 如果不是 '<'，直接拋棄，繼續等下一個位元組
    }

    // 符合對齊條件或是後續的數據，才寫入緩衝區
    buffer[bufferIndex++] = incomingByte;

    // 當收集滿了完整的封包長度 (22 Bytes)
    if (bufferIndex >= packetSize) {
      
      // 將緩衝區的記憶體安全複製到結構體物件中
      memcpy(&rxPacket, buffer, packetSize);

      // 【雙重校驗】檢查開頭與結尾是否確實為標記字元
      if (rxPacket.startMarker == '<' && rxPacket.endMarker == '>') {
        
        unsigned long currentTime = millis();
        lastPacketTime = currentTime; // 更新 Failsafe 時間戳記

        // -----------------------------------------------------------
        // 📊 計時核心：計算並印出實際發射頻率
        // -----------------------------------------------------------
        if (lastSuccessTime > 0) { 
          packetInterval = currentTime - lastSuccessTime;
          Serial.print("📡 [TEST] Packet Interval: ");
          Serial.print(packetInterval);
          Serial.print(" ms (實際發射頻率: ");
          Serial.print(1000.0 / packetInterval, 1);
          Serial.println(" Hz)");
        }
        lastSuccessTime = currentTime; // 更新成功時間戳記

        // -----------------------------------------------------------
        // 數據解析與死區 (500~700) 狀態轉換
        // -----------------------------------------------------------
        // 更新相容舊程式的原始變數
        joyX_raw = rxPacket.analogSignals[0];
        joyY_raw = rxPacket.analogSignals[1];
        
        // 模擬原本的按鈕判斷切換邏輯
        currentClawBtnState = rxPacket.btnState[0];
        if (currentClawBtnState != lastClawBtnState) {
          if (currentClawBtnState == 1) { // 假設高電位代表按下
            btnClaw_raw = (btnClaw_raw == 0) ? 1 : 0;
          }
          lastClawBtnState = currentClawBtnState;
        }

        // 核心修正：將 6 個 uint16_t 解包並轉換為 500~700 死區狀態
        for (int i = 0; i < 6; i++) {
          decodedSignals[i] = (int)rxPacket.analogSignals[i];

          // 判斷死區區間
          if (decodedSignals[i] >= 500 && decodedSignals[i] <= 700) {
            armSignalStates[i] = 'O'; // 在死區內 (停止)
          } else if (decodedSignals[i] > 700) {
            armSignalStates[i] = '+'; // 正向觸發
          } else {
            armSignalStates[i] = '-'; // 負向觸發
          }
        }

        // 乾淨的 Debug 輸出：顯示目前 6 個通道被歸類後的控制命令
        Serial.print("🤖 Arm States -> ");
        for (int i = 0; i < 6; i++) {
          Serial.print("A"); Serial.print(i + 1); Serial.print(":");
          Serial.print(armSignalStates[i]);
          Serial.print("  ");
        }
        Serial.println();
        Serial.println("-------------------------------------------------------");

      } else {
        // 頭尾標記不對，代表中途有雜訊漏包造成錯位
        Serial.println("⚠️ Packet alignment error detected! Dropping corrupt data.");
      }

      // 處理完畢（無論成功或失敗），清空緩衝區索引，重新等待下一個明確的 '<'
      bufferIndex = 0;
    }
  }
}
