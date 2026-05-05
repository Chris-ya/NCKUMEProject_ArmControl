#include <Servo.h> // 引入伺服馬達函式庫

Servo myServo;         // 建立一個伺服馬達物件
const int servoPin = 13; // 設定伺服馬達的控制腳位為 15

void setup() {
  Serial.begin(9600);       // 初始化序列埠通訊，鮑率設定為 9600
  myServo.attach(servoPin, 500, 2500);; // 將伺服馬達物件連接到指定腳位
  
  // 將馬達預設歸零
  myServo.write(0);
  
  Serial.println("=== 系統啟動 ===");
  Serial.println("請在上方輸入 0 到 180 的角度：");
}

void loop() {
  // 檢查是否有來自序列埠的資料
  if (Serial.available() > 0) {
    // 讀取輸入的字串，直到遇到換行符號
    String input = Serial.readStringUntil('\n');
    input.trim(); // 移除字串前後的隱藏空白或換行符號 (例如 \r)

    // 確保輸入不是空的
    if (input.length() > 0) {
      int angle = input.toInt(); // 將字串轉換為整數

      // 檢查輸入的角度是否在有效範圍內 (0 ~ 180度)
      if (angle >= 0 && angle <= 180) {
        myServo.write(angle); // 讓馬達轉動到指定角度
        
        Serial.print("✅ 已將馬達轉至: ");
        Serial.print(angle);
        Serial.println(" 度");
      } else {
        // 如果輸入的數字不在 0-180 之間，顯示錯誤訊息
        Serial.println("❌ 錯誤：請輸入 0 到 180 之間的數字！");
      }
    }
  }
}