// Bluetooth.ino
#include <SerialBT.h>

extern float targetX;
extern float targetY;
extern float targetZ;

void setupBluetooth() {
    // Pico 2W 的 SerialBT 預期接收鮑率 (Baud rate)，我們設定為 115200
    SerialBT.begin(115200); 
    Serial.println("Bluetooth initialized. Waiting for connection...");
}
void checkBluetooth() {
    if (SerialBT.available()) {
        String cmd = SerialBT.readStringUntil('\n');
        cmd.trim();

        // 預期格式: "X:100,Y:50,Z:80"
        int firstComma = cmd.indexOf(',');
        int secondComma = cmd.lastIndexOf(',');

        if (firstComma > 0 && secondComma > firstComma) {
            String strX = cmd.substring(0, firstComma);
            String strY = cmd.substring(firstComma + 1, secondComma);
            String strZ = cmd.substring(secondComma + 1);

            if (strX.startsWith("X:") && strY.startsWith("Y:") && strZ.startsWith("Z:")) {
                targetX = strX.substring(2).toFloat();
                targetY = strY.substring(2).toFloat();
                targetZ = strZ.substring(2).toFloat();

                Serial.print("Target -> X: ");
                Serial.print(targetX);
                Serial.print(" / Y: ");
                Serial.print(targetY);
                Serial.print(" / Z: ");
                Serial.println(targetZ);
            }
        }
    }
}