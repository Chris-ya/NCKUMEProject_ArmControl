import serial
import keyboard
import time

COM_PORT = 'COM3'  
BAUD_RATE = 115200

print(f"嘗試連線至 {COM_PORT}...")
try:
    # 建立與 Pico 的序列埠連線
    arduino = serial.Serial(COM_PORT, BAUD_RATE)
    print("✅ 連線成功！")
except Exception as e:
    print(f"❌ 連線失敗: {e}")
    print("【必看提示】請確認 Arduino IDE 的 Serial Monitor (序列埠監控視窗) 已經關閉！")
    print("因為一個 COM 埠同時只能被一個程式佔用。")
    exit()

print("\n🕹️ 遊戲控制模式已啟動！")
print("===================================")
print("【W / S】       -> 控制 Y 軸")
print("【上 / 下方向鍵】-> 控制 Z 軸")
print("【左 / 右方向鍵】-> 控制 X 軸")
print("【C】切換夾爪 | 【E】急停 | 【R】解除急停")
print("【ESC】退出虛擬遙控器")
print("===================================")

# 用來記錄單次觸發按鍵的上一次狀態 (避免長按狂送訊號)
last_c = False
last_e = False
last_r = False

while True:
    # 1. 退出機制
    if keyboard.is_pressed('esc'):
        print("🛑 結束控制")
        arduino.close()
        break

    # 2. 長按連發機制 (移動控制)
    if keyboard.is_pressed('w'):
        arduino.write(b'W')
    if keyboard.is_pressed('s'):
        arduino.write(b'S')
        
    if keyboard.is_pressed('up'):
        arduino.write(b'\x1b[A') # 模擬你 Pico 寫的 ANSI 上方向鍵
    if keyboard.is_pressed('down'):
        arduino.write(b'\x1b[B') # 模擬 ANSI 下方向鍵
    if keyboard.is_pressed('right'):
        arduino.write(b'\x1b[C') # 模擬 ANSI 右方向鍵
    if keyboard.is_pressed('left'):
        arduino.write(b'\x1b[D') # 模擬 ANSI 左方向鍵

    # 3. 單次觸發機制 (狀態切換控制)
    current_c = keyboard.is_pressed('c')
    if current_c and not last_c:
        arduino.write(b'C')
    last_c = current_c

    current_e = keyboard.is_pressed('e')
    if current_e and not last_e:
        arduino.write(b'E')
    last_e = current_e

    current_r = keyboard.is_pressed('r')
    if current_r and not last_r:
        arduino.write(b'R')
    last_r = current_r

    # 4. 配合 Pico 端的 CONTROL_LOOP_DELAY，給予 20 毫秒的延遲，避免塞爆通訊埠
    time.sleep(0.02)