#include "Config.h"

// =========================================================================
// 音符頻率定義 (Hz)
// =========================================================================
#define REST      0
#define NOTE_G3  196
#define NOTE_A3  220
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_E5  659
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_A5  880

// =========================================================================
// 歌曲資料區
// =========================================================================
const int tempo = 140; 

int melody[] = {
  NOTE_D4, NOTE_E4, NOTE_FS4, NOTE_G4, 
  NOTE_A4, NOTE_A4, NOTE_B4, NOTE_A4, NOTE_G4, NOTE_FS4,
  NOTE_D4, NOTE_E4, NOTE_FS4, NOTE_G4,
  NOTE_A4, NOTE_A4, NOTE_B4, NOTE_A4, REST
};

int durations[] = {
  8, 8, 8, 8,
  4, 8, 8, 4, 8, 8,
  8, 8, 8, 8,
  4, 8, 8, 2, 4
};

// =========================================================================
// 狀態機變數 (非阻塞核心)
// =========================================================================
bool isPlaying = false;
int currentNote = 0;
unsigned long noteStartTime = 0;
int currentNoteDuration = 0;
bool isResting = false; 

int notesCount = sizeof(melody) / sizeof(melody[0]);
int wholenote = (60000 * 4) / tempo;

// =========================================================================
// 播放控制函式
// =========================================================================

// 啟動音樂 (只負責標記開始，不卡死程式)
void startHaruhikage() {
    if (!isPlaying) {
        Serial.println("🎸 為什麼要演奏春日影！！ (背景播放啟動)");
        isPlaying = true;
        currentNote = 0;
        isResting = false;
        playCurrentNote(); // 立即發出第一個音
    }
}

// 停止音樂
void stopMusic() {
    isPlaying = false;
    noTone(SPEAKER);
}

// 負責計算並發出「當下這個音」
void playCurrentNote() {
    if (currentNote >= notesCount) {
        stopMusic();
        Serial.println("🎵 播放結束。");
        return;
    }

    int divider = durations[currentNote];
    if (divider > 0) {
        currentNoteDuration = wholenote / divider;
    } else if (divider < 0) {
        currentNoteDuration = (wholenote / abs(divider)) * 1.5;
    }

    if (melody[currentNote] != REST) {
        tone(SPEAKER, melody[currentNote]);
    } else {
        noTone(SPEAKER);
    }
    
    noteStartTime = millis();
    isResting = false;
}

// 🎯 最關鍵的更新函式：每次迴圈不斷檢查是否該換音符
void updateMusic() {
    if (!isPlaying) return;

    if (eStop) {
        stopMusic();
        return;
    }

    unsigned long currentMillis = millis();

    // 階段 1：處理防黏音 (音符播放時間達 90% 時先靜音)
    if (!isResting && (currentMillis - noteStartTime >= currentNoteDuration * 0.9)) {
        noTone(SPEAKER);
        isResting = true;
    }

    // 階段 2：整個音符週期結束，推進到下一個音
    if (currentMillis - noteStartTime >= currentNoteDuration) {
        currentNote++;
        playCurrentNote();
    }
}