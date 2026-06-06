#pragma once
#include <stdint.h>
#include "driver/ledc.h"

/* PWM 蜂鸣器 (无源蜂鸣器, LEDC 驱动) */

/* ---- 硬件引脚 (请填写) ---- */
#define BUZZER_PIN                  47
#define BUZZER_LEDC_CHANNEL         LEDC_CHANNEL_5
#define BUZZER_LEDC_TIMER           LEDC_TIMER_2

#define BUZZER_DEFAULT_FREQ         2000    /* Hz */
#define BUZZER_DEFAULT_VOLUME       50      /* 0~100% */

/* 音符频率 (Hz) */
enum Note {
    NOTE_C4=262,  NOTE_D4=294,  NOTE_E4=330,  NOTE_F4=349,
    NOTE_G4=392,  NOTE_A4=440,  NOTE_B4=494,  NOTE_C5=523,
    NOTE_D5=587,  NOTE_E5=659,  NOTE_F5=698,  NOTE_G5=784,
    NOTE_A5=880,  NOTE_B5=988,  NOTE_C6=1047,
    NOTE_REST=0,                                      /* 休止 */
};

/* 音符时长 (ms, 以四分音符=400ms 为基准) */
#define NOTE_WHOLE                  1600
#define NOTE_HALF                   800
#define NOTE_QUARTER                400
#define NOTE_EIGHTH                 200
#define NOTE_SIXTEENTH              100


class Buzzer {
public:
    Buzzer(uint8_t pin = BUZZER_PIN,
           uint8_t ledc_channel = BUZZER_LEDC_CHANNEL,
           uint8_t ledc_timer = BUZZER_LEDC_TIMER);

    void init();

    /* 播放 */
    void tone(uint16_t freq_hz, uint16_t duration_ms);
    void toneStart(uint16_t freq_hz);
    void toneStop();
    void beep(uint16_t duration_ms = 100);

    /* 旋律: Note 数组, {freq, duration} 交替 */
    void playMelody(const uint16_t *notes, uint16_t count);

    /* 音量 0~100 */
    void setVolume(uint8_t vol);

private:
    uint8_t _pin;
    uint8_t _ch;
    uint8_t _timer;
    uint8_t _volume;
};
