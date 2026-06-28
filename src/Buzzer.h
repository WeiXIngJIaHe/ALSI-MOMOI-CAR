#pragma once
#include <stdint.h>
#include "driver/ledc.h"
#include "driver/gpio.h"

/* ================================================================
 * 无源蜂鸣器 — LEDC 固定频率 + 状态机
 *
 * 引脚定义:
 *   GPIO1 → 无源蜂鸣器 (2.7kHz 方波驱动)
 *   GPIO2 → 启动按钮 (低电平有效, 内部上拉, 边沿触发)
 *   GPIO3 → 停止按钮 (低电平有效, 内部上拉)
 *
 * 行为:
 *   GPIO2 按一次 → 长鸣 2 秒 → 快速闪烁 (0.1s/0.1s) 循环
 *   GPIO3 按下   → 立即停止, 回到待机
 *
 * scan() 每 10ms 调用一次 (core1_ctrl 循环)
 * ================================================================ */

/* ---- 蜂鸣器 ---- */
#define BUZZER_PIN                  GPIO_NUM_1
#define BUZZER_LEDC_TIMER           LEDC_TIMER_2
#define BUZZER_LEDC_CHANNEL         LEDC_CHANNEL_3
#define BUZZER_FREQ_HZ              2700
#define BUZZER_RESOLUTION           8
#define BUZZER_DUTY_ON              128

/* ---- 按钮 ---- */
#define BUZZER_PLAY_PIN             GPIO_NUM_2      /* 按一次 = 启动 */
#define BUZZER_STOP_PIN             GPIO_NUM_3      /* 按下 = 停止 */

/* ---- 计时 (scan 周期 = 10ms) ---- */
#define BUZZER_SCAN_MS              10
#define TICK_2S                     (2000 / BUZZER_SCAN_MS)
#define TICK_01S                    (100  / BUZZER_SCAN_MS)

/* ---- 去抖 (50ms) ---- */
#define BUZZER_DEBOUNCE_CNT         5


class Buzzer {
public:
    Buzzer();
    void init();
    void scan();

private:
    void _output(bool on);

    enum State {
        IDLE,
        ALERT_BEEP_2S,          /* 长鸣 2 秒           */
        ALERT_FLASH_ON,         /* 闪烁 ON  0.1s      */
        ALERT_FLASH_OFF,        /* 闪烁 OFF 0.1s      */
    };

    State   _state;
    uint8_t _cnt;

    uint8_t _db2, _db3;         /* 去抖计数            */
    bool    _db2_prev;          /* GPIO2 上一拍去抖值 (边沿检测) */
};
