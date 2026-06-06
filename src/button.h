#pragma once
#include <stdint.h>
#include "TCA6408.h"

/* ================================================================
 * 按键子系统 — 4 键, 全部挂载 TCA6408
 *
 * TCA6408 P0~P3 用作按键输入, 低电平有效 (按下=GND)
 * 内部上拉由 TCA6408 配置, 也可用外部上拉电阻
 *
 * 引脚映射:
 *   P0 → BTN_1     P1 → BTN_2
 *   P2 → BTN_3     P3 → BTN_4
 *
 * ┌─── 硬件连线 (请按实际 PCB 填写) ───┐
 * │ BTN_1 → TCA6408_P0 = _________   │
 * │ BTN_2 → TCA6408_P1 = _________   │
 * │ BTN_3 → TCA6408_P2 = _________   │
 * │ BTN_4 → TCA6408_P3 = _________   │
 * └───────────────────────────────────┘
 * ================================================================ */

#define BUTTON_COUNT                4

/* 按钮到 TCA6408 引脚的映射 (低电平有效) */
#define BTN_PIN_0                  TCA6408_PIN_P0
#define BTN_PIN_1                  TCA6408_PIN_P1
#define BTN_PIN_2                  TCA6408_PIN_P2
#define BTN_PIN_3                  TCA6408_PIN_P3

/* 事件 */
enum ButtonEvent {
    BTN_NONE         = 0x00,
    BTN_PRESSED      = (1 << 0),
    BTN_RELEASED     = (1 << 1),
    BTN_LONG_PRESS   = (1 << 2),
    BTN_DOUBLE_CLICK = (1 << 3),
};

struct ButtonInfo {
    uint8_t index;      /* 按钮编号 0~3 */
    uint8_t event;      /* 事件类型 */
};

/* 时间参数 (ms) */
#define BTN_DEBOUNCE_MS            50
#define BTN_LONG_PRESS_MS          1000
#define BTN_DOUBLE_CLICK_MS        400
#define BTN_SCAN_INTERVAL_MS       10
#define BTN_DEBOUNCE_CNT           (BTN_DEBOUNCE_MS / BTN_SCAN_INTERVAL_MS)


class Button {
public:
    Button(TCA6408 &io);

    void init();
    void scan();                              /* 每 BTN_SCAN_INTERVAL_MS 调用 */
    bool getEvent(ButtonInfo &info);          /* 非阻塞取事件 */
    bool state(uint8_t index);                /* 查询按下状态 (1=按下) */

private:
    struct State {
        bool     pressed    : 1;
        bool     prev       : 1;
        uint8_t  event      : 4;
        uint16_t hold_cnt;                     /* 按住计数 (去抖+长按) */
        uint16_t release_cnt;                  /* 释放后计数 (双击间隔) */
    };

    TCA6408 &_io;
    State    _btn[BUTTON_COUNT];
    uint8_t  _poll_idx;                        /* 事件轮询游标 */
};
