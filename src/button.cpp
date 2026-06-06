#include "button.h"
#include "esp_log.h"
#include <cstring>

static const char *TAG = "Button";

/* TCA6408 引脚查找表 (P0~P3 → 4 键) */
static const uint8_t _pin_map[BUTTON_COUNT] = {
    BTN_PIN_0, BTN_PIN_1, BTN_PIN_2, BTN_PIN_3,
};

Button::Button(TCA6408 &io) : _io(io), _poll_idx(0)
{
    std::memset(_btn, 0, sizeof(_btn));
}

void Button::init()
{
    ESP_LOGI(TAG, "OK (%d 键, 低电平有效, 去抖=%lums)",
             BUTTON_COUNT, BTN_DEBOUNCE_MS);
}

/*
 * 扫描全部按钮 — 每 BTN_SCAN_INTERVAL_MS 调用一次
 * 状态机: 空闲→按下沿(PRESSED)→按住(LONG_PRESS)→释放沿(RELEASED/DOUBLE_CLICK)→空闲
 */
void Button::scan()
{
    uint8_t port = _io.readPort();              /* 读 TCA6408 8 位端口 */

    for (int i = 0; i < BUTTON_COUNT; i++) {
        State &b = _btn[i];

        /* TCA6408: 输入引脚 0=低电平(按下), 读取取反 → 1=按下 */
        bool level = (port & _pin_map[i]) == 0;

        b.prev    = b.pressed;
        b.pressed = level;

        /* ── 按下沿 (刚按下) ── */
        if (b.pressed && !b.prev) {
            b.hold_cnt = 0;
            b.event    = BTN_PRESSED;
        }

        /* ── 按住中 → 长按检测 ── */
        if (b.pressed) {
            if (b.hold_cnt < 0xFFFF) b.hold_cnt++;
            if (b.hold_cnt == BTN_LONG_PRESS_MS / BTN_SCAN_INTERVAL_MS) {
                b.event = BTN_LONG_PRESS;
            }
        }

        /* ── 释放沿 (刚松开) → 双击检测 ── */
        if (!b.pressed && b.prev) {
            b.event = BTN_RELEASED;

            /* release_cnt > 0 说明之前有一次释放仍在双击窗口内 */
            if (b.release_cnt > 0 &&
                b.release_cnt <= BTN_DOUBLE_CLICK_MS / BTN_SCAN_INTERVAL_MS) {
                b.event = BTN_DOUBLE_CLICK;
            }
            b.release_cnt = 0;
        }

        /* ── 未按下时累计释放间隔 (用于双击检测) ── */
        if (!b.pressed && b.release_cnt < 0xFFFF) {
            b.release_cnt++;
        }
    }
}

/*
 * 非阻塞取事件 — 轮询所有按钮, 返回第一个待消费事件
 * 返回 true 表示有事件, 事件信息写入 info
 */
bool Button::getEvent(ButtonInfo &info)
{
    for (int n = 0; n < BUTTON_COUNT; n++) {
        State &b = _btn[_poll_idx];
        _poll_idx = (_poll_idx + 1) % BUTTON_COUNT;

        if (b.event != BTN_NONE) {
            /* _poll_idx 已递增, 所以前一个索引 = 当前按钮 */
            info.index = (_poll_idx == 0) ? BUTTON_COUNT - 1 : _poll_idx - 1;
            info.event = b.event;
            b.event    = BTN_NONE;
            return true;
        }
    }
    return false;
}

/* 查询按钮当前是否按下 (1=按下, 0=松开) */
bool Button::state(uint8_t index)
{
    if (index >= BUTTON_COUNT) return false;
    return _io.getPin(_pin_map[index]);
}
