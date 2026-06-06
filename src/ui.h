#pragma once
#include <stdint.h>
#include "LCD.h"

/*
 * UI 抽象层 — 独立于显示驱动, 当前基于 SSD1306 (OLED 128×64)
 *
 * 布局:
 *   ┌──────────────────────┐ y=0
 *   │ 状态栏 (8px)         │ y=0..7
 *   │ 分割线               │ y=8
 *   │ 内容区 (48px, 6行)   │ y=9..56
 *   │ 分割线               │ y=56
 *   │ 底部栏 (8px)         │ y=57..63
 *   └──────────────────────┘ y=64
 */

/* 布局参数 (基于 OLED 128×64) */
#define UI_STATUS_H                 9       /* 状态栏 + 分割线 */
#define UI_BOTTOM_H                 8
#define UI_CONTENT_Y                UI_STATUS_H
#define UI_CONTENT_H                (OLED_HEIGHT - UI_STATUS_H - UI_BOTTOM_H)
#define UI_LINE_H                   8
#define UI_MAX_LINES                (UI_CONTENT_H / UI_LINE_H)


class UI {
public:
    UI(SSD1306 &disp);

    void init();

    /* 状态栏: 白色标题条 + 分割线 */
    void drawStatusBar(const char *title);

    /* 内容区 */
    void clearContent();
    void drawLine(uint8_t line, const char *text);
    void drawLineF(uint8_t line, const char *fmt, ...);
    void drawCenter(uint8_t y, const char *text);
    void drawTitle(const char *title);

    /* 进度条 (0~100) */
    void drawProgressBar(uint8_t y, uint8_t pct);

    /* 菜单: 选中项前显示 "▶" */
    void drawMenu(const char **items, uint8_t count, uint8_t selected,
                  uint8_t scroll_offset = 0);

    /* 底栏提示 */
    void drawBottom(const char *hint);

    /* 弹窗 */
    void showAlert(const char *msg);
    void showSplash(const char *line1, const char *line2 = nullptr);

private:
    SSD1306 &_disp;

    /* 反色字符绘制: 在白底上画黑字 */
    void _drawInvertedChar(uint8_t x, uint8_t y, char c);
    void _drawInvertedString(uint8_t x, uint8_t y, const char *str);
};
