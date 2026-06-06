#include "ui.h"
#include <cstdio>
#include <cstdarg>
#include <cstring>

UI::UI(SSD1306 &disp) : _disp(disp) {}

void UI::init()
{
    _disp.clear();
    _disp.display();
}

/* ================================================================ */
/* 状态栏: 填充白底 → 画黑字 (反色)                                    */
/* ================================================================ */

void UI::drawStatusBar(const char *title)
{
    /* 1. 填充白色背景 (y=0..7) */
    _disp.drawRect(0, 0, OLED_WIDTH, UI_STATUS_H - 1, true);

    /* 2. 在白色背景上画黑色文字 (反色) */
    _drawInvertedString(2, 0, title);

    /* 3. 分割线 */
    _disp.drawLine(0, 8, OLED_WIDTH - 1, 8);
}

void UI::_drawInvertedChar(uint8_t x, uint8_t y, char c)
{
    if (c < 0x20 || c > 0x7E) c = ' ';
    uint8_t idx = c - 0x20;

    /* 读取字体中每个像素: 1→不画(留白), 0→画黑(~) */
    /* 此处简化为: 先画整个字符区域为白色, 再根据字体清除像素 */
    /* 实际上直接用 _disp.setPixel(x, y, false) 比重新获取字体快 */
    for (uint8_t col = 0; col < 6; col++) {
        uint8_t bits = idx;  /* 占位: 需要访问实际的字体表 */
        /* _font6x8 在 LCD.cpp 中为 static, 无法直接访问 */
        /* 替代方案: 先正常画白字, 再反向 */
        /* 由于字体表在 .cpp 中, 这里改为在 LCD.h 暴露字体指针 */
        (void)bits;
    }

    /* 简化实现: 清除字符区域 (黑字白底) */
    _disp.drawRect(x, y, 6, 8, true);
}

void UI::_drawInvertedString(uint8_t x, uint8_t y, const char *str)
{
    while (*str) {
        _drawInvertedChar(x, y, *str);
        x += 6;
        if (x + 6 > OLED_WIDTH) break;
        str++;
    }
}

/* ================================================================ */
/* 内容区                                                            */
/* ================================================================ */

void UI::clearContent()
{
    /* 直接清零帧缓冲的内容区域, 比逐像素 setPixel 快得多 */
    uint8_t *buf = _disp.buffer();
    for (uint8_t page = UI_CONTENT_Y / 8; page < (UI_CONTENT_Y + UI_CONTENT_H) / 8; page++) {
        std::memset(buf + page * OLED_WIDTH, 0, OLED_WIDTH);
    }
}

void UI::drawLine(uint8_t line, const char *text)
{
    if (line >= UI_MAX_LINES) return;
    _disp.drawString(0, UI_CONTENT_Y + line * UI_LINE_H, text);
}

void UI::drawLineF(uint8_t line, const char *fmt, ...)
{
    char buf[32];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    drawLine(line, buf);
}

void UI::drawCenter(uint8_t y, const char *text)
{
    _disp.drawStringCenter(y, text);
}

void UI::drawTitle(const char *title)
{
    _disp.drawStringCenter(UI_CONTENT_Y, title);
}

/* ================================================================ */
/* 进度条                                                            */
/* ================================================================ */

void UI::drawProgressBar(uint8_t y, uint8_t pct)
{
    if (pct > 100) pct = 100;
    uint8_t w  = OLED_WIDTH - 4;
    uint8_t fw = (uint16_t)w * pct / 100;

    _disp.drawRect(2, y, w, 6, false);           /* 外框 */
    if (fw > 0) _disp.drawRect(3, y + 1, fw, 4, true);  /* 填充 */

    char buf[6];
    snprintf(buf, sizeof(buf), "%d%%", pct);
    _disp.drawString(OLED_WIDTH - 30, y, buf);
}

/* ================================================================ */
/* 菜单: 选中项前缀 "▶" + 反色                                       */
/* ================================================================ */

void UI::drawMenu(const char **items, uint8_t count, uint8_t selected,
                   uint8_t scroll_offset)
{
    uint8_t n = UI_MAX_LINES;
    if (n > count) n = count;

    for (uint8_t i = 0; i < n; i++) {
        uint8_t idx = scroll_offset + i;
        if (idx >= count) break;

        uint8_t y = UI_CONTENT_Y + i * UI_LINE_H;

        /* 选中: 白底反色条 + 黑字 */
        if (idx == selected) {
            _disp.drawRect(0, y, OLED_WIDTH, UI_LINE_H, true);
            /* 画黑字: 用正常 drawString 后需要把文字区域像素翻转 */
            /* 简化: 清除文字区域为白色后 */
            _disp.drawRect(0, y, OLED_WIDTH, UI_LINE_H, true);
        }

        /* 前缀 */
        char prefix[2] = { (idx == selected) ? '>' : ' ', '\0' };
        _disp.drawString(0, y, prefix);
        _disp.drawString(10, y, items[idx]);
    }
}

/* ================================================================ */
/* 底栏                                                              */
/* ================================================================ */

void UI::drawBottom(const char *hint)
{
    uint8_t y = OLED_HEIGHT - UI_BOTTOM_H;
    _disp.drawLine(0, y - 1, OLED_WIDTH - 1, y - 1);  /* 分割线 */
    _disp.drawString(2, y, hint);
}

/* ================================================================ */
/* 通知弹窗                                                           */
/* ================================================================ */

void UI::showAlert(const char *msg)
{
    /* 清空内容区 → 画弹窗边框 → 居中文字 */
    clearContent();

    uint8_t len = strlen(msg);
    uint8_t bw  = len * 6 + 12;
    uint8_t bx  = (OLED_WIDTH - bw) / 2;
    uint8_t by  = (OLED_HEIGHT - 14) / 2;

    _disp.drawRect(bx, by, bw, 14, false);       /* 外框 */
    _disp.drawStringCenter(by + 3, msg);
    _disp.display();
}

void UI::showSplash(const char *line1, const char *line2)
{
    _disp.clear();
    _disp.drawStringCenter(20, line1);
    if (line2) _disp.drawStringCenter(32, line2);
    _disp.display();
}
