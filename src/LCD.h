#pragma once
#include <stdint.h>
#include <stddef.h>

/* ================================================================
 * LCD 显示模块
 * 共用 I2C: I2C_NUM_0
 *
 * 包含两类驱动:
 *   1. LCD   — PCF8574 + HD44780 字符液晶 (16×2)
 *   2. OLED  — SSD1306 0.96寸 图形 OLED (128×64)
 * ================================================================ */

/* ================================================================
 * 第一部分: HD44780 字符液晶 (PCF8574 I2C 转接板)
 *
 * PCF8574 → HD44780:
 *   P0=RS  P1=RW  P2=EN  P3=BL
 *   P4=D4  P5=D5  P6=D6  P7=D7
 * ================================================================ */
#define LCD1602_SCL                 4
#define LCD1602_SDA                 5
#define LCD1602_I2C_ADDR            0x27
#define LCD1602_COLS                16
#define LCD1602_ROWS                2

#define LCD_CMD_CLEAR               0x01
#define LCD_CMD_HOME                0x02
#define LCD_CMD_ENTRY_MODE          0x04
#define LCD_CMD_DISPLAY_CTRL        0x08
#define LCD_CMD_FUNCTION_SET        0x20
#define LCD_CMD_SET_DDRAM           0x80

#define LCD_FLAG_ENTRY_LEFT         0x02
#define LCD_FLAG_DISPLAY_ON         0x04
#define LCD_FLAG_CURSOR_ON          0x02
#define LCD_FLAG_BLINK_ON           0x01
#define LCD_FLAG_8BIT               0x10
#define LCD_FLAG_2LINE              0x08


class LCD1602 {
public:
    LCD1602(uint8_t addr = LCD1602_I2C_ADDR, uint8_t cols = LCD1602_COLS,
            uint8_t rows = LCD1602_ROWS);

    void init();
    void clear();
    void home();
    void setCursor(uint8_t col, uint8_t row);
    void write(const char *str);
    void writeChar(char c);
    void backlight(bool on);
    void display(bool on);
    void cursor(bool on);
    void blink(bool on);

private:
    void _write4bit(uint8_t data);
    void _write8bit(uint8_t data, bool rs);
    void _pulse_enable(uint8_t data);
    void _i2c_write(uint8_t val);
    void _delayUs(uint32_t us);

    uint8_t _addr;
    uint8_t _cols, _rows;
    uint8_t _backlight;
    uint8_t _display_ctrl;
    uint8_t _entry_mode;
};

/* ================================================================
 * 第二部分: SSD1306 0.96寸 OLED (128×64, I2C)
 *
 * I2C 地址: 0x3C (SA0=GND)
 * 单色 (1 bit/像素), 8 页 × 128 列
 * 帧缓冲: 1024 字节
 * ================================================================ */
#define OLED_I2C_ADDR               0x3C
#define OLED_WIDTH                  128
#define OLED_HEIGHT                 64
#define OLED_PAGES                  (OLED_HEIGHT / 8)

/* 基本命令 */
#define OLED_CMD_DISPLAY_OFF        0xAE
#define OLED_CMD_DISPLAY_ON         0xAF
#define OLED_CMD_SET_CONTRAST       0x81
#define OLED_CMD_NORMAL_DISP        0xA6
#define OLED_CMD_INVERT_DISP        0xA7
#define OLED_CMD_MEMORY_MODE        0x20
#define OLED_CMD_COL_ADDR           0x21
#define OLED_CMD_PAGE_ADDR          0x22
#define OLED_CMD_START_LINE         0x40


class SSD1306 {
public:
    SSD1306(uint8_t addr = OLED_I2C_ADDR);

    void init();
    void clear();
    void display();

    void setPixel(uint8_t x, uint8_t y, bool on);
    void drawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
    void drawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool fill = false);
    void drawCircle(uint8_t cx, uint8_t cy, uint8_t r, bool fill = false);

    void drawChar(uint8_t x, uint8_t y, char c);
    void drawString(uint8_t x, uint8_t y, const char *str);
    void drawStringCenter(uint8_t y, const char *str);

    void drawBitmap(uint8_t x, uint8_t y, const uint8_t *bmp,
                    uint8_t w, uint8_t h);

    void setContrast(uint8_t val);
    void invert(bool en);
    void sleep(bool en);

    uint8_t* buffer()       { return _buf; }
    uint8_t  width()  const { return OLED_WIDTH; }
    uint8_t  height() const { return OLED_HEIGHT; }

private:
    void _sendCmd(uint8_t cmd);
    void _sendData(const uint8_t *data, size_t len);

    uint8_t _addr;
    uint8_t _buf[OLED_WIDTH * OLED_PAGES];
};
