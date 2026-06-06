#pragma once
#include <stdint.h>

/* ================================================================
 * IS31FL3239 I2C 36 通道恒流 LED 驱动
 * 共用 I2C: I2C_NUM_0, 与 IS31FL3729/TCA6408/LCD 共享
 *
 * 特性:
 *   - 36 路恒流输出 (OUT1~OUT36)
 *   - 每通道 8-bit PWM (0~255)
 *   - 全局电流控制 (GCC)
 *   - PWM 锁存更新 (双缓冲, 防闪烁)
 *   - 软件关断
 *   - 开路/短路检测 (OSD)
 *   - 通道最大电流由外置电阻 R_ext 设定 (典型 38mA)
 *
 * I2C 地址 (7-bit):
 *   0x3C = AD 接 GND
 *   0x3F = AD 接 VCC
 *
 * 寄存器映射 (请与数据手册核对):
 *   0x00    Shutdown      [0]SSD: 0=关断 1=正常
 *   0x01~0x24 PWM1~36     OUT1~OUT36 亮度 (0x00~0xFF)
 *   0x25    Update        写任意值锁存全部 PWM
 *   0x26    LED Control   开路/短路检测使能
 *   0x27    Global Curr   全局电流 (0x00~0xFF)
 *   0x28    PWM Freq      PWM 频率选择
 *   0x4D    Reset         写 0xC5 复位全部寄存器
 * ================================================================ */

/* ---- 硬件引脚 ---- */
#define IS31FL3239_SCL              5
#define IS31FL3239_SDA              6
#define IS31FL3239_I2C_ADDR         0x3C    /* AD=GND; AD=VCC → 0x3F */

/* ---- 通道数 ---- */
#define IS31FL3239_CH_COUNT          36

/* ---- 寄存器 ---- */
#define IS31FL3239_REG_SHUTDOWN      0x00
#define IS31FL3239_REG_PWM_BASE      0x01    /* OUT1=0x01 ... OUT36=0x24 */
#define IS31FL3239_REG_UPDATE        0x25
#define IS31FL3239_REG_LED_CTRL      0x26
#define IS31FL3239_REG_GLOBAL_CTRL   0x27
#define IS31FL3239_REG_PWM_FREQ      0x28
#define IS31FL3239_REG_RESET         0x4D

/* ---- Shutdown (0x00) ---- */
#define IS31FL3239_SSD_SHUTDOWN      0x00
#define IS31FL3239_SSD_NORMAL        0x01

/* ---- LED Control (0x26) ---- */
#define IS31FL3239_OSD_EN            (1 << 0)    /* 开路/短路检测使能 */

/* ---- Global Current (0x27) ---- */
#define IS31FL3239_CURRENT_MAX       0xFF
#define IS31FL3239_CURRENT_HALF      0x80
#define IS31FL3239_CURRENT_MIN       0x01
#define IS31FL3239_CURRENT_OFF       0x00

/* ---- PWM ---- */
#define IS31FL3239_PWM_MAX           0xFF
#define IS31FL3239_PWM_OFF           0x00

/* ---- Reset ---- */
#define IS31FL3239_RESET_CMD         0xC5

/* ================================================================ */

class IS31FL3239 {
public:
    IS31FL3239(uint8_t addr = IS31FL3239_I2C_ADDR);

    void init();

    /* 单通道 */
    void setChannel(uint8_t ch, uint8_t pwm);       /* ch: 1~36 */

    /* 批量 */
    void setAll(uint8_t pwm);
    void setChannels(const uint8_t *pwm, uint8_t count);

    /* 全局 */
    void update();                                    /* 锁存 PWM */
    void clear();
    void setCurrent(uint8_t level);                   /* 全局电流 */
    void shutdown(bool en);

    /* 寄存器读写 */
    int  writeReg(uint8_t reg, uint8_t val);
    int  readReg(uint8_t reg, uint8_t &val);

private:
    uint8_t _addr;
    uint8_t _pwm[IS31FL3239_CH_COUNT];
};
