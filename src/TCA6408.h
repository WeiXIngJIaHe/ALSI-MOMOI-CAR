#pragma once
#include <stdint.h>

/* ================================================================
 * TCA6408A I2C 8 位 IO 扩展器
 * 共用 I2C: 与 IS31FL3239、LCD1602 共享 I2C_NUM_0
 *
 * 数据手册要点:
 *   - 上电默认: 全部寄存器 = 0xFF (全引脚为输入)
 *   - I2C 地址: 0x20 (A2=A1=A0=GND), 可选 0x21~0x27
 *   - 引脚可独立配置为输入/输出
 *   - 极性翻转寄存器可改变输入逻辑
 *
 * 寄存器 (8 位):
 *   0x00 INPUT      只读  引脚实际电平 (不受极性翻转影响? 需确认)
 *   0x01 OUTPUT     读/写 输出锁存值
 *   0x02 POLARITY   读/写 位=1 → 翻转该引脚输入值
 *   0x03 CONFIG     读/写 位=0→输出, 位=1→输入 (默认 0xFF)
 *
 * 本项目用途:
 *   P0~P3 = 按键输入 (低电平有效, 按下=GND)
 *   P4~P7 = 保留 (可作为输出控制其他外设)
 * ================================================================ */

/* ---- 硬件引脚 ---- */
#define TCA6408_SCL                 4
#define TCA6408_SDA                 5
#define TCA6408_I2C_ADDR            0x20

/* ---- 寄存器 ---- */
#define TCA6408_REG_INPUT           0x00
#define TCA6408_REG_OUTPUT          0x01
#define TCA6408_REG_POLARITY        0x02
#define TCA6408_REG_CONFIG          0x03

/* ---- 引脚位掩码 ---- */
#define TCA6408_PIN_P0              (1 << 0)
#define TCA6408_PIN_P1              (1 << 1)
#define TCA6408_PIN_P2              (1 << 2)
#define TCA6408_PIN_P3              (1 << 3)
#define TCA6408_PIN_P4              (1 << 4)
#define TCA6408_PIN_P5              (1 << 5)
#define TCA6408_PIN_P6              (1 << 6)
#define TCA6408_PIN_P7              (1 << 7)
#define TCA6408_PIN_ALL             0xFF

#define TCA6408_DIR_IN              1
#define TCA6408_DIR_OUT             0


class TCA6408 {
public:
    TCA6408(uint8_t addr = TCA6408_I2C_ADDR);

    void init();

    /* 端口级操作 */
    uint8_t readPort();                          /* 读全部 8 位输入 */
    void    writePort(uint8_t mask, uint8_t val); /* 写输出 (仅输出引脚生效) */
    void    setDir(uint8_t mask);                 /* 配置方向 0=OUT 1=IN */

    /* 引脚级操作 (低电平有效) */
    bool    getPin(uint8_t pin);                  /* 读单引脚, 返回 true=按下(低) */

    /* 寄存器原子操作 */
    int     writeReg(uint8_t reg, uint8_t val);
    int     readReg(uint8_t reg, uint8_t &val);

private:
    uint8_t _addr;
};
