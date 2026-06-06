#include "TCA6408.h"
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "TCA6408";

TCA6408::TCA6408(uint8_t addr) : _addr(addr) {}

/*
 * 初始化: P0~P3 设为输入(按键), P4~P7 保持输入(默认)
 * 上电默认 0xFF 已经是全输入, 此处显式写入确保状态
 * 不修改极性翻转寄存器 (保持 P0~P3 原生逻辑: 低=按)
 */
void TCA6408::init()
{
    writeReg(TCA6408_REG_CONFIG, 0xFF);     /* 全输入 */
    writeReg(TCA6408_REG_POLARITY, 0x00);   /* 不翻转 */

    ESP_LOGI(TAG, "OK (0x%02X, P0~P3=按键输入)", _addr);
}

/* ── 寄存器读写 ── */

int TCA6408::writeReg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(I2C_NUM_0, _addr, buf, 2,
                                      pdMS_TO_TICKS(10));
}

int TCA6408::readReg(uint8_t reg, uint8_t &val)
{
    return i2c_master_write_read_device(I2C_NUM_0, _addr, &reg, 1,
                                         &val, 1, pdMS_TO_TICKS(10));
}

/* ── 端口操作 ── */

uint8_t TCA6408::readPort()
{
    uint8_t val = 0xFF;
    readReg(TCA6408_REG_INPUT, val);
    return val;     /* 位=1 → 高电平(未按), 位=0 → 低电平(按下) */
}

void TCA6408::writePort(uint8_t mask, uint8_t val)
{
    uint8_t cur = readPort();
    cur = (cur & ~mask) | (val & mask);
    writeReg(TCA6408_REG_OUTPUT, cur);
}

void TCA6408::setDir(uint8_t mask)
{
    writeReg(TCA6408_REG_CONFIG, mask);
}

/*
 * 读单引脚状态
 * 返回: true=低电平(按下), false=高电平(未按)
 * 依据: 按键按下时引脚被拉到 GND, 读回 0
 */
bool TCA6408::getPin(uint8_t pin)
{
    uint8_t val = readPort();
    return (val & pin) == 0;
}
