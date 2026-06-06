#include "IS31FL3239.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <cstring>

static const char *TAG = "IS31FL3239";

/*
 * 共用 I2C 总线 — 一次性初始化, 静态守卫
 * IS31FL3239::init() 在 hw_init() 中最先被调用, 负责初始化 I2C
 * 后续 TCA6408、LCD1602 直接使用已初始化的 I2C_NUM_0
 */
static bool _i2c_ready = false;

static void _i2c_ensure()
{
    if (_i2c_ready) return;

    i2c_config_t cfg = {};
    cfg.mode             = I2C_MODE_MASTER;
    cfg.sda_io_num       = IS31FL3239_SDA;
    cfg.scl_io_num       = IS31FL3239_SCL;
    cfg.sda_pullup_en    = GPIO_PULLUP_ENABLE;
    cfg.scl_pullup_en    = GPIO_PULLUP_ENABLE;
    cfg.master.clk_speed = 400000;

    i2c_param_config(I2C_NUM_0, &cfg);
    i2c_driver_install(I2C_NUM_0, cfg.mode, 0, 0, 0);
    _i2c_ready = true;

    ESP_LOGI("I2C", "总线初始化 (SCL=%d SDA=%d)", IS31FL3239_SCL, IS31FL3239_SDA);
}

/* ================================================================ */

IS31FL3239::IS31FL3239(uint8_t addr) : _addr(addr)
{
    std::memset(_pwm, 0, sizeof(_pwm));
}

void IS31FL3239::init()
{
    _i2c_ensure();  /* 确保 I2C 总线就绪 */

    writeReg(IS31FL3239_REG_SHUTDOWN, IS31FL3239_SSD_NORMAL);
    writeReg(IS31FL3239_REG_GLOBAL_CTRL, IS31FL3239_CURRENT_HALF);
    clear();

    ESP_LOGI(TAG, "OK (0x%02X, %d 通道)", _addr, IS31FL3239_CH_COUNT);
}

/* ── 通道控制 ── */

void IS31FL3239::setChannel(uint8_t ch, uint8_t pwm)
{
    if (ch < 1 || ch > IS31FL3239_CH_COUNT) return;
    _pwm[ch - 1] = pwm;
}

void IS31FL3239::setAll(uint8_t pwm)
{
    std::memset(_pwm, pwm, sizeof(_pwm));
}

void IS31FL3239::setChannels(const uint8_t *pwm, uint8_t count)
{
    if (count > IS31FL3239_CH_COUNT) count = IS31FL3239_CH_COUNT;
    std::memcpy(_pwm, pwm, count);
}

/* ── 批量更新 ── */

void IS31FL3239::update()
{
    uint8_t buf[1 + IS31FL3239_CH_COUNT];
    buf[0] = IS31FL3239_REG_PWM_BASE;
    std::memcpy(buf + 1, _pwm, IS31FL3239_CH_COUNT);

    i2c_master_write_to_device(I2C_NUM_0, _addr, buf, sizeof(buf),
                                pdMS_TO_TICKS(20));
    writeReg(IS31FL3239_REG_UPDATE, 0x00);
}

void IS31FL3239::clear()
{
    std::memset(_pwm, 0, sizeof(_pwm));
    update();
}

void IS31FL3239::setCurrent(uint8_t level)
{
    writeReg(IS31FL3239_REG_GLOBAL_CTRL, level);
}

void IS31FL3239::shutdown(bool en)
{
    writeReg(IS31FL3239_REG_SHUTDOWN,
             en ? IS31FL3239_SSD_SHUTDOWN : IS31FL3239_SSD_NORMAL);
}

/* ── I2C 读写 ── */

int IS31FL3239::writeReg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(I2C_NUM_0, _addr, buf, 2,
                                      pdMS_TO_TICKS(10));
}

int IS31FL3239::readReg(uint8_t reg, uint8_t &val)
{
    return i2c_master_write_read_device(I2C_NUM_0, _addr, &reg, 1,
                                         &val, 1, pdMS_TO_TICKS(10));
}
