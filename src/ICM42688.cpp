#include "ICM42688.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>
#include <cmath>

static const char *TAG = "ICM42688";

/* ---- SPI 总线 (一次性初始化, 静态守卫) ---- */
static bool _spi_bus_ready = false;

static void _spi_ensure()
{
    if (_spi_bus_ready) return;

    spi_bus_config_t cfg = {};
    cfg.mosi_io_num     = ICM42688_MOSI;
    cfg.miso_io_num     = ICM42688_MISO;
    cfg.sclk_io_num     = ICM42688_SCLK;
    cfg.quadwp_io_num   = -1;
    cfg.quadhd_io_num   = -1;
    cfg.max_transfer_sz = 64;

    spi_bus_initialize(ICM42688_SPI_HOST, &cfg, SPI_DMA_DISABLED);
    _spi_bus_ready = true;

    ESP_LOGI("SPI", "总线初始化 (SCLK=%d MOSI=%d MISO=%d CS=%d)",
             ICM42688_SCLK, ICM42688_MOSI, ICM42688_MISO, ICM42688_CS);
}

/* ================================================================ */
/* 构造 / 析构                                                        */
/* ================================================================ */

ICM42688::ICM42688()
    : _spi(nullptr), _bank(0xFF),
      _accel_lsb(ICM42688_ACCEL_LSB_2G / 9.80665f),
      _gyro_lsb(ICM42688_GYRO_LSB_250DPS),
      _gbx(0), _gby(0), _gbz(0)
{}

ICM42688::~ICM42688()
{
    if (_spi) {
        spi_bus_remove_device(_spi);
    }
}

/* ================================================================ */
/* 初始化                                                              */
/* ================================================================ */

bool ICM42688::begin()
{
    _spi_ensure();

    /* 添加 SPI 设备 */
    spi_device_interface_config_t dev_cfg = {};
    dev_cfg.mode           = 0;            /* CPOL=0, CPHA=0 */
    dev_cfg.clock_speed_hz = ICM42688_SPI_CLK_HZ;
    dev_cfg.spics_io_num   = ICM42688_CS;
    dev_cfg.queue_size     = 1;
    dev_cfg.flags          = SPI_DEVICE_HALFDUPLEX;
    dev_cfg.cs_ena_pretrans = 1;           /* CS→SCLK 间隔 (SPI cycles) */

    esp_err_t ret = spi_bus_add_device(ICM42688_SPI_HOST, &dev_cfg, &_spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI 设备添加失败");
        return false;
    }

    /* 验证 WHO_AM_I */
    uint8_t id = whoAmI();
    if (id != ICM42688_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "WHO_AM_I=0x%02X (预期 0x%02X)", id, ICM42688_WHO_AM_I_VAL);
        return false;
    }

    /* 软件复位 */
    writeReg(ICM42688_REG_DEVICE_CONFIG, 0x01);
    vTaskDelay(pdMS_TO_TICKS(10));
    _bank = 0xFF;

    /* 默认配置 */
    setPowerMode(POWER_LOW_NOISE);
    setAccelFS(ACCEL_2G);
    setGyroFS(GYRO_250DPS);
    setAccelODR(ODR_100_HZ);
    setGyroODR(ODR_100_HZ);
    enableTemp(true);

    ESP_LOGI(TAG, "OK (SPI %dMHz, 2G/250dps/100Hz)",
             ICM42688_SPI_CLK_HZ / 1000000);
    return true;
}

uint8_t ICM42688::whoAmI()
{
    uint8_t val = 0;
    readReg(ICM42688_REG_WHO_AM_I, val);
    return val;
}

void ICM42688::reset()
{
    writeReg(ICM42688_REG_DEVICE_CONFIG, 0x01);
    _bank = 0xFF;
}

/* ================================================================ */
/* 配置                                                               */
/* ================================================================ */

void ICM42688::setAccelFS(AccelFS fs)
{
    uint8_t val;
    readReg(ICM42688_REG_ACCEL_CONFIG0, val);
    val &= ~ICM42688_ACCEL_FS_MASK;
    val |= (fs & 0x03) << ICM42688_ACCEL_FS_SHIFT;
    writeReg(ICM42688_REG_ACCEL_CONFIG0, val);
    _applyScale();
}

void ICM42688::setGyroFS(GyroFS fs)
{
    uint8_t val;
    readReg(ICM42688_REG_GYRO_CONFIG0, val);
    val &= ~ICM42688_GYRO_FS_MASK;
    val |= (fs & 0x03) << ICM42688_GYRO_FS_SHIFT;
    writeReg(ICM42688_REG_GYRO_CONFIG0, val);
    _applyScale();
}

void ICM42688::setAccelODR(ODR odr)
{
    uint8_t val;
    readReg(ICM42688_REG_ACCEL_CONFIG0, val);
    val &= ~ICM42688_ACCEL_ODR_MASK;
    val |= (odr & 0x0F) << ICM42688_ACCEL_ODR_SHIFT;
    writeReg(ICM42688_REG_ACCEL_CONFIG0, val);
}

void ICM42688::setGyroODR(ODR odr)
{
    uint8_t val;
    readReg(ICM42688_REG_GYRO_CONFIG0, val);
    val &= ~ICM42688_GYRO_ODR_MASK;
    val |= (odr & 0x0F) << ICM42688_GYRO_ODR_SHIFT;
    writeReg(ICM42688_REG_GYRO_CONFIG0, val);
}

void ICM42688::setPowerMode(PowerMode mode)
{
    uint8_t val;
    readReg(ICM42688_REG_PWR_MGMT0, val);
    val &= ~ICM42688_PWR_MODE_MASK;
    val |= (mode & ICM42688_PWR_MODE_MASK);
    writeReg(ICM42688_REG_PWR_MGMT0, val);
}

void ICM42688::enableTemp(bool en)
{
    uint8_t val;
    readReg(ICM42688_REG_PWR_MGMT0, val);
    if (en) val &= ~ICM42688_PWR_TEMP_DIS;
    else    val |=  ICM42688_PWR_TEMP_DIS;
    writeReg(ICM42688_REG_PWR_MGMT0, val);
}

/* ================================================================ */
/* 中断                                                               */
/* ================================================================ */

void ICM42688::configINT1(bool latched, bool active_high, bool open_drain)
{
    uint8_t val = 0;
    if (latched)     val |= ICM42688_INT1_MODE_LATCHED;
    if (active_high) val |= ICM42688_INT1_ACTIVE_HIGH;
    if (open_drain)  val |= ICM42688_INT1_OPEN_DRAIN;
    writeReg(ICM42688_REG_INT_CONFIG, val);
}

void ICM42688::enableDataReadyINT(bool en)
{
    uint8_t val;
    readReg(ICM42688_REG_INT_SOURCE0, val);
    if (en) val |=  ICM42688_INT_DATA_RDY;
    else    val &= ~ICM42688_INT_DATA_RDY;
    writeReg(ICM42688_REG_INT_SOURCE0, val);
}

/* ================================================================ */
/* 数据读取                                                           */
/* ================================================================ */

ICM42688::Data3F ICM42688::readAll()
{
    DataRaw raw = readRaw();
    Data3F  d;
    d.ax   = (float)raw.ax * _accel_lsb;
    d.ay   = (float)raw.ay * _accel_lsb;
    d.az   = (float)raw.az * _accel_lsb;
    d.gx   = (float)raw.gx * _gyro_lsb - _gbx;
    d.gy   = (float)raw.gy * _gyro_lsb - _gby;
    d.gz   = (float)raw.gz * _gyro_lsb - _gbz;
    d.temp = (float)raw.temp / ICM42688_TEMP_LSB + ICM42688_TEMP_OFFSET;
    return d;
}

ICM42688::DataRaw ICM42688::readRaw()
{
    DataRaw raw = {};
    uint8_t buf[14];
    if (!readRegs(ICM42688_REG_ACCEL_DATA_X1, buf, 14)) return raw;

    raw.ax   = (int16_t)((buf[0]  << 8) | buf[1]);
    raw.ay   = (int16_t)((buf[2]  << 8) | buf[3]);
    raw.az   = (int16_t)((buf[4]  << 8) | buf[5]);
    raw.gx   = (int16_t)((buf[6]  << 8) | buf[7]);
    raw.gy   = (int16_t)((buf[8]  << 8) | buf[9]);
    raw.gz   = (int16_t)((buf[10] << 8) | buf[11]);
    raw.temp = (int16_t)((buf[12] << 8) | buf[13]);
    return raw;
}

void ICM42688::readAccel(float &ax, float &ay, float &az)
{
    uint8_t buf[6];
    if (!readRegs(ICM42688_REG_ACCEL_DATA_X1, buf, 6)) {
        ax = ay = az = 0; return;
    }
    ax = (float)(int16_t)((buf[0] << 8) | buf[1]) * _accel_lsb;
    ay = (float)(int16_t)((buf[2] << 8) | buf[3]) * _accel_lsb;
    az = (float)(int16_t)((buf[4] << 8) | buf[5]) * _accel_lsb;
}

void ICM42688::readGyro(float &gx, float &gy, float &gz)
{
    uint8_t buf[6];
    if (!readRegs(ICM42688_REG_GYRO_DATA_X1, buf, 6)) {
        gx = gy = gz = 0; return;
    }
    gx = (float)(int16_t)((buf[0] << 8) | buf[1]) * _gyro_lsb - _gbx;
    gy = (float)(int16_t)((buf[2] << 8) | buf[3]) * _gyro_lsb - _gby;
    gz = (float)(int16_t)((buf[4] << 8) | buf[5]) * _gyro_lsb - _gbz;
}

float ICM42688::readTemp()
{
    uint8_t buf[2];
    if (!readRegs(ICM42688_REG_TEMP_DATA1, buf, 2)) return 0;
    return (float)(int16_t)((buf[0] << 8) | buf[1]) / ICM42688_TEMP_LSB
           + ICM42688_TEMP_OFFSET;
}

/* ================================================================ */
/* 校准                                                               */
/* ================================================================ */

void ICM42688::calibrate(uint16_t samples)
{
    double sum_x = 0, sum_y = 0, sum_z = 0;
    ESP_LOGI(TAG, "陀螺仪校准 (%d 采样)...", samples);

    for (uint16_t i = 0; i < samples; i++) {
        DataRaw raw = readRaw();
        sum_x += raw.gx * _gyro_lsb;
        sum_y += raw.gy * _gyro_lsb;
        sum_z += raw.gz * _gyro_lsb;
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    _gbx = (float)(sum_x / samples);
    _gby = (float)(sum_y / samples);
    _gbz = (float)(sum_z / samples);
    ESP_LOGI(TAG, "校准完成: bias=(%.4f, %.4f, %.4f) dps", _gbx, _gby, _gbz);
}

/* ================================================================ */
/* SPI 读写 — ICM-42688 协议                                          */
/*   命令字 = (reg << 1) | R/W bit                                   */
/*   写: TX=[cmd, data]                                              */
/*   读: TX=[cmd|0x80, dummy], RX从第2字节取数据                       */
/*   半双工: RX 数据仅在 dummy 位后才有效                               */
/* ================================================================ */

bool ICM42688::writeReg(uint8_t reg, uint8_t val)
{
    uint8_t cmd = (reg << 1) | ICM42688_SPI_WRITE_BIT;

    spi_transaction_t t = {};
    t.flags = SPI_TRANS_USE_TXDATA;
    t.length = 16;                        /* 2 字节 */
    t.tx_data[0] = cmd;
    t.tx_data[1] = val;

    return spi_device_transmit(_spi, &t) == ESP_OK;
}

bool ICM42688::readReg(uint8_t reg, uint8_t &val)
{
    uint8_t cmd = (reg << 1) | ICM42688_SPI_READ_BIT;

    spi_transaction_t t = {};
    t.flags    = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    t.length   = 16;
    t.tx_data[0] = cmd;
    t.tx_data[1] = 0x00;                 /* dummy */

    esp_err_t ret = spi_device_transmit(_spi, &t);
    val = t.rx_data[1];                   /* 第二个字节 = 寄存器值 */
    return ret == ESP_OK;
}

bool ICM42688::readRegs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    /* 构造发送缓冲: 1 命令 + len 个 dummy */
    uint8_t tx[15];
    tx[0] = (reg << 1) | ICM42688_SPI_READ_BIT;
    std::memset(tx + 1, 0x00, len);

    uint8_t rx[15] = {};

    spi_transaction_t t = {};
    t.length    = (1 + len) * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    esp_err_t ret = spi_device_transmit(_spi, &t);
    /* rx[1..len] = 有效数据 */
    std::memcpy(buf, rx + 1, len);
    return ret == ESP_OK;
}

/* ================================================================ */
/* Bank 切换                                                           */
/* ================================================================ */

void ICM42688::_selectBank(uint8_t bank)
{
    if (bank == _bank) return;
    writeReg(ICM42688_REG_BANK_SEL, bank);
    _bank = bank;
}

void ICM42688::_applyScale()
{
    uint8_t val;

    readReg(ICM42688_REG_ACCEL_CONFIG0, val);
    switch ((val & ICM42688_ACCEL_FS_MASK) >> ICM42688_ACCEL_FS_SHIFT) {
    case 0: _accel_lsb = ICM42688_ACCEL_LSB_2G  / 9.80665f; break;
    case 1: _accel_lsb = ICM42688_ACCEL_LSB_4G  / 9.80665f; break;
    case 2: _accel_lsb = ICM42688_ACCEL_LSB_8G  / 9.80665f; break;
    case 3: _accel_lsb = ICM42688_ACCEL_LSB_16G / 9.80665f; break;
    }

    readReg(ICM42688_REG_GYRO_CONFIG0, val);
    switch ((val & ICM42688_GYRO_FS_MASK) >> ICM42688_GYRO_FS_SHIFT) {
    case 0: _gyro_lsb = ICM42688_GYRO_LSB_250DPS;  break;
    case 1: _gyro_lsb = ICM42688_GYRO_LSB_500DPS;  break;
    case 2: _gyro_lsb = ICM42688_GYRO_LSB_1000DPS; break;
    case 3: _gyro_lsb = ICM42688_GYRO_LSB_2000DPS; break;
    }
}
