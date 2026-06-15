#include "IMU.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include <cstring>
#include <cmath>

static const char *TAG = "QMI8658";

static bool              _spi_ready = false;
static SemaphoreHandle_t _spi_mutex = nullptr;   /* SPI 总线互斥锁 */

static void _spi_ensure()
{
    if (_spi_ready) return;
    spi_bus_config_t cfg = {};
    cfg.mosi_io_num     = QMI8658_MOSI;
    cfg.miso_io_num     = QMI8658_MISO;
    cfg.sclk_io_num     = QMI8658_SCLK;
    cfg.quadwp_io_num   = -1;
    cfg.quadhd_io_num   = -1;
    cfg.max_transfer_sz = 4096;
    spi_bus_initialize(QMI8658_SPI_HOST, &cfg, SPI_DMA_CH_AUTO);
    _spi_ready = true;
    ESP_LOGI(TAG, "SPI 总线 (SCLK=%d MOSI=%d MISO=%d CS=%d) — 软件控 CS",
             QMI8658_SCLK, QMI8658_MOSI, QMI8658_MISO, QMI8658_CS);
}

/* ---- 软件 CS 宏 ---- */
#define CS_LOW()   gpio_set_level((gpio_num_t)QMI8658_CS, 0)
#define CS_HIGH()  gpio_set_level((gpio_num_t)QMI8658_CS, 1)

/* ================================================================ */

QMI8658::QMI8658()
    : _spi(nullptr), _accel_lsb(9.80665f / QMI8658_ACC_LSB_2G),
      _gyro_lsb(1.0f / QMI8658_GYRO_LSB_2048DPS),
      _gbx(0), _gby(0), _gbz(0)
{}

QMI8658::~QMI8658()
{
    if (_spi) spi_bus_remove_device(_spi);
}

/* ================================================================ */

bool QMI8658::begin()
{
    
    _spi_ensure();
    vTaskDelay(pdMS_TO_TICKS(200));
    spi_device_interface_config_t dev_cfg = {};
    dev_cfg.mode             = 0;    /* Mode 0: CS=0, SPC=1, Mode 3:CS=0,SPC=1 */
    dev_cfg.clock_speed_hz   = QMI8658_SPI_CLK_HZ;
    dev_cfg.spics_io_num     = QMI8658_CS;    /* 硬件 CS 精准时序 */
    dev_cfg.queue_size       = 10;
    dev_cfg.cs_ena_pretrans  = 1;             /* CS 提前 2 周期 */
    dev_cfg.cs_ena_posttrans = 1;             /* CS 延后 8 周期 (锁存写数据) */
    dev_cfg.input_delay_ns   = 10;

    if (spi_bus_add_device(QMI8658_SPI_HOST, &dev_cfg, &_spi) != ESP_OK)
        return false;

    /* SPI 互斥锁 (防双核并发访问) */
    if (!_spi_mutex) {
        _spi_mutex = xSemaphoreCreateMutex();
    }

    /* 1. 验证身份 */
    /* SPI 帧诊断 */
    ESP_LOGI(TAG, "-- SPI 帧诊断 (reg 0x00, 软件CS) --");
    for (int bits = 8; bits <= 32; bits += 8) {
        spi_transaction_t t = {};
        t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
        t.length = bits;
        t.tx_data[0] = (0x00 | 0x80);  // 第一字节: 读 WHO_AM_I
        t.tx_data[1] = 0x00;           // 第二字节: dummy
        spi_device_transmit(_spi, &t);
        ESP_LOGI(TAG, "  %d-bit: rx[0]=0x%02X [1]=0x%02X [2]=0x%02X [3]=0x%02X",
                 bits, t.rx_data[0], t.rx_data[1], t.rx_data[2], t.rx_data[3]);
    }

    uint8_t id = whoAmI();
    ESP_LOGI(TAG, "WHO_AM_I=0x%02X %s", id,
             (id == QMI8658_WHO_AM_I_VAL) ? "✓" : "✗");

    /* 2. 软件复位: CTRL1[0]=1 */
    writeReg(QMI8658_REG_CTRL1, QMI8658_CTRL1_SRST);
    vTaskDelay(pdMS_TO_TICKS(50));

    /* 3. CTRL1: 地址自增[6]=1, 4线SPI[7]=0 → 0x40 */
    writeReg(QMI8658_REG_CTRL1, 0x40);

    /* 4. CTRL2: ACC ±2G @ 100Hz → 0x04 */
    /*    CTRL3: GYRO ±2048dps @ 100Hz → 0x64 */
    writeReg(QMI8658_REG_CTRL2,  0x04);
    writeReg(QMI8658_REG_CTRL3,  0x64);

    /* 5. CTRL7: 使能 ACC + GYRO (低噪声模式) */
    writeReg(QMI8658_REG_CTRL7,0x03);

    /* 6. 更新 LSB 比例系数 */
    _applyScale();

    ESP_LOGI(TAG, "QMI8658 初始化完成 ");
    return true;
}

uint8_t QMI8658::whoAmI()
{
    uint8_t val = 0;
    readReg(QMI8658_REG_WHO_AM_I, val);
    return val;
}

/* ================================================================ */

QMI8658::Data3F QMI8658::readAll()
{
    DataRaw r = readRaw();
    Data3F  d;
    d.ax   = (float)r.ax * _accel_lsb;
    d.ay   = (float)r.ay * _accel_lsb;
    d.az   = (float)r.az * _accel_lsb;
    d.gx   = (float)r.gx * _gyro_lsb - _gbx;
    d.gy   = (float)r.gy * _gyro_lsb - _gby;
    d.gz   = (float)r.gz * _gyro_lsb - _gbz;
    d.temp = (float)r.temp / QMI8658_TEMP_LSB + QMI8658_TEMP_OFFSET;
    return d;
}

QMI8658::DataRaw QMI8658::readRaw()
{
    DataRaw raw = {};
    uint8_t buf[14] = {0};

    if (!readRegs(QMI8658_REG_TEMP_L, buf, 14)) {
        ESP_LOGE(TAG, "SPI 连续读取失败!");
        return raw;
    }

    raw.temp = (int16_t)((buf[1]  << 8) | buf[0]);
    raw.ax   = (int16_t)((buf[3]  << 8) | buf[2]);
    raw.ay   = (int16_t)((buf[5]  << 8) | buf[4]);
    raw.az   = (int16_t)((buf[7]  << 8) | buf[6]);
    raw.gx   = (int16_t)((buf[9]  << 8) | buf[8]);
    raw.gy   = (int16_t)((buf[11] << 8) | buf[10]);
    raw.gz   = (int16_t)((buf[13] << 8) | buf[12]);

    return raw;
}

float QMI8658::readTemp()
{
    uint8_t buf[2];
    if (!readRegs(QMI8658_REG_TEMP_L, buf, 2)) return 0;
    return (float)(int16_t)((buf[0] << 8) | buf[1]) / QMI8658_TEMP_LSB
           + QMI8658_TEMP_OFFSET;
}

void QMI8658::calibrate(uint16_t samples)
{
    double sx = 0, sy = 0, sz = 0;
    ESP_LOGI(TAG, "陀螺仪校准 (%d 采样)...", samples);
    for (int i = 0; i < 10; i++) { readRaw(); vTaskDelay(pdMS_TO_TICKS(10)); }

    for (uint16_t i = 0; i < samples; i++) {
        DataRaw r = readRaw();
        sx += r.gx * _gyro_lsb;
        sy += r.gy * _gyro_lsb;
        sz += r.gz * _gyro_lsb;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    _gbx = (float)(sx / samples);
    _gby = (float)(sy / samples);
    _gbz = (float)(sz / samples);
    ESP_LOGI(TAG, "校准完成: (%.4f, %.4f, %.4f) dps", _gbx, _gby, _gbz);
}

/* ================================================================ */
/* SPI 读写 — 软件 CS 强控 + CPHA=1 错峰采样                         */
/* ================================================================ */

bool QMI8658::writeReg(uint8_t reg, uint8_t val)
{
    /* tx_data[0] 先发 (低字节优先, ESP-IDF 文档明确) */
    spi_transaction_t t = {};
    t.flags      = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    t.length     = 16;
    t.tx_data[0] = (uint8_t)(reg & 0x7F);   // 第一字节: 地址 + 写标志
    t.tx_data[1] = val;                      // 第二字节: 数据
    t.tx_data[2] = 0x00;
    t.tx_data[3] = 0x00;

    if (_spi_mutex) xSemaphoreTake(_spi_mutex, portMAX_DELAY);
    esp_err_t ret = spi_device_transmit(_spi, &t);
    if (_spi_mutex) xSemaphoreGive(_spi_mutex);

    ESP_LOGI(TAG, "  写 0x%02X=0x%02X → MISO rx[0]=0x%02X rx[1]=0x%02X",
             reg, val, t.rx_data[0], t.rx_data[1]);
    return ret == ESP_OK;
}

bool QMI8658::readRegs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (len > 14) return false;

    uint8_t tx[18] = {0};
    uint8_t rx[18] = {0};
    tx[0] = (reg | QMI8658_SPI_READ);

    spi_transaction_t t = {};
    t.flags     = 0;
    t.length    = (1 + len) * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    if (_spi_mutex) xSemaphoreTake(_spi_mutex, portMAX_DELAY);
    esp_err_t ret = spi_device_transmit(_spi, &t);
    if (_spi_mutex) xSemaphoreGive(_spi_mutex);

    std::memcpy(buf, rx + 1, len);
    return ret == ESP_OK;
}

bool QMI8658::readReg(uint8_t reg, uint8_t &val)
{
    /* tx_data[0] 先发 (低字节优先, ESP-IDF 文档明确) */
    spi_transaction_t t = {};
    t.flags      = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    t.length     = 16;
    t.tx_data[0] = (uint8_t)(reg | QMI8658_SPI_READ); // 第一字节: 地址 + 读标志
    t.tx_data[1] = 0x00;                                // 第二字节: dummy
    t.tx_data[2] = 0x00;
    t.tx_data[3] = 0x00;

    if (_spi_mutex) xSemaphoreTake(_spi_mutex, portMAX_DELAY);
    esp_err_t ret = spi_device_transmit(_spi, &t);
    if (_spi_mutex) xSemaphoreGive(_spi_mutex);

    val = t.rx_data[1];   // rx_data[1] = 第二字节 (芯片在该字节输出寄存器值)
    return ret == ESP_OK;
}

/* ================================================================ */

void QMI8658::scanRegisters()
{
    ESP_LOGI(TAG, "===== QMI8658 寄存器扫描 (软件CS) =====");

    /* SPI 帧诊断 */
    ESP_LOGI(TAG, "-- SPI 帧诊断 (reg 0x00) --");
    for (int bits = 8; bits <= 32; bits += 8) {
        spi_transaction_t t = {};
        t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
        t.length = bits;
        t.tx_data[0] = (0x00 | 0x80);  // 第一字节: 读 WHO_AM_I
        t.tx_data[1] = 0x00;           // 第二字节: dummy
        spi_device_transmit(_spi, &t);
        ESP_LOGI(TAG, "  %d-bit: rx[0]=0x%02X [1]=0x%02X [2]=0x%02X [3]=0x%02X",
                 bits, t.rx_data[0], t.rx_data[1], t.rx_data[2], t.rx_data[3]);
    }

    /* 写-读回测试: CTRL2 */
    uint8_t orig = 0;
    readReg(QMI8658_REG_CTRL2, orig);
    ESP_LOGI(TAG, "CTRL2(0x03) 原值=0x%02X → 写0xA5", orig);

    writeReg(QMI8658_REG_CTRL2, 0xA5);
    vTaskDelay(pdMS_TO_TICKS(2));

    uint8_t test = 0;
    readReg(QMI8658_REG_CTRL2, test);
    ESP_LOGI(TAG, "CTRL2 读回=0x%02X %s", test,
             (test == 0xA5) ? "✓ SPI 写入正常!" : "✗ SPI 写入失败");

    writeReg(QMI8658_REG_CTRL2, orig);

    /* 全地址 dump */
    ESP_LOGI(TAG, "Addr: [rx0 rx1] 每行16个寄存器");
    for (int addr = 0; addr < 128; addr += 16) {
        char l0[80] = {0}, l1[80] = {0};
        int o0 = snprintf(l0, sizeof(l0), "%02X(r0):", addr);
        int o1 = snprintf(l1, sizeof(l1), "%02X(r1):", addr);
        for (int i = 0; i < 16; i++) {
            spi_transaction_t t = {};
            t.flags      = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
            t.length     = 16;
            t.tx_data[0] = (uint8_t)((addr + i) | 0x80); // 第一字节: 读地址
            t.tx_data[1] = 0x00;                          // 第二字节: dummy
            spi_device_transmit(_spi, &t);
            o0 += snprintf(l0 + o0, sizeof(l0) - o0, "%02X ", t.rx_data[0]);
            o1 += snprintf(l1 + o1, sizeof(l1) - o1, "%02X ", t.rx_data[1]);
        }
        ESP_LOGI(TAG, "%s", l0);
        ESP_LOGI(TAG, "%s", l1);
    }
    ESP_LOGI(TAG, "===== 扫描完成 =====");
}

void QMI8658::_applyScale()
{
    _accel_lsb = 9.80665f / QMI8658_ACC_LSB_2G;
    _gyro_lsb  = 1.0f / QMI8658_GYRO_LSB_2048DPS;
}
