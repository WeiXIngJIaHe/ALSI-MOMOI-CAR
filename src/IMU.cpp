/* ================================================================
 * LSM6DSV16X — 完整 SPI 驱动 (ESP-IDF 5.5 / C++)
 *
 * 功能:
 *   1. 原始 6 轴传感器数据 + 温度  (100Hz ~ 7680Hz)
 *   2. SFLP 内置 6 轴融合 → Game Rotation Vector (四元数)
 *   3. 陀螺仪零偏自动校准
 *   4. FIFO 流模式读取
 *
 * SPI 协议:
 *   命令字节 = (reg<<1) | R/W   (bit0=1 读, bit0=0 写)
 *   Mode 0 (CPOL=0, CPHA=0) — ST 传感器默认
 *   IF_INC=1 支持多字节连续读取 (地址自增)
 *
 * DMA:
 *   总线初始化时 SPI_DMA_CH_AUTO 启用 DMA
 *   读缓冲: DRAM_ATTR 静态对齐, 长度补齐 4B 倍数
 *   写操作: 4B TXDATA/RXDATA 内联 (无需 DMA 缓冲)
 * ================================================================ */

#include "IMU.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <cstring>
#include <cmath>

static const char *TAG = "LSM6DSV16X";

/* ── 前置声明 ── */
static float _halfToFloat(uint16_t h);

/* ── DMA-safe buffer pool (DRAM + 32-bit aligned, 4-byte multiple) ── */
#define DMA_POOL_SIZE  20

static DRAM_ATTR uint8_t __attribute__((aligned(4))) _dma_tx[DMA_POOL_SIZE];
static DRAM_ATTR uint8_t __attribute__((aligned(4))) _dma_rx[DMA_POOL_SIZE];

/* FreeRTOS mutex — SPI shared between Core0 (sensor) and Core1 (UART cmd)
 * DO NOT use portENTER_CRITICAL here: it disables interrupts,
 * which blocks SPI DMA completion → crash */
static SemaphoreHandle_t _spi_mutex = NULL;

static inline void _spi_lock_acquire() {
    if (_spi_mutex) xSemaphoreTake(_spi_mutex, portMAX_DELAY);
}
static inline void _spi_lock_release() {
    if (_spi_mutex) xSemaphoreGive(_spi_mutex);
}

/* ── 静态: 总线只初始化一次 ── */
static bool _spi_ready = false;

static void _spi_ensure()
{
    if (_spi_ready) return;

    /* CS 先拉低再初始化 SPI — LSM6DSV16X 上电时检测 CS 电平选接口:
     * CS=HIGH → I2C 模式, CS=LOW → SPI 模式. 必须在总线初始化前拉低 */
    gpio_set_direction((gpio_num_t)IMU_CS, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)IMU_CS, 0);
    vTaskDelay(pdMS_TO_TICKS(1));  /* 给芯片 1ms 锁定 SPI 模式 */

    spi_bus_config_t cfg = {};
    cfg.mosi_io_num     = IMU_MOSI;
    cfg.miso_io_num     = IMU_MISO;
    cfg.sclk_io_num     = IMU_SCLK;
    cfg.quadwp_io_num   = -1;
    cfg.quadhd_io_num   = -1;
    cfg.max_transfer_sz = 4096;
    cfg.flags           = 0;
    spi_bus_initialize(IMU_SPI_HOST, &cfg, SPI_DMA_CH_AUTO);

    if (!_spi_mutex) _spi_mutex = xSemaphoreCreateMutex();

    _spi_ready = true;
    ESP_LOGI(TAG, "SPI bus: SCLK=%d MOSI=%d MISO=%d CS=%d @ %dHz",
             IMU_SCLK, IMU_MOSI, IMU_MISO, IMU_CS, (int)IMU_SPI_CLK_HZ);
}

/* ================================================================
 * 构造 / 析构
 * ================================================================ */

LSM6DSV16X::LSM6DSV16X()
    : _spi(nullptr),
      _accel_lsb(LSM_ACCEL_LSB_2G * 0.001f * 9.80665f),  /* mg→m/s² */
      _gyro_lsb(LSM_GYRO_LSB_2000DPS * 0.001f),          /* mdps→dps */
      _gbx(0), _gby(0), _gbz(0)
{}

LSM6DSV16X::~LSM6DSV16X()
{
    if (_spi) spi_bus_remove_device(_spi);
}

/* ── 全寄存器扫描 (诊断用, 打印所有可读寄存器值) ── */
static void _dumpRegs(spi_device_handle_t spi)
{
    ESP_LOGI(TAG, "===== 寄存器扫描 =====");

    struct { uint8_t addr; const char *name; } regs[] = {
        {0x01, "FUNC_CFG_ACCESS"}, {0x02, "PIN_CTRL"}, {0x03, "IF_CFG"},
        {0x04, "EMB_FUNC_EN_A"},   {0x05, "EMB_FUNC_EN_B"},
        {0x07, "FIFO_CTRL1"},      {0x08, "FIFO_CTRL2"},
        {0x09, "FIFO_CTRL3"},      {0x0A, "FIFO_CTRL4"},
        {0x0D, "INT1_CTRL"},       {0x0E, "INT2_CTRL"},
        {0x0F, "WHO_AM_I"},        {0x10, "CTRL1"},
        {0x11, "CTRL2"},           {0x12, "CTRL3"},
        {0x13, "CTRL4"},           {0x14, "CTRL5"},
        {0x15, "CTRL6"},           {0x16, "CTRL7"},
        {0x17, "CTRL8"},           {0x18, "CTRL9"},
        {0x19, "CTRL10"},          {0x1A, "CTRL_STATUS"},
        {0x1B, "FIFO_STATUS1"},    {0x1C, "FIFO_STATUS2"},
        {0x1D, "ALL_INT_SRC"},     {0x1E, "STATUS_REG"},
        {0x20, "OUT_TEMP_L"},      {0x21, "OUT_TEMP_H"},
    };

    for (auto &r : regs) {
        spi_transaction_t t = {};
        t.flags      = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
        t.length     = 16;
        t.tx_data[0] = (uint8_t)((r.addr << 1) | 0x01);
        t.tx_data[1] = 0x00;
        spi_device_transmit(spi, &t);
        ESP_LOGI(TAG, "  0x%02X %-16s = 0x%02X", r.addr, r.name, t.rx_data[1]);
    }
    ESP_LOGI(TAG, "===== 扫描结束 =====");
}

bool LSM6DSV16X::begin()
{
    _spi_ensure();

    spi_device_interface_config_t dev_cfg = {};
    dev_cfg.mode             = 3;              // LSM6DSV16X: CPOL=1 CPHA=1
    dev_cfg.clock_speed_hz   = IMU_SPI_CLK_HZ;
    dev_cfg.spics_io_num     = IMU_CS;
    dev_cfg.queue_size       = 1;
    // cs_ena_pretrans/posttrans 仅在半双工模式有效, 全双工由硬件 FSM 控制 CS

    if (spi_bus_add_device(IMU_SPI_HOST, &dev_cfg, &_spi) != ESP_OK)
        return false;

    /* 全寄存器扫描 — 诊断 SPI 通信 */
    _dumpRegs(_spi);

    /* 软件复位 */
    writeReg(LSM_CTRL3, LSM_CTRL3_SW_RESET);
    vTaskDelay(pdMS_TO_TICKS(20));

    /* WHO_AM_I 校验 */
    uint8_t id = whoAmI();
    ESP_LOGI(TAG, "WHO_AM_I=0x%02X %s", id, (id == LSM_WHO_AM_I_VAL) ? "OK" : "?");

    /* 基础配置 */
    writeReg(LSM_CTRL3, LSM_CTRL3_BDU | LSM_CTRL3_IF_INC);
    writeReg(LSM_CTRL1, LSM_ODR_XL_120HZ);
    writeReg(LSM_CTRL2, LSM_ODR_G_120HZ);
    writeReg(LSM_CTRL6, LSM_FS_G_2000DPS);
    writeReg(LSM_CTRL8, LSM_FS_XL_2G);

    _applyScale();
    ESP_LOGI(TAG, "初始化完成: ACC=±2g GYRO=±2000dps ODR=120Hz");
    return true;
}

uint8_t LSM6DSV16X::whoAmI()
{
    uint8_t val = 0;
    readReg(LSM_WHO_AM_I, val);
    return val;
}

bool LSM6DSV16X::reset()
{
    writeReg(LSM_CTRL3, LSM_CTRL3_SW_RESET);
    vTaskDelay(pdMS_TO_TICKS(20));
    return true;
}

/* ================================================================
 * 原始传感器读取
 * ================================================================ */

LSM6DSV16X::Data3F LSM6DSV16X::readAll()
{
    DataRaw r = readRaw();
    Data3F  d;
    d.ax   = (float)r.ax * _accel_lsb;
    d.ay   = (float)r.ay * _accel_lsb;
    d.az   = (float)r.az * _accel_lsb;
    d.gx   = (float)r.gx * _gyro_lsb - _gbx;
    d.gy   = (float)r.gy * _gyro_lsb - _gby;
    d.gz   = (float)r.gz * _gyro_lsb - _gbz;
    d.temp = (float)r.temp / LSM_TEMP_LSB + LSM_TEMP_OFFSET;
    return d;
}

LSM6DSV16X::DataRaw LSM6DSV16X::readRaw()
{
    DataRaw raw = {};

    /* ── DMA 突发: 单次 14 字节 (TEMP 0x20 → ACCEL_Z 0x2D)
     *     readRegs 内部使用 DRAM_ATTR 对齐缓冲, 自动补齐 4B ── */
    uint8_t buf[14];
    if (!readRegs(LSM_OUT_TEMP_L, buf, 14)) return raw;

    /* 小端解析 */
    raw.temp = (int16_t)((buf[1]  << 8) | buf[0]);       /* 0x20-0x21 */
    raw.gx   = (int16_t)((buf[3]  << 8) | buf[2]);       /* 0x22-0x23 */
    raw.gy   = (int16_t)((buf[5]  << 8) | buf[4]);       /* 0x24-0x25 */
    raw.gz   = (int16_t)((buf[7]  << 8) | buf[6]);       /* 0x26-0x27 */
    raw.ax   = (int16_t)((buf[9]  << 8) | buf[8]);       /* 0x28-0x29 */
    raw.ay   = (int16_t)((buf[11] << 8) | buf[10]);      /* 0x2A-0x2B */
    raw.az   = (int16_t)((buf[13] << 8) | buf[12]);      /* 0x2C-0x2D */

    return raw;
}

float LSM6DSV16X::readTemp()
{
    uint8_t buf[2];
    if (!readRegs(LSM_OUT_TEMP_L, buf, 2)) return 0;
    return (float)(int16_t)((buf[1] << 8) | buf[0]) / LSM_TEMP_LSB
           + LSM_TEMP_OFFSET;
}

/* ================================================================
 * 陀螺仪零偏校准
 * ================================================================ */

void LSM6DSV16X::calibrate(uint16_t samples)
{
    double sx = 0, sy = 0, sz = 0;
    ESP_LOGI(TAG, "陀螺仪校准 %d 采样...", samples);

    /* 先丢弃前 10 个样本让传感器稳定 */
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
    ESP_LOGI(TAG, "校准完成 bias=(%.4f, %.4f, %.4f) dps", _gbx, _gby, _gbz);
}

/* ================================================================
 * SFLP — 内置 6 轴姿态融合 (Game Rotation Vector)
 *
 * 输出: 四元数 (X,Y,Z,W) @ 15~480Hz
 *
 * 配置步骤:
 *   1. FUNC_CFG_ACCESS bit7=1  (解锁嵌入式寄存器)
 *   2. 设置 SFLP ODR (≤ 传感器 ODR)
 *   3. EMB_FUNC_FIFO_EN_A bit1=1 (四元数进 FIFO)
 *   4. FIFO_CTRL4 设为流模式
 *   5. EMB_FUNC_EN_A bit1=1 (启动 SFLP)
 * ================================================================ */

bool LSM6DSV16X::sflpBegin(uint8_t odr)
{
    /* ── 1. 解锁嵌入式功能寄存器 ── */
    _setEmbAccess(true);

    /* ── 2. 设置 SFLP ODR ── */
    writeReg(LSM_SFLP_GAME_ODR, (odr & 0x07));

    /* ── 3. 使能 Game Rotation Vector → FIFO ── */
    writeReg(LSM_EMB_FUNC_FIFO_EN_A, LSM_SFLP_GAME_FIFO_EN);

    /* ── 4. FIFO 流模式 ── */
    writeReg(LSM_FIFO_CTRL4, LSM_FIFO_MODE_CONTINUOUS);

    /* ── 5. 启动 SFLP Game Rotation Vector ── */
    writeReg(LSM_EMB_FUNC_EN_A, LSM_SFLP_GAME_EN);

    /* ── 6. 锁回嵌入式寄存器 ── */
    _setEmbAccess(false);

    _sflp_enabled = true;
    ESP_LOGI(TAG, "SFLP 启动: Game Rotation Vector ODR=%d", odr);
    return true;
}

bool LSM6DSV16X::sflpIsReady()
{
    /* FIFO 中有数据即就绪 (不读 TAG, 避免副作用) */
    return (_readFifoCount() > 0);
}

LSM6DSV16X::SFLPData LSM6DSV16X::sflpRead()
{
    SFLPData d = {};
    if (!_sflp_enabled) return d;

    /* ── 1. 读 FIFO 水位, 确保有足够数据 ── */
    uint16_t count = _readFifoCount();
    if (count == 0) return d;

    /* ── 2. 读 TAG ── */
    uint8_t tag = _readFifoTag();

    /* ── 3. 根据 TAG 解析 ── */
    switch (tag) {
    case LSM_SFLP_GAME_ROTATION_VECTOR_TAG:   /* 0x13: 四元数 */
        {
            uint8_t buf[6];
            if (!_readFifoData(buf, 6)) return d;
            /* 字节序: XL, YL, ZL, XH, YH, ZH */
            uint16_t hx = ((uint16_t)buf[3] << 8) | buf[0];
            uint16_t hy = ((uint16_t)buf[4] << 8) | buf[1];
            uint16_t hz = ((uint16_t)buf[5] << 8) | buf[2];
            d.quat.qx = _halfToFloat(hx);
            d.quat.qy = _halfToFloat(hy);
            d.quat.qz = _halfToFloat(hz);
            float sq = d.quat.qx * d.quat.qx
                     + d.quat.qy * d.quat.qy
                     + d.quat.qz * d.quat.qz;
            d.quat.qw = (sq <= 1.0f) ? std::sqrt(1.0f - sq) : 0.0f;
            break;
        }
    case LSM_SFLP_GRAVITY_VECTOR_TAG:         /* 0x14: 重力向量 */
        {
            uint8_t buf[6];
            if (!_readFifoData(buf, 6)) return d;
            uint16_t hx = ((uint16_t)buf[3] << 8) | buf[0];
            uint16_t hy = ((uint16_t)buf[4] << 8) | buf[1];
            uint16_t hz = ((uint16_t)buf[5] << 8) | buf[2];
            d.gravity[0] = _halfToFloat(hx);
            d.gravity[1] = _halfToFloat(hy);
            d.gravity[2] = _halfToFloat(hz);
            break;
        }
    case LSM_SFLP_GYROSCOPE_BIAS_TAG:         /* 0x15: 陀螺仪零偏 */
        {
            uint8_t buf[6];
            if (!_readFifoData(buf, 6)) return d;
            int16_t gbx = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
            int16_t gby = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
            int16_t gbz = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);
            d.gbias[0] = (float)gbx * LSM_GYRO_LSB_2000DPS * 0.001f;
            d.gbias[1] = (float)gby * LSM_GYRO_LSB_2000DPS * 0.001f;
            d.gbias[2] = (float)gbz * LSM_GYRO_LSB_2000DPS * 0.001f;
            break;
        }
    default:
        /* 未知 TAG: 清空该条目 (读 6 字节丢弃, 防止 FIFO 阻塞) */
        {
            uint8_t dummy[6];
            _readFifoData(dummy, 6);
            break;
        }
    }

    return d;
}

/* ── SPI 读写 — LSM6DSV16X 协议 (bit0=R/W) ──
 * 写: cmd=(reg<<1)&0xFE, 只用 TXDATA
 * 读: cmd=(reg<<1)|0x01, TXDATA|RXDATA, 结果在 rx[1]
 * 互斥锁: FreeRTOS mutex (不禁中断, 避免 DMA 死锁) */

bool LSM6DSV16X::writeReg(uint8_t reg, uint8_t val)
{
    if (!_spi) return false;

    _spi_lock_acquire();
    std::memset(_dma_tx, 0, 4);
    
    // bit7 = 0 为写，寄存器地址不移位
    _dma_tx[0] = reg & 0x7F;  
    _dma_tx[1] = val;

    spi_transaction_t t = {};
    t.length    = 16;           // 发送 2 字节 = 16 bits
    t.tx_buffer = _dma_tx;      // 使用字节流 buffer，强制顺序发送

    esp_err_t ret = spi_device_transmit(_spi, &t);
    _spi_lock_release();
    return ret == ESP_OK;
}

bool LSM6DSV16X::readReg(uint8_t reg, uint8_t &val)
{
    if (!_spi) return false;

    _spi_lock_acquire();
    std::memset(_dma_tx, 0, 4);
    std::memset(_dma_rx, 0, 4);
    
    // bit7 = 1 为读，寄存器地址不移位
    _dma_tx[0] = reg | 0x80; 
    _dma_tx[1] = 0x00;          // Dummy 空字节

    spi_transaction_t t = {};
    t.length    = 16;
    t.tx_buffer = _dma_tx;
    t.rx_buffer = _dma_rx;

    esp_err_t ret = spi_device_transmit(_spi, &t);
    val = _dma_rx[1];           // 第0字节是废数据，有效数据在第1字节
    _spi_lock_release();
    return ret == ESP_OK;
}

bool LSM6DSV16X::readRegs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (!_spi || !buf || len == 0) return false;
    
    // DMA 缓存保护：命令字节(1) + 数据字节(len) 不能超过 DMA_POOL_SIZE
    if (len + 1 > DMA_POOL_SIZE) return false;

    _spi_lock_acquire();
    
    // 向上补齐到 4 字节的倍数，满足 ESP32 DMA 硬件要求
    uint8_t total = (uint8_t)((len + 1 + 3) & ~3u);
    
    std::memset(_dma_tx, 0, total);
    std::memset(_dma_rx, 0, total);

    // bit7 = 1 为读
    _dma_tx[0] = reg | 0x80; 

    spi_transaction_t t = {};
    t.length    = total * 8;    // 传输总 bit 数
    t.tx_buffer = _dma_tx;
    t.rx_buffer = _dma_rx;

    esp_err_t ret = spi_device_transmit(_spi, &t);
    
    // 掐头去尾：跳过第0字节(命令返回)，将 len 长度的有效数据拷给用户
    std::memcpy(buf, _dma_rx + 1, len);
    
    _spi_lock_release();
    return ret == ESP_OK;
}

/* ================================================================
 * 内部辅助函数
 * ================================================================ */

void LSM6DSV16X::_applyScale()
{
    /* 加速度计: mg/LSB → m/s²  (0.061 mg * 0.001 g/mg * 9.80665 m/s²/g) */
    _accel_lsb = LSM_ACCEL_LSB_2G * 0.001f * 9.80665f;
    /* 陀螺仪: mdps/LSB → dps */
    _gyro_lsb  = LSM_GYRO_LSB_2000DPS * 0.001f;
}

void LSM6DSV16X::_setEmbAccess(bool enable)
{
    uint8_t val = enable ? LSM_FUNC_CFG_EMB_ACCESS : 0x00;
    writeReg(LSM_FUNC_CFG_ACCESS, val);
}

uint16_t LSM6DSV16X::_readFifoCount()
{
    uint8_t l, h;
    readReg(LSM_FIFO_STATUS1, l);
    readReg(LSM_FIFO_STATUS2, h);
    return ((uint16_t)(h & 0x01) << 8) | l;
}

uint8_t LSM6DSV16X::_readFifoTag()
{
    uint8_t tag = 0;
    readReg(LSM_FIFO_DATA_OUT_TAG, tag);
    return tag;
}

bool LSM6DSV16X::_readFifoData(uint8_t *buf, uint8_t len)
{
    return readRegs(LSM_FIFO_DATA_OUT_X_L, buf, len);
}

/* ── IEEE 754 half-precision float → float32 ──
 * binary16 布局: bit[15]符号 bit[14:10]指数(bias=15) bit[9:0]尾数
 * 移植自 ST 官方 lsm6dsv16x_reg.c (BSD-3-Clause)
 * SFLP 四元数 XYZ 分量以 half-float 存入 FIFO, W=√(1-X²-Y²-Z²) */

static uint32_t _ToFloatBits(uint16_t h)
{
    uint16_t h_exp = (h & 0x7C00u);
    uint32_t f_sgn = ((uint32_t)h & 0x8000u) << 16u;
    uint32_t result = 0;

    switch (h_exp) {
    case 0x0000u: {  /* 0 或 subnormal */
        uint16_t h_sig = (h & 0x03FFu);
        if (h_sig == 0u) {
            result = f_sgn;       /* ±0 */
            break;
        }
        /* 规整化 subnormal */
        h_sig <<= 1u;
        while ((h_sig & 0x0400u) == 0u) {
            h_sig <<= 1u;
            h_exp++;
        }
        uint32_t f_exp = ((uint32_t)(127u - 15u - (uint32_t)h_exp)) << 23u;
        uint32_t f_sig = ((uint32_t)(h_sig & 0x03FFu)) << 13u;
        result = f_sgn + f_exp + f_sig;
        break;
    }
    case 0x7C00u:  /* inf 或 NaN */
        result = f_sgn + 0x7F800000u + (((uint32_t)(h & 0x03FFu)) << 13u);
        break;
    default:  /* normalized */
        result = f_sgn + (((uint32_t)(h & 0x7FFFu) + 0x1C000u) << 13u);
        break;
    }
    return result;
}

static float _halfToFloat(uint16_t h)
{
    uint32_t bits = _ToFloatBits(h);
    float f;
    std::memcpy(&f, &bits, sizeof(f));
    return f;
}
