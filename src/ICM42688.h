#pragma once
#include <stdint.h>
#include "driver/spi_master.h"

/* ================================================================
 * ICM-42688-P 6轴 IMU (加速度 + 陀螺仪) — SPI 驱动
 * SPI 总线: SPI2_HOST, 独立于 I2C 设备
 *
 * ICM-42688 SPI 协议:
 *   命令字节 bit[7:1]=寄存器地址, bit[0]=R/W (0=写, 1=读)
 *   写: CS↓ → [reg<<1|0] [data] → CS↑
 *   读: CS↓ → [reg<<1|1] [dummy] → CS↑  (接收数据在 dummy 位之后)
 *   模式: CPOL=0, CPHA=0 (SPI Mode 0)
 *   最大速率: 24 MHz
 * ================================================================ */

/* ---- SPI 引脚 (请填写) ---- */
#define ICM42688_CS                 10
#define ICM42688_SCLK               12
#define ICM42688_MOSI               11
#define ICM42688_MISO               13
#define ICM42688_SPI_HOST           SPI2_HOST
#define ICM42688_SPI_CLK_HZ         24000000   /* 24 MHz */
#define ICM42688_INT1_PIN           4
#define ICM42688_INT2_PIN           3

/* ---- SPI 协议 ---- */
#define ICM42688_SPI_READ_BIT       0x80      /* bit0=1 → 读 */
#define ICM42688_SPI_WRITE_BIT      0x00      /* bit0=0 → 写 */

/* ---- WHO_AM_I ---- */
#define ICM42688_WHO_AM_I_VAL       0x47

/* ================================================================
 * 寄存器 (Bank 0)
 * ================================================================ */
#define ICM42688_REG_DEVICE_CONFIG  0x11
#define ICM42688_REG_DRIVE_CONFIG   0x13
#define ICM42688_REG_INT_CONFIG     0x14
#define ICM42688_REG_FIFO_CONFIG    0x16
#define ICM42688_REG_TEMP_DATA1     0x1D
#define ICM42688_REG_ACCEL_DATA_X1  0x1F
#define ICM42688_REG_GYRO_DATA_X1   0x25
#define ICM42688_REG_PWR_MGMT0      0x4E
#define ICM42688_REG_GYRO_CONFIG0   0x4F
#define ICM42688_REG_ACCEL_CONFIG0  0x50
#define ICM42688_REG_GYRO_CONFIG1   0x51
#define ICM42688_REG_TMST_CONFIG    0x53
#define ICM42688_REG_APEX_CONFIG0   0x56
#define ICM42688_REG_INT_CONFIG1    0x5D
#define ICM42688_REG_FIFO_CONFIG4   0x5E
#define ICM42688_REG_FIFO_CONFIG1   0x5F
#define ICM42688_REG_INT_SOURCE0    0x65
#define ICM42688_REG_WHO_AM_I       0x75
#define ICM42688_REG_BANK_SEL       0x76

/* ---- Bank 1 ---- */
#define ICM42688_REG_INTF_CONFIG0   0x0B
#define ICM42688_REG_INTF_CONFIG1   0x0C

/* ---- Bank 2 ---- */
#define ICM42688_REG_ACCEL_CONFIG1  0x19

/* ---- PWR_MGMT0 ---- */
#define ICM42688_PWR_OFF            0x00
#define ICM42688_PWR_SLEEP          0x01
#define ICM42688_PWR_LOW_NOISE      0x03
#define ICM42688_PWR_MODE_MASK      0x03
#define ICM42688_PWR_TEMP_DIS       (1 << 5)

/* ---- ACCEL_CONFIG0 ---- */
#define ICM42688_ACCEL_ODR_SHIFT    0
#define ICM42688_ACCEL_ODR_MASK     0x0F
#define ICM42688_ACCEL_FS_SHIFT     5
#define ICM42688_ACCEL_FS_MASK      (0x03 << 5)

/* ---- GYRO_CONFIG0 ---- */
#define ICM42688_GYRO_ODR_SHIFT     0
#define ICM42688_GYRO_ODR_MASK      0x0F
#define ICM42688_GYRO_FS_SHIFT      5
#define ICM42688_GYRO_FS_MASK       (0x03 << 5)

/* ---- INT_CONFIG ---- */
#define ICM42688_INT1_MODE_PULSED   0x00
#define ICM42688_INT1_MODE_LATCHED  (1 << 2)
#define ICM42688_INT1_ACTIVE_LOW    0x00
#define ICM42688_INT1_ACTIVE_HIGH   (1 << 1)
#define ICM42688_INT1_PUSH_PULL     0x00
#define ICM42688_INT1_OPEN_DRAIN    (1 << 0)

/* ---- INT_SOURCE0 ---- */
#define ICM42688_INT_DATA_RDY       (1 << 3)
#define ICM42688_INT_FIFO_THS       (1 << 5)
#define ICM42688_INT_FIFO_FULL      (1 << 6)

/* ---- 量程系数 (LSB/g) ---- */
#define ICM42688_ACCEL_LSB_2G       (16384.0f)
#define ICM42688_ACCEL_LSB_4G       (8192.0f)
#define ICM42688_ACCEL_LSB_8G       (4096.0f)
#define ICM42688_ACCEL_LSB_16G      (2048.0f)

/* ---- 量程系数 (LSB/(°/s)) ---- */
#define ICM42688_GYRO_LSB_250DPS    (131.0f)
#define ICM42688_GYRO_LSB_500DPS    (65.5f)
#define ICM42688_GYRO_LSB_1000DPS   (32.8f)
#define ICM42688_GYRO_LSB_2000DPS   (16.4f)

/* ---- 温度 ---- */
#define ICM42688_TEMP_LSB           132.48f
#define ICM42688_TEMP_OFFSET        25.0f

/* ================================================================ */

class ICM42688 {
public:
    enum AccelFS  { ACCEL_2G=0, ACCEL_4G=1, ACCEL_8G=2, ACCEL_16G=3 };
    enum GyroFS   { GYRO_250DPS=0, GYRO_500DPS=1, GYRO_1000DPS=2, GYRO_2000DPS=3 };
    enum ODR {
        ODR_1_5625_HZ=0x08, ODR_6_25_HZ=0x09, ODR_12_5_HZ=0x0A,
        ODR_25_HZ=0x0B, ODR_50_HZ=0x0C, ODR_100_HZ=0x0D, ODR_200_HZ=0x0E,
        ODR_500_HZ=0x0F, ODR_1_KHZ=0x10, ODR_2_KHZ=0x11,
        ODR_4_KHZ=0x12, ODR_8_KHZ=0x13,
    };
    enum PowerMode { POWER_OFF=0, POWER_SLEEP=1, POWER_LOW_NOISE=3 };

    struct Data3F  { float ax, ay, az, gx, gy, gz, temp; };
    struct DataRaw { int16_t ax, ay, az, gx, gy, gz, temp; };

    ICM42688();
    ~ICM42688();

    bool    begin();
    uint8_t whoAmI();

    void setAccelFS(AccelFS fs);
    void setGyroFS(GyroFS fs);
    void setAccelODR(ODR odr);
    void setGyroODR(ODR odr);
    void setPowerMode(PowerMode mode);
    void enableTemp(bool en);

    void configINT1(bool latched, bool active_high, bool open_drain);
    void enableDataReadyINT(bool en);

    Data3F  readAll();
    DataRaw readRaw();
    void    readAccel(float &ax, float &ay, float &az);
    void    readGyro(float &gx, float &gy, float &gz);
    float   readTemp();

    void reset();
    void calibrate(uint16_t samples = 200);

    float gyroBiasX() const { return _gbx; }
    float gyroBiasY() const { return _gby; }
    float gyroBiasZ() const { return _gbz; }

    bool writeReg(uint8_t reg, uint8_t val);
    bool readReg(uint8_t reg, uint8_t &val);
    bool readRegs(uint8_t reg, uint8_t *buf, uint8_t len);

private:
    void _selectBank(uint8_t bank);
    void _applyScale();

    spi_device_handle_t _spi;
    uint8_t _bank;

    float _accel_lsb;
    float _gyro_lsb;
    float _gbx, _gby, _gbz;
};
