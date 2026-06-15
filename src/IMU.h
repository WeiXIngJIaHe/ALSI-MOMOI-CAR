#pragma once
#include <stdint.h>
#include "driver/spi_master.h"

/* ================================================================
 * QMI8658C 6 轴 IMU — 4 线 SPI 驱动
 *
 * WHO_AM_I = 0x05 (寄存器 0x00)
 * SPI: bit7=R/W, bit[6:0]=地址, Mode 0/3
 * ================================================================ */

#define QMI8658_CS                  10
#define QMI8658_SCLK                9
#define QMI8658_MOSI                8
#define QMI8658_MISO                7
#define QMI8658_INT_PIN             6
#define QMI8658_SPI_HOST            SPI2_HOST
#define QMI8658_SPI_CLK_HZ          5000000


#define QMI8658_REG_WHO_AM_I        0x00
#define QMI8658_WHO_AM_I_VAL        0x3E  // 本芯片实际 
#define QMI8658_SPI_READ            0x80
#define QMI8658_SPI_WRITE           0x00

#define QMI8658_REG_REVISION        0x01    // 硅片版本号

/* 核心控制寄存器 */
#define QMI8658_REG_CTRL1           0x02    // [7]系统SPI配置 [6]地址自增 [0]软复位
#define QMI8658_REG_CTRL2           0x03    // [7:4]ACC_FS量程 [3:0]ACC_ODR速率
#define QMI8658_REG_CTRL3           0x04    // [7:4]GYRO_FS量程 [3:0]GYRO_ODR速率
#define QMI8658_REG_CTRL4           0x05    // (保留，通常不用)
#define QMI8658_REG_CTRL5           0x06    // [7:4]内部低通滤波LPF配置
#define QMI8658_REG_CTRL6           0x07    // (保留，通常不用)
#define QMI8658_REG_CTRL7           0x08    // 【终极开关】[7]GYRO_EN [3]ACC_EN
#define QMI8658_REG_CTRL8           0x09    // 协处理器命令 (不要随便往这里写东西！)
#define QMI8658_REG_CTRL9           0x0A    // 主机命令

/* 传感器数据 FIFO 起始地址 (标准小端模式) */
/* 连续读取顺序：温度 -> 加速度X/Y/Z -> 陀螺仪X/Y/Z */
#define QMI8658_REG_TEMP_L          0x33    // 温度低八位
#define QMI8658_REG_TEMP_H          0x34    // 温度高八位
#define QMI8658_REG_AX_L            0x35    // 加速度 X 低八位
#define QMI8658_REG_AX_H            0x36
#define QMI8658_REG_AY_L            0x37    // 加速度 Y 低八位
#define QMI8658_REG_AY_H            0x38
#define QMI8658_REG_AZ_L            0x39    // 加速度 Z 低八位
#define QMI8658_REG_AZ_H            0x3A
#define QMI8658_REG_GX_L            0x3B    // 陀螺仪 X 低八位
#define QMI8658_REG_GX_H            0x3C
#define QMI8658_REG_GY_L            0x3D    // 陀螺仪 Y 低八位
#define QMI8658_REG_GY_H            0x3E
#define QMI8658_REG_GZ_L            0x3F    // 陀螺仪 Z 低八位
#define QMI8658_REG_GZ_H            0x40    // 陀螺仪 Z 高八位

/* 状态与中断 */
#define QMI8658_REG_STATUS0         0x2C    // Bit0: 内部数据就绪标志
#define QMI8658_REG_STATUS1         0x2D

/* CTRL1 */
#define QMI8658_CTRL1_SRST          (1 << 0)

/* CTRL7 */
#define QMI8658_CTRL7_EN_ACC        (1 << 3)
#define QMI8658_CTRL7_EN_GYRO       (1 << 7)
#define QMI8658_CTRL7_ACC_MODE_LN   0x03
#define QMI8658_CTRL7_GYRO_MODE_LN  (0x03 << 4)

/* STATUSINT */
#define QMI8658_ST_ACC_DRDY         (1 << 0)
#define QMI8658_ST_GYRO_DRDY        (1 << 1)

/* 量程 */
#define QMI8658_ACC_LSB_2G          (16384.0f)
#define QMI8658_ACC_LSB_4G          (8192.0f)
#define QMI8658_ACC_LSB_8G          (4096.0f)
#define QMI8658_ACC_LSB_16G         (2048.0f)

#define QMI8658_GYRO_LSB_16DPS      (2048.0f)
#define QMI8658_GYRO_LSB_32DPS      (1024.0f)
#define QMI8658_GYRO_LSB_64DPS      (512.0f)
#define QMI8658_GYRO_LSB_128DPS     (256.0f)
#define QMI8658_GYRO_LSB_256DPS     (128.0f)
#define QMI8658_GYRO_LSB_512DPS     (64.0f)
#define QMI8658_GYRO_LSB_1024DPS    (32.0f)
#define QMI8658_GYRO_LSB_2048DPS    (16.0f)

#define QMI8658_TEMP_LSB            256.0f
#define QMI8658_TEMP_OFFSET         25.0f

/* ================================================================ */

class QMI8658 {
public:
    struct Data3F  { float ax, ay, az, gx, gy, gz, temp; };
    struct DataRaw { int16_t ax, ay, az, gx, gy, gz, temp; };

    QMI8658();
    ~QMI8658();

    bool    begin();
    uint8_t whoAmI();
    void    scanRegisters();

    Data3F  readAll();
    DataRaw readRaw();
    float   readTemp();
    void    calibrate(uint16_t samples = 200);

    float   gyroBiasX() const { return _gbx; }
    float   gyroBiasY() const { return _gby; }
    float   gyroBiasZ() const { return _gbz; }

    bool writeReg(uint8_t reg, uint8_t val);
    bool readReg(uint8_t reg, uint8_t &val);
    bool readRegs(uint8_t reg, uint8_t *buf, uint8_t len);

private:
    void _applyScale();

    spi_device_handle_t _spi;
    float _accel_lsb;
    float _gyro_lsb;
    float _gbx, _gby, _gbz;
};
