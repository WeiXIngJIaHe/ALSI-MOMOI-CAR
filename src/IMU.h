#pragma once
/* ================================================================
 * LSM6DSV16X 6轴 IMU — SPI 驱动 (ESP-IDF 5.5)
 *
 * ST MEMS SPI 协议:
 *   命令字节 = {AD[6:0], R/W}  其中 bit0=1 读, bit0=0 写
 *   读: CS↓ → [(reg<<1)|0x01] → [dummy] → CS↑  数据在 rx[1]
 *   写: CS↓ → [(reg<<1)|0x00] → [data]   → CS↑  只用 TXDATA
 *
 * WHO_AM_I = 0x70 @ 0x0F
 *
 * 功能:
 *   - 原始加速度计 / 陀螺仪 / 温度读取
 *   - SFLP 内置 6 轴姿态融合 → Game Rotation Vector (四元数)
 *   - FIFO 中断 / 轮询读取
 *   - 陀螺仪零偏自动校准
 * ================================================================ */

#include <stdint.h>
#include "driver/spi_master.h"

/* ================================================================
 * SPI 硬件引脚 (与 QMI8658 共用)
 * ================================================================ */
#define IMU_CS          10
#define IMU_SCLK        9
#define IMU_MOSI        8
#define IMU_MISO        7
#define IMU_SPI_HOST    SPI2_HOST
#define IMU_SPI_CLK_HZ  6000000     /* 6MHz — LSM6DSV16X 支持 ≤10MHz */
#define IMU_INT1_PIN    6
#define IMU_INT2_PIN    -1

/* ================================================================
 * LSM6DSV16X 寄存器地址
 * ================================================================ */
/* 基本配置 */
#define LSM_FUNC_CFG_ACCESS     0x01    /* 嵌入式功能访问 */
#define LSM_PIN_CTRL            0x02    /* 引脚控制 */
#define LSM_IF_CFG              0x03    /* 接口配置 */
#define LSM_FIFO_CTRL1          0x07    /* FIFO 水位标记 */
#define LSM_FIFO_CTRL2          0x08    /* FIFO 压缩 / ODR 变更 */
#define LSM_FIFO_CTRL3          0x09    /* FIFO 批量数据速率 */
#define LSM_FIFO_CTRL4          0x0A    /* FIFO 模式选择 */
#define LSM_INT1_CTRL           0x0D    /* INT1 引脚控制 */
#define LSM_INT2_CTRL           0x0E    /* INT2 引脚控制 */
#define LSM_WHO_AM_I            0x0F    /* 设备 ID (只读) */
#define LSM_CTRL1               0x10    /* 加速度计 ODR + 模式 */
#define LSM_CTRL2               0x11    /* 陀螺仪 ODR + 模式 */
#define LSM_CTRL3               0x12    /* 软件复位 / BDU / 自增 */
#define LSM_CTRL4               0x13    /* INT2 配置 */
#define LSM_CTRL5               0x14    /* I3C 配置 */
#define LSM_CTRL6               0x15    /* 陀螺仪 量程 + 低通 */
#define LSM_CTRL7               0x16    /* 陀螺仪 LPF1 使能 */
#define LSM_CTRL8               0x17    /* 加速度计 量程 + 低通 */
#define LSM_CTRL9               0x18    /* 加速度计 LPF2 / HP */
#define LSM_CTRL10              0x19    /* 自检 / 调试 */
#define LSM_STATUS_REG          0x1E    /* 数据就绪标志 */

/* FIFO 状态 */
#define LSM_FIFO_STATUS1        0x1B    /* FIFO 计数 [7:0] */
#define LSM_FIFO_STATUS2        0x1C    /* FIFO 计数 [8] + 标志 */

/* 输出数据 (16位, 小端: 先低后高) */
#define LSM_OUT_TEMP_L          0x20
#define LSM_OUT_TEMP_H          0x21
#define LSM_OUTX_L_G            0x22    /* 陀螺仪 X 低 */
#define LSM_OUTX_H_G            0x23    /* 陀螺仪 X 高 */
#define LSM_OUTY_L_G            0x24
#define LSM_OUTY_H_G            0x25
#define LSM_OUTZ_L_G            0x26
#define LSM_OUTZ_H_G            0x27
#define LSM_OUTX_L_A            0x28    /* 加速度计 X 低 */
#define LSM_OUTX_H_A            0x29
#define LSM_OUTY_L_A            0x2A
#define LSM_OUTY_H_A            0x2B
#define LSM_OUTZ_L_A            0x2C
#define LSM_OUTZ_H_A            0x2D

/* 嵌入式功能寄存器 (需设 FUNC_CFG_ACCESS bit7=1 才能访问 0x40-0x6F) */
#define LSM_EMB_FUNC_EN_A       0x04    /* SFLP / 计步器 / 倾斜 使能 */
#define LSM_EMB_FUNC_FIFO_EN_A  0x44    /* SFLP FIFO 使能 */
#define LSM_EMB_FUNC_INIT_A     0x66    /* SFLP 复位 */
#define LSM_SFLP_GAME_ODR       0x50    /* SFLP 输出速率 */

/* FIFO 输出 */
#define LSM_FIFO_DATA_OUT_TAG   0x78    /* FIFO TAG */
#define LSM_FIFO_DATA_OUT_X_L   0x79
#define LSM_FIFO_DATA_OUT_Y_L   0x7A
#define LSM_FIFO_DATA_OUT_Z_L   0x7B
#define LSM_FIFO_DATA_OUT_X_H   0x7C
#define LSM_FIFO_DATA_OUT_Y_H   0x7D
#define LSM_FIFO_DATA_OUT_Z_H   0x7E

/* ================================================================
 * WHO_AM_I
 * ================================================================ */
#define LSM_WHO_AM_I_VAL        0x70

/* ================================================================
 * CTRL3 位
 * ================================================================ */
#define LSM_CTRL3_SW_RESET      (1 << 0)
#define LSM_CTRL3_IF_INC        (1 << 2)   /* 地址自增 (多字节读必须) */
#define LSM_CTRL3_BDU           (1 << 6)   /* Block Data Update */
#define LSM_CTRL3_BOOT          (1 << 7)

/* ================================================================
 * FUNC_CFG_ACCESS 位
 * ================================================================ */
#define LSM_FUNC_CFG_EMB_ACCESS (1 << 7)
#define LSM_FUNC_CFG_SHUB_ACCESS (1 << 6)

/* ================================================================
 * CTRL1 (加速度计) ODR 编码 [3:0] — 高性能模式 (OP_MODE=000)
 * ================================================================ */
#define LSM_ODR_XL_POWER_DOWN   0x00
#define LSM_ODR_XL_7_5HZ        0x01
#define LSM_ODR_XL_15HZ         0x02
#define LSM_ODR_XL_30HZ         0x04
#define LSM_ODR_XL_60HZ         0x05
#define LSM_ODR_XL_120HZ        0x06
#define LSM_ODR_XL_240HZ        0x07
#define LSM_ODR_XL_480HZ        0x08
#define LSM_ODR_XL_960HZ        0x09
#define LSM_ODR_XL_1920HZ       0x0A
#define LSM_ODR_XL_3840HZ       0x0B
#define LSM_ODR_XL_7680HZ       0x0C

/* ================================================================
 * CTRL2 (陀螺仪) ODR 编码 [3:0]
 * ================================================================ */
#define LSM_ODR_G_POWER_DOWN    0x00
#define LSM_ODR_G_7_5HZ         0x01
#define LSM_ODR_G_15HZ          0x02
#define LSM_ODR_G_30HZ          0x04
#define LSM_ODR_G_60HZ          0x05
#define LSM_ODR_G_120HZ         0x06
#define LSM_ODR_G_240HZ         0x07
#define LSM_ODR_G_480HZ         0x08
#define LSM_ODR_G_960HZ         0x09
#define LSM_ODR_G_1920HZ        0x0A
#define LSM_ODR_G_3840HZ        0x0B
#define LSM_ODR_G_7680HZ        0x0C

/* ================================================================
 * CTRL6 (陀螺仪量程) [3:0]
 * ================================================================ */
#define LSM_FS_G_250DPS         0x00
#define LSM_FS_G_125DPS         0x01
#define LSM_FS_G_500DPS         0x02
#define LSM_FS_G_1000DPS        0x03
#define LSM_FS_G_2000DPS        0x06
#define LSM_FS_G_4000DPS        0x08

/* ================================================================
 * CTRL8 (加速度计量程) [1:0]
 * ================================================================ */
#define LSM_FS_XL_2G            0x00
#define LSM_FS_XL_4G            0x01
#define LSM_FS_XL_8G            0x02
#define LSM_FS_XL_16G           0x03

/* ================================================================
 * EMB_FUNC_EN_A (0x04) 位
 * ================================================================ */
#define LSM_SFLP_GAME_EN        (1 << 1)

/* ================================================================
 * EMB_FUNC_FIFO_EN_A (0x44) 位
 * ================================================================ */
#define LSM_SFLP_GAME_FIFO_EN   (1 << 1)

/* ================================================================
 * EMB_FUNC_INIT_A (0x66) 位
 * ================================================================ */
#define LSM_SFLP_GAME_INIT      (1 << 1)

/* ================================================================
 * SFLP ODR 编码 [2:0]
 * ================================================================ */
#define LSM_SFLP_ODR_15HZ       0
#define LSM_SFLP_ODR_30HZ       1
#define LSM_SFLP_ODR_60HZ       2
#define LSM_SFLP_ODR_120HZ      3
#define LSM_SFLP_ODR_240HZ      4
#define LSM_SFLP_ODR_480HZ      5

/* ================================================================
 * STATUS_REG 位
 * ================================================================ */
#define LSM_STATUS_XLDA         (1 << 0)  /* 加速度计数据就绪 */
#define LSM_STATUS_GDA          (1 << 1)  /* 陀螺仪数据就绪 */
#define LSM_STATUS_TDA          (1 << 2)  /* 温度数据就绪 */

/* ================================================================
 * FIFO TAG
 * ================================================================ */
#define LSM_SFLP_GAME_ROTATION_VECTOR_TAG    0x13
#define LSM_SFLP_GRAVITY_VECTOR_TAG          0x14
#define LSM_SFLP_GYROSCOPE_BIAS_TAG          0x15

/* ================================================================
 * 灵敏度常量
 * ================================================================ */
#define LSM_ACCEL_LSB_2G        (0.061f)    /* mg/LSB → g: *0.061/1000 */
#define LSM_ACCEL_LSB_4G        (0.122f)
#define LSM_ACCEL_LSB_8G        (0.244f)
#define LSM_ACCEL_LSB_16G       (0.488f)

#define LSM_GYRO_LSB_125DPS     (4.375f)    /* mdps/LSB → dps: *4.375/1000 */
#define LSM_GYRO_LSB_250DPS     (8.75f)
#define LSM_GYRO_LSB_500DPS     (17.5f)
#define LSM_GYRO_LSB_1000DPS    (35.0f)
#define LSM_GYRO_LSB_2000DPS    (70.0f)
#define LSM_GYRO_LSB_4000DPS    (140.0f)

#define LSM_TEMP_LSB            256.0f      /* LSB/°C */
#define LSM_TEMP_OFFSET         25.0f       /* °C */

/* ================================================================
 * FIFO 配置
 * ================================================================ */
#define LSM_FIFO_MODE_BYPASS    0x00
#define LSM_FIFO_MODE_FIFO      0x01
#define LSM_FIFO_MODE_CONTINUOUS 0x06     /* 流模式：满后覆盖旧数据 */

/* ================================================================
 * LSM6DSV16X 类
 * ================================================================ */
class LSM6DSV16X {
public:
    /* 原始输出 (工程单位) */
    struct Data3F {
        float ax, ay, az;    /* 加速度 g */
        float gx, gy, gz;    /* 角速度 dps */
        float temp;          /* 温度 °C */
    };

    /* 四元数 */
    struct Quaternion {
        float qx, qy, qz, qw;  /* X, Y, Z, W */
    };

    /* 原始 ADC 值 */
    struct DataRaw {
        int16_t ax, ay, az;
        int16_t gx, gy, gz;
        int16_t temp;
    };

    /* SFLP 融合输出 */
    struct SFLPData {
        Quaternion quat;       /* 游戏旋转向量 (四元数) */
        float      gravity[3]; /* 重力向量 (g) */
        float      gbias[3];   /* 陀螺仪零偏 (dps) */
    };

    enum AccelFS  { AFS_2G=2, AFS_4G=4, AFS_8G=8, AFS_16G=16 };
    enum GyroFS   { GFS_125=125, GFS_250=250, GFS_500=500,
                    GFS_1000=1000, GFS_2000=2000, GFS_4000=4000 };

    LSM6DSV16X();
    ~LSM6DSV16X();

    /* ── 初始化 ── */
    bool          begin();
    uint8_t       whoAmI();
    bool          reset();

    /* ── 原始传感器读取 ── */
    Data3F        readAll();
    DataRaw       readRaw();
    float         readTemp();

    /* ── SFLP (六轴融合) ── */
    bool          sflpBegin(uint8_t odr = LSM_SFLP_ODR_120HZ);
    bool          sflpIsReady();
    SFLPData      sflpRead();

    /* ── 陀螺仪校准 ── */
    void          calibrate(uint16_t samples = 200);
    float         gyroBiasX() const { return _gbx; }
    float         gyroBiasY() const { return _gby; }
    float         gyroBiasZ() const { return _gbz; }

    /* ── 底层 SPI 读写 ── */
    bool          writeReg(uint8_t reg, uint8_t val);
    bool          readReg(uint8_t reg, uint8_t &val);
    bool          readRegs(uint8_t reg, uint8_t *buf, uint8_t len);

private:
    void          _applyScale();
    void          _setEmbAccess(bool enable);
    uint16_t      _readFifoCount();
    uint8_t       _readFifoTag();
    bool          _readFifoData(uint8_t *buf, uint8_t len);

    spi_device_handle_t _spi;
    float       _accel_lsb;          /* m/s² / LSB */
    float       _gyro_lsb;           /* dps / LSB */
    float       _gbx, _gby, _gbz;    /* 陀螺仪零偏 */

    bool        _sflp_enabled = false;
};

/* ── 兼容旧代码的别名 ── */
typedef LSM6DSV16X ICM42688;
typedef LSM6DSV16X QMI8658;
