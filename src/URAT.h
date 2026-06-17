#pragma once
#include <stdint.h>
#include <stddef.h>
#include "driver/uart.h"

/* ================================================================
 * UART 上位机通信链路
 *
 * 用途: ESP32 ←→ 上位机 (树莓派/Jetson/PC) 双向数据交互
 *
 * 物理连接:
 *   ESP32 TX(43) → 上位机 RX
 *   ESP32 RX(44) → 上位机 TX
 *   GND ←→ GND
 *
 * 通信协议:
 *   帧格式: [STX(0xAA)] [CMD(1B)] [LEN(1B)] [DATA(N)] [XOR(1B)] [ETX(0x55)]
 *   常用命令:
 *     0x01 = 查询 IMU 数据     → 响应: 6×float
 *     0x02 = 查询电机状态       → 响应: 4×int16
 *     0x03 = 设置电机 PWM       → 发送: 2×int16
 *     0x04 = 查询电池电压       → 响应: 1×float
 *     0x10 = 心跳              → 响应: 0xAC
 *     0xFF = 复位               → 响应: 0xAC
 * ================================================================ */

/* ---- 硬件引脚 (上位机通信) ---- */
#define URAT_TX_PIN                 17
#define URAT_RX_PIN                 18
#define URAT_BAUDRATE               115200
#define URAT_RX_BUF_SIZE            2048
#define URAT_TX_BUF_SIZE            512

/* USB Serial/JTAG (ESP32-S3 内置调试口, 独立于上位机 UART) */
#define URAT_DEBUG_PORT             UART_NUM_0


class URAT {
public:
    URAT(uart_port_t port = UART_NUM_1, int tx = URAT_TX_PIN, int rx = URAT_RX_PIN,
         int baud = URAT_BAUDRATE);

    void init();
    void deinit();

    /* 发送 */
    int  write(const uint8_t *data, size_t len);
    int  write(const char *str);
    int  printf(const char *fmt, ...);

    /* 打包发送帧 */
    int  sendFrame(uint8_t cmd, const uint8_t *data, uint8_t len);

    /* 接收 (非阻塞) */
    int  read(uint8_t *buf, size_t max_len);
    int  getc();                                     /* 读单字节, 无数据返回 -1 */
    int  available();

    /* 配置 */
    void setBaudrate(int baud);
    void flush();

private:
    uart_port_t _port;
    int _tx, _rx, _baud;
};
