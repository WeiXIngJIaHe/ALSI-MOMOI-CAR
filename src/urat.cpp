#include "URAT.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <cstdarg>
#include <cstdio>
#include <cstring>

static const char *TAG = "URAT";

/*
 * 帧格式:
 *   STX(0xAA) + CMD + LEN + DATA[0..N] + XOR + ETX(0x55)
 *   XOR = STX ^ CMD ^ LEN ^ DATA[0] ^ ... ^ DATA[N-1]
 */

#define FRAME_STX                   0xAA
#define FRAME_ETX                   0x55

URAT::URAT(uart_port_t port, int tx, int rx, int baud)
    : _port(port), _tx(tx), _rx(rx), _baud(baud) {}

void URAT::init()
{
    uart_config_t cfg = {};
    cfg.baud_rate           = _baud;
    cfg.data_bits           = UART_DATA_8_BITS;
    cfg.parity              = UART_PARITY_DISABLE;
    cfg.stop_bits           = UART_STOP_BITS_1;
    cfg.flow_ctrl           = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk          = UART_SCLK_DEFAULT;

    uart_driver_install(_port, URAT_RX_BUF_SIZE, URAT_TX_BUF_SIZE, 0, NULL, 0);
    uart_param_config(_port, &cfg);
    uart_set_pin(_port, _tx, _rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ESP_LOGI(TAG, "上位机链路就绪 (port=%d, %d baud)", _port, _baud);
}

void URAT::deinit() { uart_driver_delete(_port); }

/* ── 发送 ── */

int URAT::write(const uint8_t *data, size_t len)
{
    return uart_write_bytes(_port, data, len);
}

int URAT::write(const char *str)
{
    return uart_write_bytes(_port, str, strlen(str));
}

int URAT::printf(const char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (n > 0) uart_write_bytes(_port, buf, n);
    return n;
}

/*
 * 打包并发送一帧: [STX][CMD][LEN][DATA..][XOR][ETX]
 */
int URAT::sendFrame(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    uint8_t buf[258];              /* 2 + 1 + 1 + max252 + 1 + 1 */
    uint8_t xor_val = 0;
    int pos = 0;

    buf[pos++] = FRAME_STX;
    buf[pos++] = cmd;
    buf[pos++] = len;

    for (uint8_t i = 0; i < len; i++)
        buf[pos++] = data[i];

    /* XOR checksum: STX ^ CMD ^ LEN ^ DATA[0..] */
    for (int i = 0; i < pos; i++) xor_val ^= buf[i];
    buf[pos++] = xor_val;
    buf[pos++] = FRAME_ETX;

    return uart_write_bytes(_port, buf, pos);
}

/* ── 接收 (非阻塞) ── */

int URAT::read(uint8_t *buf, size_t max_len)
{
    size_t avail;
    uart_get_buffered_data_len(_port, &avail);
    if (avail == 0) return 0;
    if (avail > max_len) avail = max_len;
    return uart_read_bytes(_port, buf, avail, 0);
}

int URAT::getc()
{
    uint8_t byte;
    if (uart_read_bytes(_port, &byte, 1, 0) > 0) return byte;
    return -1;
}

int URAT::available()
{
    size_t avail;
    uart_get_buffered_data_len(_port, &avail);
    return (int)avail;
}

void URAT::setBaudrate(int baud)
{
    _baud = baud;
    uart_set_baudrate(_port, baud);
}

void URAT::flush() { uart_flush(_port); }
