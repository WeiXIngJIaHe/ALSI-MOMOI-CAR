#pragma once
#include <stdint.h>
#include "esp_event.h"

/* ================================================================
 * WiFi STA 模式
 *
 * 用法:
 *   WiFi wifi("SSID", "password");
 *   wifi.init();
 *   wifi.waitConnected(10000);   // 等待最多 10s
 *   ...
 *   if (wifi.isConnected()) { ... }
 * ================================================================ */

#define WIFI_SSID                   "JiaHe"
#define WIFI_PASS                   "12345678"
#define WIFI_MAX_RETRY              5
#define WIFI_RETRY_INTERVAL_MS      5000


class WiFi {
public:
    WiFi(const char *ssid = WIFI_SSID, const char *pass = WIFI_PASS);

    bool init();                                    /* 初始化并连接 */
    bool isConnected();                             /* 是否已连接 */
    bool waitConnected(uint32_t timeout_ms);        /* 阻塞等待连接 */
    const char* ip() const;                         /* IP 地址字符串 */

private:
    static void _eventHandler(void *arg, esp_event_base_t base,
                              int32_t id, void *data);
    void _onEvent(int32_t id);

    const char *_ssid;
    const char *_pass;
    bool _connected;
    int  _retry;
};
