#pragma once
#include "esp_event.h"

/* ================================================================
 * WiFi STA — ESP-IDF v5.5 standard init sequence
 *
 * Usage:
 *   WiFi wifi("SSID", "password");
 *   wifi.init();
 *   if (wifi.waitConnected(10000)) { ... }
 * ================================================================ */

#define WIFI_SSID                   "JiaHe"
#define WIFI_PASS                   "12345678"
#define WIFI_MAX_RETRY              5

class WiFi {
public:
    WiFi(const char *ssid = WIFI_SSID, const char *pass = WIFI_PASS);
    ~WiFi();

    bool init();
    bool deinit();
    bool isConnected();
    bool waitConnected(uint32_t timeout_ms);
    const char* ip() const;

    /* RF 天线测试 */
    int  getRSSI();                  /* 当前 RSSI (dBm), 未连接返回 0 */
    void scanStart();                /* 启动 AP 扫描 */
    void scanLog();                  /* 打印扫描结果 (AP列表+RSSI) */

private:
    static void _onWifiEvent(void *arg, esp_event_base_t base,
                             int32_t id, void *data);
    static void _onIpEvent(void *arg, esp_event_base_t base,
                           int32_t id, void *data);

    const char *_ssid;
    const char *_pass;
    bool  _connected;
    int   _retry;

    esp_event_handler_instance_t _wifi_evt;
    esp_event_handler_instance_t _ip_evt;
};
