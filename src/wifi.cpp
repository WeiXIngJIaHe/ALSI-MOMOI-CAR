#include "wifi.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>

static const char *TAG = "WiFi";

/* FreeRTOS 事件组: BIT0=已连接, BIT1=失败 */
static const int WIFI_CONNECTED_BIT = BIT0;
static const int WIFI_FAIL_BIT      = BIT1;

WiFi::WiFi(const char *ssid, const char *pass)
    : _ssid(ssid), _pass(pass), _connected(false), _retry(0)
{}

bool WiFi::init()
{
    /* 1. 网络栈初始化 */
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    /* 2. 注册事件回调 (静态方法中转) */
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                               &WiFi::_eventHandler, this);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                               &WiFi::_eventHandler, this);

    /* 3. 配置并启动 */
    wifi_config_t wifi_cfg = {};
    strncpy((char *)wifi_cfg.sta.ssid, _ssid, sizeof(wifi_cfg.sta.ssid) - 1);
    strncpy((char *)wifi_cfg.sta.password, _pass, sizeof(wifi_cfg.sta.password) - 1);
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_cfg.sta.pmf_cfg.capable = true;
    wifi_cfg.sta.pmf_cfg.required = false;

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
    esp_wifi_start();

    ESP_LOGI(TAG, "连接中: %s ...", _ssid);
    return true;
}

bool WiFi::isConnected()
{
    return _connected;
}

bool WiFi::waitConnected(uint32_t timeout_ms)
{
    /* 轮询等待连接 (WiFi 事件在独立任务中回调) */
    uint32_t elapsed = 0;
    const uint32_t tick = 20;

    while (elapsed < timeout_ms) {
        if (_connected) {
            ESP_LOGI(TAG, "已连接! IP: %s", ip());
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(tick));
        elapsed += tick;
    }

    ESP_LOGE(TAG, "连接超时");
    return false;
}

const char* WiFi::ip() const
{
    static char buf[16];
    esp_netif_ip_info_t info;
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif && esp_netif_get_ip_info(netif, &info) == ESP_OK) {
        snprintf(buf, sizeof(buf), IPSTR, IP2STR(&info.ip));
    } else {
        strcpy(buf, "0.0.0.0");
    }
    return buf;
}

/* ── 事件处理 ── */

void WiFi::_eventHandler(void *arg, esp_event_base_t base,
                          int32_t id, void *data)
{
    WiFi *self = (WiFi *)arg;
    self->_onEvent(id);
}

void WiFi::_onEvent(int32_t id)
{
    if (id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    }
    else if (id == WIFI_EVENT_STA_DISCONNECTED) {
        _connected = false;
        if (_retry < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            _retry++;
            ESP_LOGW(TAG, "断开, 重连 %d/%d ...", _retry, WIFI_MAX_RETRY);
        } else {
            ESP_LOGE(TAG, "重连失败, 已达最大次数");
        }
    }
    else if (id == IP_EVENT_STA_GOT_IP) {
        _connected = true;
        _retry = 0;
    }
}
