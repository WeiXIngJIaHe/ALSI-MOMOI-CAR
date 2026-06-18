#include "wifi.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>

static const char *TAG = "WiFi";

WiFi::WiFi(const char *ssid, const char *pass)
    : _ssid(ssid), _pass(pass), _connected(false), _retry(0),
      _wifi_evt(nullptr), _ip_evt(nullptr)
{}

WiFi::~WiFi()
{
    deinit();
}

/* ESP-IDF v5.5 WiFi STA 标准初始化序列
 *   1. TCP/IP 栈 → 2. 事件循环 → 3. STA 接口
 *   4. WiFi 驱动 → 5. 事件回调 → 6. 配置启动
 *   每步检查返回值, 失败即停止 */
bool WiFi::init()
{
    esp_netif_init();                               // 1. TCP/IP 协议栈

    esp_err_t ret = esp_event_loop_create_default(); // 2. 默认事件循环
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "event_loop_create: %s", esp_err_to_name(ret));
        return false;
    }

    esp_netif_create_default_wifi_sta();            // 3. 创建 WiFi STA 接口

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);                      // 4. WiFi 驱动初始化
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "wifi_init: %s", esp_err_to_name(ret));
        return false;
    }

    // 5. 注册事件回调 (v5.x 实例化 API, 支持注销)
    ret = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                              &_onWifiEvent, this, &_wifi_evt);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "reg wifi_evt: %s", esp_err_to_name(ret)); return false; }

    ret = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                              &_onIpEvent, this, &_ip_evt);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "reg ip_evt: %s", esp_err_to_name(ret)); return false; }

    // 6. 配置 SSID/密码, 启动连接
    esp_wifi_set_mode(WIFI_MODE_STA);
    wifi_config_t wifi_cfg = {};
    strncpy((char *)wifi_cfg.sta.ssid, _ssid, sizeof(wifi_cfg.sta.ssid) - 1);
    strncpy((char *)wifi_cfg.sta.password, _pass, sizeof(wifi_cfg.sta.password) - 1);
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;  // 最低 WPA2 安全级别
    esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
    ret = esp_wifi_start();
    if (ret != ESP_OK) { ESP_LOGE(TAG, "wifi_start: %s", esp_err_to_name(ret)); return false; }

    ESP_LOGI(TAG, "connecting: %s", _ssid);
    return true;
}

bool WiFi::deinit()
{
    if (_wifi_evt) {
        esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, _wifi_evt);
        _wifi_evt = nullptr;
    }
    if (_ip_evt) {
        esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, _ip_evt);
        _ip_evt = nullptr;
    }
    esp_wifi_stop();
    esp_wifi_deinit();
    _connected = false;
    return true;
}

bool WiFi::isConnected()
{
    return _connected;
}

bool WiFi::waitConnected(uint32_t timeout_ms)
{
    uint32_t elapsed = 0;
    const uint32_t tick = 50;

    while (elapsed < timeout_ms) {
        if (_connected) {
            ESP_LOGI(TAG, "connected, IP: %s", ip());
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(tick));
        elapsed += tick;
    }

    ESP_LOGE(TAG, "connect timeout");
    return false;
}

const char* WiFi::ip() const
{
    static char buf[16];
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif) return "0.0.0.0";

    esp_netif_ip_info_t info;
    if (esp_netif_get_ip_info(netif, &info) == ESP_OK)
        snprintf(buf, sizeof(buf), IPSTR, IP2STR(&info.ip));
    else
        strcpy(buf, "0.0.0.0");
    return buf;
}

/* ── 天线射频测试 ── */

int WiFi::getRSSI()
{
    int rssi = -127; // 使用 -127 作为无效底噪默认值
    esp_err_t ret = esp_wifi_sta_get_rssi(&rssi);
    
    if (ret != ESP_OK) {
        // 如果没连上，打印真实的错误原因，而不是默默返回0
        return -127; 
    }
    
    return rssi;// dBm, 未连接返回 0
}

// 主动扫描周围 AP, 测试天线接收灵敏度
void WiFi::scanStart()
{
    wifi_scan_config_t cfg = {};
    cfg.scan_type = WIFI_SCAN_TYPE_ACTIVE;
    esp_wifi_scan_start(&cfg, true);   // true = 阻塞至扫描完成
    scanLog();
}

// 打印扫描结果, 按 RSSI 评级天线性能
void WiFi::scanLog()
{
    uint16_t num = 0;
    esp_wifi_scan_get_ap_num(&num);
    if (num == 0) {
        ESP_LOGI(TAG, "RF scan: 未发现 AP (检查天线连接)");
        return;
    }

    wifi_ap_record_t *list = (wifi_ap_record_t *)malloc(num * sizeof(wifi_ap_record_t));
    if (!list) return;
    esp_wifi_scan_get_ap_records(&num, list);

    // RSSI 评级: ≥-55 优秀, -55~-70 良好, -70~-85 一般, <-85 差
    for (uint16_t i = 0; i < num; i++) {
        const char *qual;
        if (list[i].rssi >= -55)      qual = "优秀";
        else if (list[i].rssi >= -70) qual = "良好";
        else if (list[i].rssi >= -85) qual = "一般";
        else                          qual = "差";

        ESP_LOGI(TAG, "AP[%d]: %s  CH=%d  RSSI=%d dBm  %s",
                 i, list[i].ssid, list[i].primary, list[i].rssi, qual);
    }

    int best = (num > 0) ? list[0].rssi : -100;
    ESP_LOGI(TAG, "扫描完成: %d 个AP, 最强 RSSI=%d dBm → 天线%s",
             num, best, (best >= -80) ? "正常" : "偏弱 (检查硬件)");
    free(list);
}

/* ── WiFi 事件回调 (在事件任务上下文中执行) ── */

void WiFi::_onWifiEvent(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    WiFi *self = (WiFi *)arg;

    switch (id) {
    case WIFI_EVENT_STA_START:
        esp_wifi_connect();
        break;

    case WIFI_EVENT_STA_DISCONNECTED: {
        wifi_event_sta_disconnected_t *evt = (wifi_event_sta_disconnected_t *)data;
        self->_connected = false;
        if (self->_retry < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            self->_retry++;
            ESP_LOGW(TAG, "disconnected (reason=%d), retry %d/%d",
                     evt->reason, self->_retry, WIFI_MAX_RETRY);
        } else {
            ESP_LOGE(TAG, "max retries reached");
        }
        break;
    }

    case WIFI_EVENT_STA_CONNECTED:
        ESP_LOGI(TAG, "associated");
        break;

    default:
        break;
    }
}

/* ── IP_EVENT handler ── */

void WiFi::_onIpEvent(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    WiFi *self = (WiFi *)arg;

    if (id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *evt = (ip_event_got_ip_t *)data;
        self->_connected = true;
        self->_retry = 0;
        ESP_LOGI(TAG, "got IP: " IPSTR, IP2STR(&evt->ip_info.ip));
    }
}
