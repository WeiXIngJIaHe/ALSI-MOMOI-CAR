#include "mqtt.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <cstring>

static const char *TAG = "MQTT";

MQTT::MQTT(const char *uri)
    : _client(nullptr), _uri(uri), _rx_queue(nullptr)
{}

bool MQTT::init()
{
    _rx_queue = xQueueCreate(MQTT_RX_QUEUE_SIZE, sizeof(Msg));
    if (!_rx_queue) {
        ESP_LOGE(TAG, "队列创建失败");
        return false;
    }

    esp_mqtt_client_config_t cfg = {};
    cfg.broker.address.uri = _uri;
    cfg.credentials.username = MQTT_USERNAME;
    cfg.credentials.authentication.password = MQTT_PASSWORD;
    cfg.session.keepalive = 60;

    _client = esp_mqtt_client_init(&cfg);
    if (!_client) {
        ESP_LOGE(TAG, "客户端创建失败");
        return false;
    }

    /* 注册事件回调 */
    esp_mqtt_client_register_event(_client, MQTT_EVENT_ANY,
                                    &MQTT::_eventHandler, this);

    esp_mqtt_client_start(_client);

    ESP_LOGI(TAG, "连接中: %s ...", _uri);
    return true;
}

/* ── 发布 ── */

int MQTT::publish(const char *topic, const char *payload)
{
    return esp_mqtt_client_publish(_client, topic, payload, 0, 1, 0);
}

int MQTT::publish(const char *topic, const uint8_t *data, int len)
{
    return esp_mqtt_client_publish(_client, topic, (const char *)data, len, 1, 0);
}

/* ── 订阅 ── */

int MQTT::subscribe(const char *topic)
{
    return esp_mqtt_client_subscribe(_client, topic, 0);
}

/* ── 接收 ── */

bool MQTT::receive(Msg &msg)
{
    if (!_rx_queue) return false;
    return xQueueReceive(_rx_queue, &msg, 0) == pdTRUE;
}

/* ── 状态 ── */

bool MQTT::isConnected()
{
    /* 简单检查: 如果队列存在且客户端未空, 认为已连接 */
    return _client != nullptr;
}

/* ── 事件回调 ── */

void MQTT::_eventHandler(void *arg, esp_event_base_t base,
                          int32_t id, void *data)
{
    MQTT *self = (MQTT *)arg;
    esp_mqtt_event_handle_t evt = (esp_mqtt_event_handle_t)data;

    switch ((esp_mqtt_event_id_t)id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "已连接");
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "断开");
        break;

    case MQTT_EVENT_DATA: {
        /* 收到的消息 → 入队, 供 Core1 轮询消费 */
        if (self->_rx_queue && evt->topic && evt->data_len > 0) {
            Msg msg;
            msg.len = (evt->data_len < (int)(sizeof(msg.data) - 1))
                      ? evt->data_len : (int)(sizeof(msg.data) - 1);
            strncpy(msg.topic, evt->topic, sizeof(msg.topic) - 1);
            msg.topic[sizeof(msg.topic) - 1] = '\0';
            memcpy(msg.data, evt->data, msg.len);
            msg.data[msg.len] = '\0';

            /* 非阻塞发送, 队列满则丢弃 */
            xQueueSend(self->_rx_queue, &msg, 0);
        }
        break;
    }

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "错误");
        break;

    default:
        break;
    }
}
