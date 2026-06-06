#pragma once
#include <stdint.h>
#include "mqtt_client.h"

/* ================================================================
 * MQTT 客户端 (基于 ESP-IDF esp_mqtt)
 *
 * 用法:
 *   MQTT mqtt("mqtt://broker:1883");
 *   mqtt.init();
 *   mqtt.subscribe("/topic");
 *   mqtt.publish("/topic", "hello");
 *   ...
 *   while (1) {
 *       MQTT::Msg msg;
 *       if (mqtt.receive(msg)) { ... }
 *       vTaskDelay(10);
 *   }
 * ================================================================ */

#define MQTT_BROKER_URI             "mqtt://192.168.1.100:1883"
#define MQTT_CLIENT_ID              "cra_car"
#define MQTT_USERNAME               ""
#define MQTT_PASSWORD               ""
#define MQTT_RX_QUEUE_SIZE          16


class MQTT {
public:
    struct Msg {
        char    topic[64];
        uint8_t data[256];
        int     len;
    };

    MQTT(const char *uri = MQTT_BROKER_URI);

    bool init();

    /* 发布 */
    int publish(const char *topic, const char *payload);
    int publish(const char *topic, const uint8_t *data, int len);

    /* 订阅 */
    int subscribe(const char *topic);

    /* 接收 (非阻塞, 从内部队列取) */
    bool receive(Msg &msg);

    /* 状态 */
    bool isConnected();

private:
    static void _eventHandler(void *arg, esp_event_base_t base,
                              int32_t id, void *data);

    esp_mqtt_client_handle_t _client;
    const char *_uri;
    QueueHandle_t _rx_queue;
};
