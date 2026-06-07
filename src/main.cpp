/*---- 双核框架 (C++ 类架构) ----
 * Core0: 传感器采集 + 姿态解算 (2ms)
 * Core1: 控制 + 电机 + UI       (5ms)
 *
 * 共用 I2C_NUM_0: IS31FL3239(0x3C) + TCA6408(0x20) + LCD(0x27)
 * SPI2_HOST:      ICM42688 (独立 SPI 总线)
 * UART1:          上位机通信 (TX=43, RX=44, 115200bps)
 * UART0:          USB Serial/JTAG 调试口
 * LED:   12 组 RGB → IS31FL3239 (36 通道, 每 RGB 占 3 通道)
 * 按键:  4 键 → TCA6408
 * IMU:   6轴 → ICM42688 (SPI)
 * WiFi:  STA 模式 → MQTT
 */

#include "IS31FL3239.h"
#include "TCA6408.h"
#include "LCD.h"
#include "LED.h"
#include "button.h"
#include "pid_controller.h"
#include "ICM42688.h"
#include "DRV8870.h"
#include "URAT.h"
#include "wifi.h"
#include "mqtt.h"
#include "wifi_remote.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "Core";

/* ---- 全局设备 ---- */
IS31FL3239 g_led_drv(IS31FL3239_I2C_ADDR);
TCA6408    g_ioexp(TCA6408_I2C_ADDR);
LCD1602    g_lcd(LCD1602_I2C_ADDR);
LED        g_led(g_led_drv);
Button     g_btn(g_ioexp);
ICM42688   g_imu;
URAT       g_uart(UART_NUM_1, URAT_TX_PIN, URAT_RX_PIN, URAT_BAUDRATE);
WiFi       g_wifi(WIFI_SSID, WIFI_PASS);
MQTT       g_mqtt(MQTT_BROKER_URI);
MotorDriver g_motor;
WiFiRemote  g_wifi_remote(g_motor);

/* ---- 队列 ---- */
struct Msg {
    enum { IMU, BTN, UART, CMD } type;
    union {
        struct { float ax, ay, az, gx, gy, gz, temp; uint32_t ts; } imu;
        ButtonInfo btn;
        uint8_t    uart_byte;
        int32_t    cmd_val;
    };
};
static QueueHandle_t _sq, _cq;

/* ================================================================ */
static void hw_init()
{
    /* I2C 总线: IS31FL3239 init 内部触发一次性初始化 */
    g_led_drv.init();
    g_ioexp.init();
    g_lcd.init();

    /* 上层封装 */
    g_led.init();
    g_btn.init();

    /* 电机驱动 (PWM) */
    g_motor.init();

    /* IMU */
    if (!g_imu.begin()) { ESP_LOGE(TAG, "IMU 初始化失败"); }
    g_imu.configINT1(false, false, false);  /* 推挽/低有效/脉冲 */
    g_imu.enableDataReadyINT(true);

    /* UART */
    g_uart.init();

    /* WiFi + MQTT */
    g_wifi.init();
    g_wifi.waitConnected(15000);
    g_mqtt.init();
    g_mqtt.subscribe("/car/cmd");
    g_mqtt.subscribe("/car/led");

    /* WiFi 遥控 (TCP Server) */
    g_wifi_remote.init();

    /* 陀螺仪零偏校准 (需静止) */
    g_imu.calibrate(200);

    g_led.setRGBAll(0, 0, 255);  g_led.update();
    g_lcd.write("DualCore OK");
    ESP_LOGI(TAG, "硬件初始化完成");
}

/* ================================================================ */
static void core0_sensor(void *pv)
{
    Msg msg{ .type = Msg::IMU };
    TickType_t xl = xTaskGetTickCount();
    ESP_LOGI(TAG, "Core0 OK");

    while (1) {
        auto d = g_imu.readAll();
        msg.imu = { d.ax, d.ay, d.az, d.gx, d.gy, d.gz, d.temp, (uint32_t)xTaskGetTickCount() };
        xQueueSend(_sq, &msg, 0);

        Msg cmd;
        if (xQueueReceive(_cq, &cmd, 0) == pdPASS) { /* 指令 */ }

        vTaskDelayUntil(&xl, pdMS_TO_TICKS(2));
    }
}

/* ================================================================ */
static void core1_ctrl(void *pv)
{
    Msg msg;
    int  tick = 0;

    /* PID: 姿态角速度控制 → 电机差速 */
    PID pid_balance(0.8f, 0.05f, 0.03f, 5.0f, -1023, 1023);  /* 输出→电机占空比 */
    PID pid_turn(0.5f, 0.02f, 0.01f, 5.0f, -512, 512);

    /* 目标: 保持静止 (角速度=0, 方向=0) */
    float target_gyro_z = 0.0f;    /* 目标偏航角速度 */
    float target_turn   = 0.0f;

    TickType_t xl = xTaskGetTickCount();
    ESP_LOGI(TAG, "Core1 OK");

    while (1) {
        /* ── 传感器 ── */
        decltype(Msg::imu) latest{};
        bool has_new = false;
        while (xQueueReceive(_sq, &msg, 0) == pdPASS) {
            if (msg.type == Msg::IMU) { latest = msg.imu; has_new = true; }
        }

        /* ── PID → 电机 ── */
        if (has_new) {
            /* 姿态平衡 PID: 角速度误差 → 线速度修正 */
            float balance_out = pid_balance.update(target_gyro_z, latest.gz);
            float turn_out    = pid_turn.update(target_turn, latest.gz);

            /* 驱动电机 (差分: speed±turn) */
            g_motor.drive((int16_t)balance_out, (int16_t)turn_out);
        }

        /* ── 按键 ── */
        g_btn.scan();
        ButtonInfo bi;
        while (g_btn.getEvent(bi)) {
            msg.type = Msg::BTN;
            msg.btn  = bi;
            /* TODO: 按键处理 */
        }

        /* ── WiFi 遥控 (TCP Server, 非阻塞) ── */
        g_wifi_remote.process();

        /* ── MQTT 接收 ── */
        MQTT::Msg mq;
        while (g_mqtt.receive(mq)) {
            if (strstr(mq.topic, "/car/cmd")) {
                g_mqtt.publish("/car/ack", "ok");
            }
        }

        /* ── 上位机 UART 协议处理 ──
         * 帧格式: [STX(0xAA)] [CMD] [LEN] [DATA..] [XOR] [ETX(0x55)]
         * 接收到完整帧后解析命令, 发送响应
         */
        static uint8_t rx_buf[64];
        static int     rx_pos = 0;
        static bool    in_frame = false;

        int c;
        while ((c = g_uart.getc()) >= 0) {
            uint8_t b = (uint8_t)c;

            if (!in_frame) {
                if (b == 0xAA) {            /* 帧头 STX */
                    in_frame = true;
                    rx_pos = 0;
                }
            } else {
                if (rx_pos < (int)sizeof(rx_buf))
                    rx_buf[rx_pos++] = b;

                /* 帧尾 ETX 检测: 帧格式中倒数第2字节是XOR, 倒数第1是ETX */
                /* 最小帧: STX CMD LEN XOR ETX = 5字节 */
                if (rx_pos >= 4 && b == 0x55) {
                    /* 解析: buf[0]=CMD, buf[1]=LEN, buf[2..]=DATA, buf[-2]=XOR */
                    uint8_t cmd  = rx_buf[0];
                    uint8_t len  = rx_buf[1];
                    uint8_t xorr = rx_buf[rx_pos - 2];

                    /* 验证 XOR (简化: 跳过, 实际应计算) */
                    (void)len;
                    (void)xorr;

                    /* 命令分发 */
                    switch (cmd) {
                    case 0x10:   /* 心跳 */
                        g_uart.sendFrame(0x10, nullptr, 0);
                        break;

                    case 0x01: { /* 返回 IMU */
                        auto d = g_imu.readAll();
                        float imu[6] = { d.ax, d.ay, d.az, d.gx, d.gy, d.gz };
                        g_uart.sendFrame(0x01, (uint8_t*)imu, sizeof(imu));
                        break;
                    }

                    case 0x03:   /* 电机控制: 2×int16 */
                        if (len >= 4) {
                            /* int16_t m1 = *(int16_t*)(rx_buf + 2); */
                            /* int16_t m2 = *(int16_t*)(rx_buf + 4); */
                        }
                        break;

                    case 0xFF:   /* 复位 */
                        g_uart.sendFrame(0xFF, nullptr, 0);
                        esp_restart();
                        break;

                    default:
                        break;
                    }

                    in_frame = false;
                    rx_pos = 0;
                }

                /* 帧太长/超时则丢弃 */
                if (rx_pos >= (int)sizeof(rx_buf)) {
                    in_frame = false;
                    rx_pos = 0;
                }
            }
        }

        /* ── LED 效果 ── */
        if (++tick >= 40) {
            g_led.effect(LED_EFFECT_BREATH, 2);
            tick = 0;
        }

        vTaskDelayUntil(&xl, pdMS_TO_TICKS(5));
    }
}

/* ================================================================ */
extern "C" void app_main()
{
    hw_init();

    _sq = xQueueCreate(16, sizeof(Msg));
    _cq = xQueueCreate(8,  sizeof(Msg));

    xTaskCreatePinnedToCore(core0_sensor, "sensor",
        4096, NULL, configMAX_PRIORITIES - 1, NULL, 0);

    xTaskCreatePinnedToCore(core1_ctrl, "control",
        4096, NULL, configMAX_PRIORITIES - 2, NULL, 1);

    g_led.setRGBAll(0, 255, 0);  g_led.update();
    ESP_LOGI(TAG, "双核 C++ 框架启动");
    vTaskDelete(NULL);
}
