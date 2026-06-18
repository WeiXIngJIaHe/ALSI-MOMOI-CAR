/*---- 双核实时框架 (ESP32-S3, C++ OOP) ----
 * Core0 (优先级高): 传感器 2ms → 姿态解算 → 队列发送
 * Core1 (优先级低): 队列接收 → PID 控制 → 电机/UI/UART/WiFi
 *
 * 总线分配:
 *   I2C_NUM_0  → IS31FL3239(0x3C) LED驱动 + TCA6408(0x20) IO扩展 + LCD1602(0x27)
 *   SPI2_HOST  → LSM6DSV16X IMU (DMA, 独立总线)
 *   UART1      → 上位机通信 (TX=43 RX=44)
 *   UART0      → USB Serial/JTAG 调试
 *
 * 传感器融合:
 *   SFLP (芯片内置) → Game Rotation Vector 四元数 @ 120Hz
 *   Mahony (软件)   → 互补滤波, Ki 积分项消除陀螺零漂
 *
 * 数据流:
 *   Core0: IMU → readAll() → SFLP + Mahony → Msg → Queue → Core1
 *   Core1: Queue → Msg → PID → Motor / UART / WiFi
 */

#include "IS31FL3239.h"
#include "TCA6408.h"
#include "LCD.h"
#include "LED.h"
#include "button.h"
#include "pid_controller.h"
#include "IMU.h"
#include "DRV8870.h"
#include "URAT.h"
#include "wifi.h"
#include "mqtt.h"
#include "wifi_remote.h"
#include "Mahony.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include <cmath>

static const char *TAG = "Core";

/* 初始化同步: BIT0=硬件就绪, BIT1=WiFi就绪 */
static EventGroupHandle_t _init_evt = nullptr;
#define INIT_BIT_HW_READY   (1 << 0)
#define INIT_BIT_WIFI_READY (1 << 1)

/* 是否启用 SFLP (内置6轴融合) */
#define USE_SFLP  1   /* 1=芯片内置四元数, 0=Mahony 软件融合 */

/* ---- 全局设备 ---- */
IS31FL3239  g_led_drv(IS31FL3239_I2C_ADDR);
TCA6408     g_ioexp(TCA6408_I2C_ADDR);
LCD1602     g_lcd(LCD1602_I2C_ADDR);
LED         g_led(g_led_drv);
Button      g_btn(g_ioexp);
ICM42688    g_imu;
URAT        g_uart(UART_NUM_1, URAT_TX_PIN, URAT_RX_PIN, URAT_BAUDRATE);
WiFi        g_wifi(WIFI_SSID, WIFI_PASS);
MQTT        g_mqtt(MQTT_BROKER_URI);
MotorDriver g_motor;
WiFiRemote  g_wifi_remote(g_motor);
Mahony      g_mahony;                        /* Mahony 互补滤波 */

/* ---- 队列 ---- */
struct Msg {
    enum { IMU, BTN, UART, CMD } type;
    union {
        struct {
            float ax, ay, az;     /* 加速度 m/s² */
            float gx, gy, gz;     /* 角速度 dps   */
            float temp;           /* 温度 °C     */
            float roll, pitch;    /* 欧拉角 rad  */
            float yaw;            /* 航向角 rad  */
            float qw, qx, qy, qz; /* 四元数 (SFLP) */
            uint32_t ts;
        } imu;
        ButtonInfo btn;
        uint8_t    uart_byte;
        int32_t    cmd_val;
    };
};
static QueueHandle_t _sq, _cq;

/* ── 硬件初始化 (app_main 中调用, 阻塞执行) ──
 * NVS → I2C 设备 → LED/按键/电机 → UART → WiFi → IMU → 姿态融合
 * 全部完成后设 INIT_BIT_HW_READY 解锁双核任务 */
static void hw_init()
{
    // NVS: WiFi 必需, 损坏则自动擦除重试 (ESP-IDF 标准模式)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* I2C 总线: IS31FL3239 init 内部触发一次性初始化 */
    g_led_drv.init();
    g_ioexp.init();
    g_lcd.init();

    /* 上层封装 */
    g_led.init();
    g_btn.init();

    /* 电机驱动 (PWM) */
    g_motor.init();

    /* UART */
    g_uart.init();

    /* WiFi */
    if (g_wifi.init()) {
        /* 天线射频测试: 扫描周围 AP, 打印 RSSI (dBm) */
        g_wifi.scanStart();

        /* 连接后初始化 MQTT (需要正确 SSID/密码) */
        //if (g_wifi.waitConnected(10000)) {
        //    g_mqtt.init();
        //    g_mqtt.subscribe("/car/cmd");
        //    g_wifi_remote.init();
        //}
    }

    if (!g_imu.begin())
        ESP_LOGE(TAG, "IMU 初始化失败!");
    g_imu.calibrate(200);

#if USE_SFLP
    if (!g_imu.sflpBegin(LSM_SFLP_ODR_120HZ))
        ESP_LOGW(TAG, "SFLP 启动失败");
#endif

    g_mahony.begin(200.0f, 0.5f, 0.01f);

    g_led.setRGBAll(0, 0, 255);  g_led.update();
    g_lcd.write("DualCore OK");
    ESP_LOGI(TAG, "硬件初始化完成");

    /* 通知所有等待任务: 硬件已就绪 */
    if (_init_evt) xEventGroupSetBits(_init_evt, INIT_BIT_HW_READY);
}

/* ── Core0: 传感器采集 + 姿态融合 (100Hz)
 *   readAll() DMA 突发 → SFLP 四元数 → Mahony 去零漂 → 队列发送
 *   每 500ms 打印一次: 加速度/角速度/四元数/欧拉角/WiFi RSSI */
static void core0_sensor(void *pv)
{
    if (_init_evt)
        xEventGroupWaitBits(_init_evt, INIT_BIT_HW_READY,
                            pdFALSE, pdTRUE, portMAX_DELAY);

    Msg msg = {};
    msg.type = Msg::IMU;
    float deg2rad = 3.14159265f / 180.0f;
    int   pr_cnt  = 0;

    ESP_LOGI(TAG, "Core0 start");

    while (1) {
        auto d = g_imu.readAll();

        float sflp_qw = 0, sflp_qx = 0, sflp_qy = 0, sflp_qz = 0;
#if USE_SFLP
        if (g_imu.sflpIsReady()) {
            auto s = g_imu.sflpRead();
            sflp_qw = s.quat.qw; sflp_qx = s.quat.qx;
            sflp_qy = s.quat.qy; sflp_qz = s.quat.qz;
        }
#endif

        g_mahony.update(d.ax, d.ay, d.az,
                        d.gx * deg2rad, d.gy * deg2rad, d.gz * deg2rad,
                        0.01f);

        float mahony_qw, mahony_qx, mahony_qy, mahony_qz;
        g_mahony.getQuaternion(mahony_qw, mahony_qx, mahony_qy, mahony_qz);
        float roll  = g_mahony.roll();
        float pitch = g_mahony.pitch();
        float yaw   = g_mahony.yaw();

        auto &m = msg.imu;
        m.ax = d.ax; m.ay = d.ay; m.az = d.az;
        m.gx = d.gx; m.gy = d.gy; m.gz = d.gz;
        m.temp  = d.temp;
        m.roll  = roll;  m.pitch = pitch;  m.yaw = yaw;
        m.qw = mahony_qw; m.qx = mahony_qx; m.qy = mahony_qy; m.qz = mahony_qz;
        m.ts = (uint32_t)xTaskGetTickCount();
        xQueueSend(_sq, &msg, 0);

        if (++pr_cnt >= 50) {
            float bz = g_mahony.biasZ();
            int wifi_rssi = g_wifi.getRSSI();
            ESP_LOGI(TAG, "ACC(m/s²): %.2f %.2f %.2f | GYRO(dps): %.1f %.1f %.1f | BiasZ: %.4f",
                     d.ax, d.ay, d.az, d.gx, d.gy, d.gz, bz);
            ESP_LOGI(TAG, "Mahony Q: %.4f %.4f %.4f %.4f | Euler(°): %.1f %.1f %.1f | WiFi: %d dBm",
                     mahony_qw, mahony_qx, mahony_qy, mahony_qz,
                     roll * 57.3f, pitch * 57.3f, yaw * 57.3f, wifi_rssi);
#if USE_SFLP
            if (sflp_qw != 0 || sflp_qx != 0 || sflp_qy != 0 || sflp_qz != 0)
                ESP_LOGI(TAG, "SFLP Q: %.4f %.4f %.4f %.4f",
                         sflp_qw, sflp_qx, sflp_qy, sflp_qz);
#endif
            ESP_LOGI(TAG, "Temp: %.1f°C", d.temp);
            pr_cnt = 0;
        }

        Msg cmd;
        if (xQueueReceive(_cq, &cmd, 0) == pdPASS) { (void)cmd; }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ── Core1: 控制中枢
 *   队列接收姿态 → PID 平衡/转向 → 电机驱动
 *   外设扫描: 按键/WiFi遥控/MQTT/UART 协议解析 */
static void core1_ctrl(void *pv)
{
    /* 等待硬件初始化完成 */
    if (_init_evt)
        xEventGroupWaitBits(_init_evt, INIT_BIT_HW_READY,
                            pdFALSE, pdTRUE, portMAX_DELAY);

    Msg msg;
    int  tick = 0;

    PID pid_balance(0.8f, 0.05f, 0.03f, 5.0f, -1023, 1023);
    PID pid_turn(0.5f, 0.02f, 0.01f, 5.0f, -512, 512);
    float target_z = 0.0f;

    ESP_LOGI(TAG, "Core1 start");

    while (1) {
        decltype(Msg::imu) latest{};
        bool has_new = false;
        while (xQueueReceive(_sq, &msg, 0) == pdPASS) {
            if (msg.type == Msg::IMU) {
                latest = msg.imu;
                has_new = true;
            }
        }

        if (has_new) {
            float rad2deg = 180.0f / 3.14159265f;
            float roll_d  = latest.roll * rad2deg;
            float pitch_d = latest.pitch * rad2deg;
            pid_balance.update(target_z, roll_d);
            pid_turn.update(target_z, latest.gz);
            (void)pitch_d;
        }

        g_btn.scan();
        ButtonInfo bi;
        while (g_btn.getEvent(bi)) {
            msg.type = Msg::BTN;
            msg.btn  = bi;
        }

        if (g_wifi.isConnected()) {
            g_wifi_remote.process();
            MQTT::Msg mq;
            while (g_mqtt.receive(mq)) {
                if (strstr(mq.topic, "/car/cmd"))
                    g_mqtt.publish("/car/ack", "ok");
            }
        }

        /* UART 帧解析: STX(0xAA) CMD LEN DATA[] XOR ETX(0x55) */
        static uint8_t rx_buf[64];
        static int     rx_pos = 0;
        static bool    in_frame = false;

        int c;
        while ((c = g_uart.getc()) >= 0) {
            uint8_t b = (uint8_t)c;
            if (!in_frame) {
                if (b == 0xAA) { in_frame = true; rx_pos = 0; }
            } else {
                if (rx_pos < (int)sizeof(rx_buf)) rx_buf[rx_pos++] = b;
                if (rx_pos >= 4 && b == 0x55) {
                    uint8_t cmd  = rx_buf[0];
                    uint8_t len  = rx_buf[1];
                    (void)len;

                    switch (cmd) {
                    case 0x10:
                        g_uart.sendFrame(0x10, nullptr, 0);
                        break;
                    case 0x01: {
                        float imu[10] = { latest.ax, latest.ay, latest.az,
                                          latest.gx, latest.gy, latest.gz,
                                          latest.roll, latest.pitch, latest.yaw,
                                          latest.temp };
                        g_uart.sendFrame(0x01, (uint8_t*)imu, sizeof(imu));
                        break;
                    }
                    case 0x03:
                        break;
                    case 0xFF:
                        g_uart.sendFrame(0xFF, nullptr, 0);
                        esp_restart();
                        break;
                    default:
                        break;
                    }
                    in_frame = false;
                    rx_pos = 0;
                }
                if (rx_pos >= (int)sizeof(rx_buf)) { in_frame = false; rx_pos = 0; }
            }
        }

        if (++tick >= 40) {
            g_led.effect(LED_EFFECT_BREATH, 2);
            tick = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ── 固件入口 ──
 *   硬件初始化 (阻塞) → 创建队列 → 启动双核任务 → 自身退出
 *   任务由 INIT_BIT_HW_READY 事件位同步, 确保硬件就绪后才运行 */
extern "C" void app_main()
{
    vTaskDelay(pdMS_TO_TICKS(2000));
    setvbuf(stdout, NULL, _IONBF, 0);  // stdout 无缓冲, 日志立即输出
    ESP_LOGI(TAG, "===== 固件启动 v5 — LSM6DSV16X SFLP+Mahony =====");

    _init_evt = xEventGroupCreate();

    hw_init();

    /* 短延时确保日志吐出, 立即启动任务 */
    vTaskDelay(pdMS_TO_TICKS(100));

    _sq = xQueueCreate(16, sizeof(Msg));
    _cq = xQueueCreate(8,  sizeof(Msg));

    xTaskCreatePinnedToCore(core0_sensor, "sensor",
        5120, NULL, configMAX_PRIORITIES - 1, NULL, 0);

    xTaskCreatePinnedToCore(core1_ctrl, "control",
        5120, NULL, configMAX_PRIORITIES - 2, NULL, 1);

    g_led.setRGBAll(0, 255, 0);  g_led.update();
    ESP_LOGI(TAG, "双核 C++ 框架启动");
    vTaskDelete(NULL);
}
