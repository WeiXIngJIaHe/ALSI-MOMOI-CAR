#include "DRV8870.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "DRV8870";

void drv8870_init(void)
{
    // TODO: 初始化 PWM 定時器與通道
    ESP_LOGI(TAG, "DRV8870 initialized (PWM freq: %lu Hz)", DRV8870_PWM_FREQ);
}

void drv8870_motor_control(uint8_t motor, uint8_t mode, uint16_t duty)
{
    // TODO: 根據 mode 設置 IN1/IN2 與 PWM duty
}
