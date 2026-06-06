#include "buzzer.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "Buzzer";

Buzzer::Buzzer(uint8_t pin, uint8_t ch, uint8_t timer)
    : _pin(pin), _ch(ch), _timer(timer), _volume(50) {}

void Buzzer::init()
{
    ledc_timer_config_t tmr = {};
    tmr.speed_mode      = LEDC_LOW_SPEED_MODE;
    tmr.duty_resolution = LEDC_TIMER_10_BIT;
    tmr.timer_num       = (ledc_timer_t)_timer;
    tmr.freq_hz         = BUZZER_DEFAULT_FREQ;
    tmr.clk_cfg         = LEDC_AUTO_CLK;
    ledc_timer_config(&tmr);

    ledc_channel_config_t ch = {};
    ch.gpio_num   = _pin;
    ch.speed_mode = LEDC_LOW_SPEED_MODE;
    ch.channel    = (ledc_channel_t)_ch;
    ch.timer_sel  = (ledc_timer_t)_timer;
    ch.duty       = 0;
    ch.hpoint     = 0;
    ledc_channel_config(&ch);

    ESP_LOGI(TAG, "OK (GPIO %d)", _pin);
}

void Buzzer::setVolume(uint8_t vol)
{
    _volume = (vol > 100) ? 100 : vol;
}

void Buzzer::tone(uint16_t freq, uint16_t ms)
{
    if (freq == 0) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)_ch, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)_ch);
        vTaskDelay(pdMS_TO_TICKS(ms));
        return;
    }

    ledc_set_freq(LEDC_LOW_SPEED_MODE, (ledc_timer_t)_timer, freq);
    uint32_t duty = (512 * _volume) / 100;   /* 10-bit: 50% → 512 */
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)_ch, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)_ch);
    vTaskDelay(pdMS_TO_TICKS(ms));
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)_ch, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)_ch);
}

void Buzzer::toneStart(uint16_t freq)
{
    ledc_set_freq(LEDC_LOW_SPEED_MODE, (ledc_timer_t)_timer, freq);
    uint32_t duty = (512 * _volume) / 100;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)_ch, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)_ch);
}

void Buzzer::toneStop()
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)_ch, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)_ch);
}

void Buzzer::beep(uint16_t ms)
{
    tone(BUZZER_DEFAULT_FREQ, ms);
}

void Buzzer::playMelody(const uint16_t *notes, uint16_t count)
{
    for (uint16_t i = 0; i < count; i += 2) {
        uint16_t freq = notes[i];
        uint16_t dur  = notes[i + 1];
        tone(freq, dur);
        vTaskDelay(pdMS_TO_TICKS(20));   /* 音符间间隙 */
    }
}
