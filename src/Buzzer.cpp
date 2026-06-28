#include "Buzzer.h"
#include "esp_log.h"

static const char *TAG = "Buzzer";

/* ================================================================
 * GPIO2 按一次 → 启动警报 (长鸣 2 秒 → 0.1s/0.1s 闪烁循环)
 * GPIO3 按下   → 立即停止, 回 IDLE
 *
 * GPIO2 是边沿触发: 只在按下沿启动, 松开不影响运行
 * GPIO3 在任意状态按下都立即停止
 * ================================================================ */

Buzzer::Buzzer()
    : _state(IDLE), _cnt(0),
      _db2(0), _db3(0), _db2_prev(false)
{}

void Buzzer::init()
{
    ledc_timer_config_t tmr = {};
    tmr.speed_mode      = LEDC_LOW_SPEED_MODE;
    tmr.duty_resolution = (ledc_timer_bit_t)BUZZER_RESOLUTION;
    tmr.timer_num       = BUZZER_LEDC_TIMER;
    tmr.freq_hz         = BUZZER_FREQ_HZ;
    tmr.clk_cfg         = LEDC_AUTO_CLK;
    ledc_timer_config(&tmr);

    ledc_channel_config_t ch = {};
    ch.gpio_num   = (int)BUZZER_PIN;
    ch.speed_mode = LEDC_LOW_SPEED_MODE;
    ch.channel    = BUZZER_LEDC_CHANNEL;
    ch.timer_sel  = BUZZER_LEDC_TIMER;
    ch.duty       = 0;
    ch.hpoint     = 0;
    ch.intr_type  = LEDC_INTR_DISABLE;
    ledc_channel_config(&ch);

    gpio_set_direction(BUZZER_PLAY_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUZZER_PLAY_PIN, GPIO_PULLUP_ONLY);
    gpio_set_direction(BUZZER_STOP_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUZZER_STOP_PIN, GPIO_PULLUP_ONLY);

    ESP_LOGI(TAG, "就绪 (蜂鸣器 GPIO%d, 启动 GPIO%d, 停止 GPIO%d, %dHz)",
             (int)BUZZER_PIN, (int)BUZZER_PLAY_PIN, (int)BUZZER_STOP_PIN,
             BUZZER_FREQ_HZ);
}

void Buzzer::_output(bool on)
{
    uint32_t duty = on ? BUZZER_DUTY_ON : 0;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CHANNEL);
}

void Buzzer::scan()
{
    /* ── 去抖 ── */
    bool raw2 = (gpio_get_level(BUZZER_PLAY_PIN) == 0);
    bool raw3 = (gpio_get_level(BUZZER_STOP_PIN) == 0);

    _db2 = raw2 ? (_db2 < 0xFF ? _db2 + 1 : _db2) : 0;
    _db3 = raw3 ? (_db3 < 0xFF ? _db3 + 1 : _db3) : 0;

    bool btn2 = (_db2 >= BUZZER_DEBOUNCE_CNT);
    bool btn3 = (_db3 >= BUZZER_DEBOUNCE_CNT);

    /* ── GPIO3: 任意状态立即停止 ── */
    if (btn3 && _state != IDLE) {
        _state = IDLE;
        _cnt   = 0;
        _output(false);
        _db2_prev = btn2;
        return;
    }

    /* ── GPIO2 按下沿: 启动警报 ── */
    bool edge2 = (btn2 && !_db2_prev);
    _db2_prev  = btn2;

    if (edge2 && _state == IDLE) {
        _state = ALERT_BEEP_2S;
        _cnt   = 0;
        _output(true);
        return;
    }

    /* ── 状态机 ── */
    switch (_state) {

    case ALERT_BEEP_2S:
        if (++_cnt >= TICK_2S) {
            _state = ALERT_FLASH_ON;
            _cnt   = 0;
            _output(true);
        }
        break;

    case ALERT_FLASH_ON:
        if (++_cnt >= TICK_01S) {
            _state = ALERT_FLASH_OFF;
            _cnt   = 0;
            _output(false);
        }
        break;

    case ALERT_FLASH_OFF:
        if (++_cnt >= TICK_01S) {
            _state = ALERT_FLASH_ON;
            _cnt   = 0;
            _output(true);
        }
        break;

    default:
        break;
    }
}
