#include "Encoder.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "Encoder";

/* ================================================================
 * ABEncoder
 * ================================================================ */

ABEncoder::ABEncoder(int a, int b)
    : _unit(nullptr), _a(a), _b(b), _last_count(0), _last_time(0)
{}

void ABEncoder::init()
{
    pcnt_unit_config_t unit_cfg = {};
    unit_cfg.high_limit = 32767;
    unit_cfg.low_limit  = -32768;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &_unit));

    pcnt_glitch_filter_config_t flt = {};
    flt.max_glitch_ns = 1000;
    pcnt_unit_set_glitch_filter(_unit, &flt);

    pcnt_chan_config_t ch = {};
    ch.edge_gpio_num  = _a;
    ch.level_gpio_num = _b;
    pcnt_channel_handle_t ch_handle = nullptr;
    ESP_ERROR_CHECK(pcnt_new_channel(_unit, &ch, &ch_handle));

    /* v5.5 API: pos_act, neg_act (2 params) */
    pcnt_channel_set_edge_action(ch_handle,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(ch_handle,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    pcnt_unit_enable(_unit);
    pcnt_unit_clear_count(_unit);
    pcnt_unit_start(_unit);

    _last_time = esp_timer_get_time();

    int val = 0;
    pcnt_unit_get_count(_unit, &val);
    ESP_LOGI(TAG, "enc A=GPIO%d B=GPIO%d OK count=%d", _a, _b, val);
}

int32_t ABEncoder::count()
{
    int val = 0;
    pcnt_unit_get_count(_unit, &val);
    return val;
}

void ABEncoder::reset()
{
    pcnt_unit_clear_count(_unit);
    _last_count = 0;
    _last_time  = esp_timer_get_time();
}

int32_t ABEncoder::speed()
{
    int32_t now = count();
    int64_t t   = esp_timer_get_time();
    int64_t dt  = t - _last_time;
    int32_t spd = 0;
    if (dt > 0)
        spd = (int32_t)((now - _last_count) * 1000000LL / dt);
    _last_count = now;
    _last_time  = t;
    return spd;
}

float ABEncoder::rotations()  { return (float)count() / (float)ENC_CPR; }
float ABEncoder::speedRPM()   { return (float)speed() * 60.0f / (float)ENC_CPR; }

/* ================================================================
 * Encoders
 * ================================================================ */

Encoders::Encoders()
    : fl(ENC_FL_A, ENC_FL_B)
    , fr(ENC_FR_A, ENC_FR_B)
    , bl(ENC_BL_A, ENC_BL_B)
    , br(ENC_BR_A, ENC_BR_B)
{}

void Encoders::init()
{
    fl.init(); fr.init(); bl.init(); br.init();
    ESP_LOGI(TAG, "4 encoders ready CPR=%d", ENC_CPR);
}

void Encoders::resetAll() { fl.reset(); fr.reset(); bl.reset(); br.reset(); }

int32_t Encoders::countFL() { return fl.count(); }
int32_t Encoders::countFR() { return fr.count(); }
int32_t Encoders::countBL() { return bl.count(); }
int32_t Encoders::countBR() { return br.count(); }

int32_t Encoders::speedFL() { return fl.speed(); }
int32_t Encoders::speedFR() { return fr.speed(); }
int32_t Encoders::speedBL() { return bl.speed(); }
int32_t Encoders::speedBR() { return br.speed(); }
