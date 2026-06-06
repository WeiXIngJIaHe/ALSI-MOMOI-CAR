#include "LED.h"
#include "esp_log.h"

static const char *TAG = "LED";

LED::LED(IS31FL3239 &drv) : _drv(drv) {}

void LED::init()
{
    clear();
    ESP_LOGI(TAG, "OK (IS31FL3239, %d 通道 → %d RGB)", IS31FL3239_CH_COUNT, LED_RGB_COUNT);
}

/* ── RGB 控制 ── */

void LED::setRGB(uint8_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= LED_RGB_COUNT) return;
    uint8_t base = index * LED_CH_PER_RGB;
    _drv.setChannel(base + 1, r);
    _drv.setChannel(base + 2, g);
    _drv.setChannel(base + 3, b);
}

void LED::setRGBAll(uint8_t r, uint8_t g, uint8_t b)
{
    for (uint8_t i = 0; i < LED_RGB_COUNT; i++) {
        setRGB(i, r, g, b);
    }
}

void LED::off(uint8_t index)
{
    setRGB(index, 0, 0, 0);
}

/* ── 单通道 ── */

void LED::setChannel(uint8_t ch, uint8_t pwm)
{
    _drv.setChannel(ch, pwm);
}

void LED::setAllChannels(uint8_t pwm)
{
    _drv.setAll(pwm);
}

/* ── 全局 ── */

void LED::update()
{
    _drv.update();
}

void LED::clear()
{
    _drv.clear();
}

void LED::setBrightness(uint8_t level)
{
    _drv.setCurrent(level);
}

void LED::shutdown(bool en)
{
    _drv.shutdown(en);
}

/* ── 效果引擎 ── */

void LED::effect(uint8_t mode, uint8_t speed)
{
    static int   phase = 0;
    static int   breath_val = 0;
    static int8_t dir = 1;
    static uint8_t rainbow_hue = 0;

    phase++;

    switch (mode) {
    case LED_EFFECT_OFF:
        clear();
        break;

    case LED_EFFECT_BREATH:
        breath_val += dir * (speed + 1);
        if (breath_val >= 255) { breath_val = 255; dir = -1; }
        if (breath_val <= 0)   { breath_val = 0;   dir =  1; }
        setRGBAll(breath_val, breath_val, breath_val);
        update();
        break;

    case LED_EFFECT_MARQUEE:
        setRGBAll(0, 0, 0);
        setRGB(phase % LED_RGB_COUNT, 255, 255, 255);
        update();
        break;

    case LED_EFFECT_RAINBOW:
        rainbow_hue += speed;
        {
            uint8_t r = (rainbow_hue        * 3) & 0xFF;
            uint8_t g = (rainbow_hue * 3 + 85)  & 0xFF;
            uint8_t b = (rainbow_hue * 3 + 170) & 0xFF;
            setRGBAll(r, g, b);
            update();
        }
        break;
    }
}
