#pragma once
#include <stdint.h>
#include "IS31FL3239.h"

/* ================================================================
 * LED 子系统 — 全部挂载 IS31FL3239 (36 通道)
 *
 * 每颗 RGB LED 占 3 个通道 [R, G, B]
 * 36 通道 → 12 组 RGB LED
 *
 * 通道映射:
 *   LED[0]  → ch1=R, ch2=G, ch3=B
 *   LED[1]  → ch4=R, ch5=G, ch6=B
 *   ...
 *   LED[11] → ch34=R, ch35=G, ch36=B
 *
 * ┌────── 硬件连线 (请按实际 PCB 填写) ──────┐
 * │ LED[0]  R→OUT__  G→OUT__  B→OUT__        │
 * │ LED[1]  R→OUT__  G→OUT__  B→OUT__        │
 * │ ...                                       │
 * │ LED[11] R→OUT__  G→OUT__  B→OUT__        │
 * └───────────────────────────────────────────┘
 * ================================================================ */

#define LED_CH_PER_RGB              3
#define LED_RGB_COUNT               (IS31FL3239_CH_COUNT / LED_CH_PER_RGB)  /* 12 */
#define LED_BLINK_FAST_MS           100
#define LED_BLINK_SLOW_MS           500

enum LEDEffect {
    LED_EFFECT_OFF     = 0,
    LED_EFFECT_BREATH  = 1,
    LED_EFFECT_MARQUEE = 2,
    LED_EFFECT_RAINBOW = 3,
};


class LED {
public:
    LED(IS31FL3239 &drv);

    void init();

    /* RGB 控制 (每灯 3 通道) */
    void setRGB(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
    void setRGBAll(uint8_t r, uint8_t g, uint8_t b);
    void off(uint8_t index);

    /* 单通道控制 (直通 IS31FL3239) */
    void setChannel(uint8_t ch, uint8_t pwm);
    void setAllChannels(uint8_t pwm);

    /* 全局 */
    void update();
    void clear();
    void setBrightness(uint8_t level);
    void shutdown(bool en);

    /* 效果 */
    void effect(uint8_t mode, uint8_t speed = 4);

private:
    IS31FL3239 &_drv;
};
