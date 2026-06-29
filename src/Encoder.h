#pragma once
#include <stdint.h>
#include "driver/pulse_cnt.h"
#include "hal/gpio_types.h"

/* ================================================================
 * AB 相霍尔编码器 — ESP32 PCNT 硬件脉冲计数 (v5.x API)
 * ================================================================ */

#define ENC_FL_A    GPIO_NUM_1
#define ENC_FL_B    GPIO_NUM_2
#define ENC_FR_A    GPIO_NUM_3
#define ENC_FR_B    GPIO_NUM_4
#define ENC_BL_A    GPIO_NUM_5
#define ENC_BL_B    GPIO_NUM_6
#define ENC_BR_A    GPIO_NUM_7
#define ENC_BR_B    GPIO_NUM_8

#define ENC_PPR         11
#define ENC_GEAR_RATIO  30
#define ENC_CPR         (ENC_PPR * ENC_GEAR_RATIO)

class ABEncoder {
public:
    ABEncoder(int a, int b);

    void     init();
    int32_t  count();
    void     reset();
    int32_t  speed();          /* 脉冲/秒 */
    float    rotations();      /* 累计圈数 */
    float    speedRPM();       /* RPM */

private:
    pcnt_unit_handle_t _unit;
    int         _a, _b;
    int32_t     _last_count;
    int64_t     _last_time;
};

class Encoders {
public:
    Encoders();
    void init();
    void resetAll();

    int32_t countFL(); int32_t countFR();
    int32_t countBL(); int32_t countBR();
    int32_t speedFL(); int32_t speedFR();
    int32_t speedBL(); int32_t speedBR();

    ABEncoder fl, fr, bl, br;
};
