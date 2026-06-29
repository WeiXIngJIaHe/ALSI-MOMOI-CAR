#pragma once
#include <stdint.h>
#include "driver/ledc.h"
#include "driver/gpio.h"

/* ================================================================
 * DRV8870 四路电机驱动 — 每电机双LEDC通道
 *
 * ESP32-S3 共8通道: 每电机占用2通道(IN1_PWM + IN2_PWM)
 *   正转: IN1=duty, IN2=0
 *   反转: IN1=0,    IN2=duty
 *   刹车: IN1=0,    IN2=0
 * ================================================================ */

/* ---- 电机 1 (左前) ---- */
#define M1_IN1_PIN                  11
#define M1_IN2_PIN                  12
#define M1_CH_IN1                   LEDC_CHANNEL_0
#define M1_CH_IN2                   LEDC_CHANNEL_1

/* ---- 电机 2 (右前) ---- */
#define M2_IN1_PIN                  45
#define M2_IN2_PIN                  46
#define M2_CH_IN1                   LEDC_CHANNEL_2
#define M2_CH_IN2                   LEDC_CHANNEL_3

/* ---- 电机 3 (左后) ---- */
#define M3_IN1_PIN                  17
#define M3_IN2_PIN                  18
#define M3_CH_IN1                   LEDC_CHANNEL_4
#define M3_CH_IN2                   LEDC_CHANNEL_5

/* ---- 电机 4 (右后) ---- */
#define M4_IN1_PIN                  33
#define M4_IN2_PIN                  34
#define M4_CH_IN1                   LEDC_CHANNEL_6
#define M4_CH_IN2                   LEDC_CHANNEL_7

/* ---- PWM 参数 ---- */
#define DRV8870_PWM_FREQ            20000
#define DRV8870_PWM_RESOLUTION      10
#define DRV8870_PWM_TIMER           LEDC_TIMER_0
#define DRV8870_PWM_MODE            LEDC_LOW_SPEED_MODE
#define DRV8870_DUTY_MAX            1023
#define DRV8870_TEST_PIN            GPIO_NUM_2

class DRV8870 {
public:
    DRV8870(uint8_t in1, uint8_t in2, uint8_t ch1, uint8_t ch2);
    void init();
    void forward(uint16_t duty);
    void reverse(uint16_t duty);
    void brake();
    void coast();
    void setSpeed(int16_t speed);

private:
    uint8_t _in1, _in2, _ch1, _ch2;
    void _write(uint16_t d1, uint16_t d2);
};

/* ================================================================
 * 四电机差分驱动底盘
 * ================================================================ */
class MotorDriver {
public:
    MotorDriver();
    void init();
    void setLeftFront(int16_t s);
    void setRightFront(int16_t s);
    void setLeftBack(int16_t s);
    void setRightBack(int16_t s);
    void setLeft(int16_t speed);
    void setRight(int16_t speed);
    void drive(int16_t speed, int16_t turn);
    bool testCheck();
    void testRun();         /* GPIO3 LOW → 四轮50%PWM, 否则不动 */
    void allStop();
    void allBrake();

    DRV8870 lf, rf, lb, rb;
};
