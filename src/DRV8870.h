#pragma once
#include <stdint.h>
#include "driver/ledc.h"
#include "driver/gpio.h"

/* ================================================================
 * DRV8870 双路 H 桥电机驱动 (PWM 控制)
 *
 * DRV8870 控制逻辑:
 *   IN1=L  IN2=L  → 刹车 (Brake, 电机两端短接到 GND)
 *   IN1=H  IN2=L  → 正转 (Forward, 电流 IN1→电机→IN2)
 *   IN1=L  IN2=H  → 反转 (Reverse, 电流 IN2→电机→IN1)
 *   IN1=H  IN2=H  → 惯性滑行 (Coast, 高阻态)
 *
 * PWM: IN1 或 IN2 上施加 PWM 来控制转速
 *   正转时 PWM 在 IN1 上, IN2=L; 反转时 PWM 在 IN2 上, IN1=L
 *
 * 直接接入 PID 输出:
 *   setSpeed(idx, pid_output);
 *   pid_output 范围 [-max, +max] → 自动映射到方向和占空比
 * ================================================================ */

/* ---- 电机 1 (左轮) ---- */
#define M1_IN1_PIN                  21
#define M1_IN2_PIN                  22
#define M1_PWM_CHANNEL              LEDC_CHANNEL_0

/* ---- 电机 2 (右轮) ---- */
#define M2_IN1_PIN                  23
#define M2_IN2_PIN                  24
#define M2_PWM_CHANNEL              LEDC_CHANNEL_1

/* ---- PWM 参数 ---- */
#define DRV8870_PWM_FREQ            20000       /* 20kHz, 超听觉范围 */
#define DRV8870_PWM_RESOLUTION      10          /* 10-bit (0~1023) */
#define DRV8870_PWM_TIMER           LEDC_TIMER_0
#define DRV8870_PWM_MODE            LEDC_LOW_SPEED_MODE

#define DRV8870_DUTY_MAX            1023
#define DRV8870_DUTY_OFF            0


class DRV8870 {
public:
    /*
     * 单个电机构造
     * in1/in2: 方向控制 GPIO
     * pwm_ch:  LEDC 通道编号 (两个电机不能共用通道)
     */
    DRV8870(uint8_t in1, uint8_t in2, uint8_t pwm_ch);

    void init();

    /* ── 底层控制 ── */

    /* 直接设置方向和占空比 (duty: 0~1023) */
    void forward(uint16_t duty);
    void reverse(uint16_t duty);
    void brake();                                   /* 刹车 */
    void coast();                                   /* 惯性滑行 */
    void stop();                                    /* 停止 (PWM 输出 0) */

    /* ── PID 友好接口 ── */

    /*
     * 接收 PID 输出值, 自动映射到方向和占空比
     * 速度范围 -1023..0..+1023
     *   > 0 → 正转, PWM = speed
     *   < 0 → 反转, PWM = -speed
     *   = 0 → 刹车
     */
    void setSpeed(int16_t speed);

private:
    uint8_t _in1, _in2;
    uint8_t _pwm_ch;
};

/* ================================================================
 * 双电机便捷封装 (差分驱动底盘)
 * ================================================================ */

class MotorDriver {
public:
    MotorDriver();

    void init();

    /* 单轮速度 (-1023~+1023) */
    void setLeft(int16_t speed);
    void setRight(int16_t speed);

    /* 差分驱动: 线速度 + 角速度 → 双轮速度 */
    void drive(int16_t speed, int16_t turn);         /* turn>0=右转 */

    /* 全部停止 */
    void allStop();
    void allBrake();

    DRV8870 left;    /* 电机1 (左轮) */
    DRV8870 right;   /* 电机2 (右轮) */
};
