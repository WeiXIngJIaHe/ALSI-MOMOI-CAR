#ifndef DRV8870_H
#define DRV8870_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* DRV8870 H桥电机驱动 */

/* 电机1 */
#define DRV8870_MOTOR1_IN1_PIN      21
#define DRV8870_MOTOR1_IN2_PIN      22

/* 电机2 */
#define DRV8870_MOTOR2_IN1_PIN      23
#define DRV8870_MOTOR2_IN2_PIN      24

/* PWM */
#define DRV8870_PWM_FREQ            20000   /* 20kHz */
#define DRV8870_PWM_RESOLUTION      10      /* 10-bit (0-1023) */
#define DRV8870_PWM_MOTOR1_CHANNEL  LEDC_CHANNEL_0
#define DRV8870_PWM_MOTOR2_CHANNEL  LEDC_CHANNEL_1
#define DRV8870_PWM_MODE            LEDC_LOW_SPEED_MODE
#define DRV8870_PWM_TIMER           LEDC_TIMER_0

/* 电机状态 */
#define DRV8870_MODE_STOP           0x00    /* 刹车 */
#define DRV8870_MODE_FORWARD        0x01    /* 正转 */
#define DRV8870_MODE_REVERSE        0x02    /* 反转 */

/* 占空比 */
#define DRV8870_DUTY_CYCLE_MAX      1023
#define DRV8870_DUTY_CYCLE_HALF     512
#define DRV8870_DUTY_CYCLE_MIN      0

/* API */
void drv8870_init(void);
void drv8870_motor_control(uint8_t motor, uint8_t mode, uint16_t duty);

#ifdef __cplusplus
}
#endif

#endif
