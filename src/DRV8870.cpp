#include "DRV8870.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include <algorithm>

static const char *TAG = "DRV8870";

/* ================================================================ */
/* 单电机                                                             */
/* ================================================================ */

DRV8870::DRV8870(uint8_t in1, uint8_t in2, uint8_t pwm_ch)
    : _in1(in1), _in2(in2), _pwm_ch(pwm_ch)
{}

void DRV8870::init()
{
    /* GPIO 推挽输出 */
    gpio_set_direction((gpio_num_t)_in1, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)_in2, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)_in1, 0);
    gpio_set_level((gpio_num_t)_in2, 0);

    /* LEDC PWM 通道 */
    ledc_channel_config_t ch = {};
    ch.gpio_num   = _in1;    /* 初始绑定到 IN1 (正转时 PWM 在 IN1) */
    ch.speed_mode = DRV8870_PWM_MODE;
    ch.channel    = (ledc_channel_t)_pwm_ch;
    ch.timer_sel  = DRV8870_PWM_TIMER;
    ch.duty       = 0;
    ch.hpoint     = 0;
    ch.intr_type  = LEDC_INTR_DISABLE;
    ledc_channel_config(&ch);

    ESP_LOGI(TAG, "电机 (IN1=%d IN2=%d ch=%d)", _in1, _in2, _pwm_ch);
}

/*
 * 正转: IN2=L, PWM 切换到 IN1 输出
 * DRV8870 的 PWM 控制端必须是高电平期间有效, 所以:
 *   - 正转时 IN1=PWM, IN2=L
 */
void DRV8870::forward(uint16_t duty)
{
    if (duty > DRV8870_DUTY_MAX) duty = DRV8870_DUTY_MAX;

    gpio_set_level((gpio_num_t)_in2, 0);

    /* 将 PWM 输出改绑到 IN1 */
    ledc_stop(DRV8870_PWM_MODE, (ledc_channel_t)_pwm_ch, 0);
    ledc_set_pin(_in1, DRV8870_PWM_MODE, (ledc_channel_t)_pwm_ch);

    ledc_set_duty(DRV8870_PWM_MODE, (ledc_channel_t)_pwm_ch, duty);
    ledc_update_duty(DRV8870_PWM_MODE, (ledc_channel_t)_pwm_ch);
}

/*
 * 反转: IN1=L, PWM 切换到 IN2 输出
 */
void DRV8870::reverse(uint16_t duty)
{
    if (duty > DRV8870_DUTY_MAX) duty = DRV8870_DUTY_MAX;

    gpio_set_level((gpio_num_t)_in1, 0);

    ledc_stop(DRV8870_PWM_MODE, (ledc_channel_t)_pwm_ch, 0);
    ledc_set_pin(_in2, DRV8870_PWM_MODE, (ledc_channel_t)_pwm_ch);

    ledc_set_duty(DRV8870_PWM_MODE, (ledc_channel_t)_pwm_ch, duty);
    ledc_update_duty(DRV8870_PWM_MODE, (ledc_channel_t)_pwm_ch);
}

/* 刹车: IN1=L, IN2=L, 电机两端短接 → 反电动势制动 */
void DRV8870::brake()
{
    ledc_stop(DRV8870_PWM_MODE, (ledc_channel_t)_pwm_ch, 0);
    gpio_set_level((gpio_num_t)_in1, 0);
    gpio_set_level((gpio_num_t)_in2, 0);
}

/* 惯性滑行: IN1=H, IN2=H, 高阻态 */
void DRV8870::coast()
{
    ledc_stop(DRV8870_PWM_MODE, (ledc_channel_t)_pwm_ch, 0);
    gpio_set_level((gpio_num_t)_in1, 1);
    gpio_set_level((gpio_num_t)_in2, 1);
}

void DRV8870::stop()
{
    coast();
}

/* PID 友好: speed ∈ [-1023, +1023] */
void DRV8870::setSpeed(int16_t speed)
{
    if (speed >  DRV8870_DUTY_MAX) speed =  DRV8870_DUTY_MAX;
    if (speed < -DRV8870_DUTY_MAX) speed = -DRV8870_DUTY_MAX;

    if (speed > 20) {
        forward((uint16_t)speed);
    } else if (speed < -20) {
        reverse((uint16_t)(-speed));
    } else {
        brake();   /* 死区: 无指令时制动, 防止溜车 */
    }
}

/* ================================================================ */
/* 双电机差分驱动                                                      */
/* ================================================================ */

MotorDriver::MotorDriver()
    : left(M1_IN1_PIN, M1_IN2_PIN, M1_PWM_CHANNEL),
      right(M2_IN1_PIN, M2_IN2_PIN, M2_PWM_CHANNEL)
{}

void MotorDriver::init()
{
    /* LEDC 定时器 (两个电机共用) */
    ledc_timer_config_t tmr = {};
    tmr.speed_mode      = DRV8870_PWM_MODE;
    tmr.duty_resolution = (ledc_timer_bit_t)DRV8870_PWM_RESOLUTION;
    tmr.timer_num       = DRV8870_PWM_TIMER;
    tmr.freq_hz         = DRV8870_PWM_FREQ;
    tmr.clk_cfg         = LEDC_AUTO_CLK;
    ledc_timer_config(&tmr);

    left.init();
    right.init();

    ESP_LOGI(TAG, "双电机驱动就绪 (PWM %dHz/%d-bit)",
             DRV8870_PWM_FREQ, DRV8870_PWM_RESOLUTION);
}

void MotorDriver::setLeft(int16_t speed)  { left.setSpeed(speed); }
void MotorDriver::setRight(int16_t speed) { right.setSpeed(speed); }

/*
 * 差分驱动:
 *   speed = 线速度 (-1023~+1023)
 *   turn  = 转弯量  (-1023~+1023, >0 右转)
 *
 *   左轮 = speed + turn
 *   右轮 = speed - turn
 *   两者都会被限幅到 [-1023, +1023]
 */
void MotorDriver::drive(int16_t speed, int16_t turn)
{
    int16_t l = speed + turn;
    int16_t r = speed - turn;

    /* 限幅 */
    if (l >  1023) l =  1023;
    if (l < -1023) l = -1023;
    if (r >  1023) r =  1023;
    if (r < -1023) r = -1023;

    setLeft(l);
    setRight(r);
}

void MotorDriver::allStop()  { left.stop();  right.stop(); }
void MotorDriver::allBrake() { left.brake(); right.brake(); }
