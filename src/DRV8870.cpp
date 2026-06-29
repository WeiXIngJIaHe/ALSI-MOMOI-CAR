#include "DRV8870.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "DRV8870";

/* ================================================================
 * 单电机 — 双 LEDC 通道
 *   IN1 和 IN2 各有独立 LEDC 通道, 无需动态切换引脚
 *   forward: 停 IN2 通道 → IN1 输出 PWM
 *   reverse: 停 IN1 通道 → IN2 输出 PWM
 *   brake:   双通道停止, 输出 LOW
 * ================================================================ */

DRV8870::DRV8870(uint8_t in1, uint8_t in2, uint8_t ch1, uint8_t ch2)
    : _in1(in1), _in2(in2), _ch1(ch1), _ch2(ch2)
{}



void DRV8870::init()
{
    ledc_channel_config_t cfg = {};
    cfg.speed_mode = DRV8870_PWM_MODE;
    cfg.timer_sel  = DRV8870_PWM_TIMER;
    cfg.duty       = 0;
    cfg.hpoint     = 0;
    cfg.intr_type  = LEDC_INTR_DISABLE;

    cfg.gpio_num = _in1;
    cfg.channel  = (ledc_channel_t)_ch1;
    ESP_ERROR_CHECK(ledc_channel_config(&cfg));

    cfg.gpio_num = _in2;
    cfg.channel  = (ledc_channel_t)_ch2;
    ESP_ERROR_CHECK(ledc_channel_config(&cfg));

    /* 初始高阻: 双通道100%占空比 */
    ledc_set_duty(DRV8870_PWM_MODE, (ledc_channel_t)_ch1, DRV8870_DUTY_MAX);
    ledc_set_duty(DRV8870_PWM_MODE, (ledc_channel_t)_ch2, DRV8870_DUTY_MAX);
    ledc_update_duty(DRV8870_PWM_MODE, (ledc_channel_t)_ch1);
    ledc_update_duty(DRV8870_PWM_MODE, (ledc_channel_t)_ch2);

    ESP_LOGI(TAG, "motor ch%d/%d GPIO%d/%d OK", _ch1, _ch2, _in1, _in2);
}

void DRV8870::forward(uint16_t duty)
{
    if (duty > DRV8870_DUTY_MAX) duty = DRV8870_DUTY_MAX;
    ledc_set_duty(DRV8870_PWM_MODE, (ledc_channel_t)_ch1, duty);
    ledc_set_duty(DRV8870_PWM_MODE, (ledc_channel_t)_ch2, 0);
    ledc_update_duty(DRV8870_PWM_MODE, (ledc_channel_t)_ch1);
    ledc_update_duty(DRV8870_PWM_MODE, (ledc_channel_t)_ch2);
}

void DRV8870::reverse(uint16_t duty)
{
    if (duty > DRV8870_DUTY_MAX) duty = DRV8870_DUTY_MAX;
    ledc_set_duty(DRV8870_PWM_MODE, (ledc_channel_t)_ch1, 0);
    ledc_set_duty(DRV8870_PWM_MODE, (ledc_channel_t)_ch2, duty);
    ledc_update_duty(DRV8870_PWM_MODE, (ledc_channel_t)_ch1);
    ledc_update_duty(DRV8870_PWM_MODE, (ledc_channel_t)_ch2);
}

void DRV8870::brake()
{
    ledc_set_duty(DRV8870_PWM_MODE, (ledc_channel_t)_ch1, 0);
    ledc_set_duty(DRV8870_PWM_MODE, (ledc_channel_t)_ch2, 0);
    ledc_update_duty(DRV8870_PWM_MODE, (ledc_channel_t)_ch1);
    ledc_update_duty(DRV8870_PWM_MODE, (ledc_channel_t)_ch2);
}

void DRV8870::coast()
{
    ledc_set_duty(DRV8870_PWM_MODE, (ledc_channel_t)_ch1, DRV8870_DUTY_MAX);
    ledc_set_duty(DRV8870_PWM_MODE, (ledc_channel_t)_ch2, DRV8870_DUTY_MAX);
    ledc_update_duty(DRV8870_PWM_MODE, (ledc_channel_t)_ch1);
    ledc_update_duty(DRV8870_PWM_MODE, (ledc_channel_t)_ch2);
}

void DRV8870::setSpeed(int16_t speed)
{
    if (speed >  DRV8870_DUTY_MAX) speed =  DRV8870_DUTY_MAX;
    if (speed < -DRV8870_DUTY_MAX) speed = -DRV8870_DUTY_MAX;

    if (speed > 20) {
        forward((uint16_t)speed);
    } else if (speed < -20) {
        reverse((uint16_t)(-speed));
    } else {
        coast();
    }
}

/* ================================================================ */
/* 四电机差分驱动                                                      */
/* ================================================================ */

MotorDriver::MotorDriver()
    : lf(M1_IN1_PIN, M1_IN2_PIN, M1_CH_IN1, M1_CH_IN2)
    , rf(M2_IN1_PIN, M2_IN2_PIN, M2_CH_IN1, M2_CH_IN2)
    , lb(M3_IN1_PIN, M3_IN2_PIN, M3_CH_IN1, M3_CH_IN2)
    , rb(M4_IN1_PIN, M4_IN2_PIN, M4_CH_IN1, M4_CH_IN2)
{}

void MotorDriver::init()
{
    ledc_timer_config_t tmr = {};
    tmr.speed_mode      = DRV8870_PWM_MODE;
    tmr.duty_resolution = (ledc_timer_bit_t)DRV8870_PWM_RESOLUTION;
    tmr.timer_num       = DRV8870_PWM_TIMER;
    tmr.freq_hz         = DRV8870_PWM_FREQ;
    tmr.clk_cfg         = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&tmr));

    /* 所有电机 GPIO 先复位为高阻输入, 防止上电瞬间电机抖动 */
    gpio_reset_pin(GPIO_NUM_11); gpio_reset_pin(GPIO_NUM_12);
    gpio_reset_pin(GPIO_NUM_33); gpio_reset_pin(GPIO_NUM_34);

    /* GPIO45/46: 禁用 Strapping 功能 */
    gpio_reset_pin(GPIO_NUM_45);
    gpio_reset_pin(GPIO_NUM_46);
    gpio_hold_dis(GPIO_NUM_45);
    gpio_hold_dis(GPIO_NUM_46);

    /* GPIO17/18: 禁用 UART2 功能 (U2RXD/U2TXD) */
    gpio_reset_pin(GPIO_NUM_17);
    gpio_reset_pin(GPIO_NUM_18);
    gpio_hold_dis(GPIO_NUM_17);
    gpio_hold_dis(GPIO_NUM_18);
    gpio_sleep_sel_dis(GPIO_NUM_17);
    gpio_sleep_sel_dis(GPIO_NUM_18);

    gpio_set_direction(DRV8870_TEST_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(DRV8870_TEST_PIN, GPIO_PULLUP_ONLY);

    /* PWM 接管 GPIO (gpio_reset_pin 已将 IO MUX 切回 GPIO) */
    lf.init(); rf.init(); lb.init(); rb.init();

    ESP_LOGI(TAG, "4WD ready PWM=%dHz test=GPIO%d", DRV8870_PWM_FREQ, DRV8870_TEST_PIN);
}

bool MotorDriver::testCheck()
{
    return gpio_get_level(DRV8870_TEST_PIN) == 0;
}

void MotorDriver::testRun()
{
    if (testCheck()) {
        int16_t t = DRV8870_DUTY_MAX * 9 / 10;   /* 90% */
        lf.forward(t); rf.forward(t);
        lb.forward(t); rb.forward(t);
    }
}

void MotorDriver::setLeftFront(int16_t s)  { lf.setSpeed(s); }
void MotorDriver::setRightFront(int16_t s) { rf.setSpeed(s); }
void MotorDriver::setLeftBack(int16_t s)   { lb.setSpeed(s); }
void MotorDriver::setRightBack(int16_t s)  { rb.setSpeed(s); }
void MotorDriver::setLeft(int16_t speed)   { lf.setSpeed(speed); lb.setSpeed(speed); }
void MotorDriver::setRight(int16_t speed)  { rf.setSpeed(speed); rb.setSpeed(speed); }

void MotorDriver::drive(int16_t speed, int16_t turn)
{
    if (testCheck()) {
        int16_t t = DRV8870_DUTY_MAX * 5 / 10;   /* 90% */
        lf.forward(t); rf.forward(t);
        lb.forward(t); rb.forward(t);
        return;
    }

    int16_t l = speed + turn;
    int16_t r = speed - turn;
    if (l >  1023) l =  1023;
    if (l < -1023) l = -1023;
    if (r >  1023) r =  1023;
    if (r < -1023) r = -1023;
    setLeft(l);
    setRight(r);
}

void MotorDriver::allStop()  { lf.brake(); rf.brake(); lb.brake(); rb.brake(); }
void MotorDriver::allBrake() { lf.brake(); rf.brake(); lb.brake(); rb.brake(); }
