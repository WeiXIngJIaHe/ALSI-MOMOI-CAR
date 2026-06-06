#pragma once

/* ================================================================
 * PID 控制器
 *
 * 特性:
 *   - 位置式 PID
 *   - 积分分离 + 限幅 (抗饱和 anti-windup)
 *   - 微分先行 (微分项仅对测量值, 避免设定值突变冲击)
 *   - 输出限幅
 *
 * 用法:
 *   PID pid(1.0f, 0.1f, 0.05f, 5.0f, -100.0f, 100.0f);
 *   loop {
 *       float out = pid.update(setpoint, measurement);
 *       apply(out);
 *   }
 * ================================================================ */

class PID {
public:
    PID(float kp = 0, float ki = 0, float kd = 0,
        float dt_ms = 10, float out_min = -100, float out_max = 100);

    float update(float setpoint, float measurement);
    void  reset();

    /* 在线调参 (自动重置积分) */
    void setGains(float kp, float ki, float kd);
    void setLimits(float min, float max);

    /* 只读 */
    float output()    const { return _output; }
    float setpoint()  const { return _setpoint; }
    float integral()  const { return _integral; }

private:
    float _kp, _ki, _kd;
    float _dt;
    float _out_min, _out_max;

    float _setpoint;
    float _integral;
    float _prev_error;
    float _prev_meas;
    float _output;
};
