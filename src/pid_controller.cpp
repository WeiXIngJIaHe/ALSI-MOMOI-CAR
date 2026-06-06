#include "pid_controller.h"
#include <cmath>

PID::PID(float kp, float ki, float kd, float dt_ms, float out_min, float out_max)
    : _kp(kp), _ki(ki), _kd(kd),
      _dt(dt_ms / 1000.0f),
      _out_min(out_min), _out_max(out_max),
      _setpoint(0), _integral(0), _prev_error(0), _prev_meas(0), _output(0)
{}

float PID::update(float setpoint, float measurement)
{
    _setpoint = setpoint;

    /* 比例 */
    float error  = setpoint - measurement;
    float p_term = _kp * error;

    /* 积分 (限幅防饱和) */
    _integral += error * _dt;
    if (_ki > 0.001f) {
        float i_limit = (_out_max - _out_min) / _ki;
        if (_integral >  i_limit) _integral =  i_limit;
        if (_integral < -i_limit) _integral = -i_limit;
    }
    float i_term = _ki * _integral;

    /* 微分 (微分先行: 对测量值微分) */
    float d_term = _kd * (_prev_meas - measurement) / _dt;
    _prev_meas = measurement;

    /* 合成 + 限幅 */
    _output = p_term + i_term + d_term;
    if (_output > _out_max) _output = _out_max;
    if (_output < _out_min) _output = _out_min;

    _prev_error = error;
    return _output;
}

void PID::reset()
{
    _integral   = 0;
    _prev_error = 0;
    _prev_meas  = 0;
    _output     = 0;
}

void PID::setGains(float kp, float ki, float kd)
{
    _kp = kp; _ki = ki; _kd = kd;
    reset();
}

void PID::setLimits(float min, float max)
{
    _out_min = min; _out_max = max;
}
