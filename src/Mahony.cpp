/* ================================================================
 * Mahony AHRS — 互补滤波器实现
 *
 * 算法流程 (每步):
 *   1. 归一化加速度计向量
 *   2. 从当前四元数计算重力方向投影
 *   3. 加速度计向量 × 重力投影 → 姿态误差 (叉积)
 *   4. PI 控制器: 误差 → 陀螺仪修正量
 *   5. 修正后的角速度 → 四元数积分 (一阶龙格-库塔)
 *   6. 四元数归一化
 * ================================================================ */

#include "Mahony.h"
#include <cmath>

/* ESP32-S3 64-bit 微秒计时器 */
extern "C" {
#include "esp_timer.h"
}

Mahony::Mahony()
    : _q0(1.0f), _q1(0.0f), _q2(0.0f), _q3(0.0f),
      _kp(0.5f), _ki(0.0f),
      _halfSampleFreq(100.0f),
      _ibx(0), _iby(0), _ibz(0),
      _lastUs(0), _timed(false)
{}

void Mahony::begin(float sampleFreq, float kp, float ki)
{
    _kp             = kp;
    _ki             = ki;
    _halfSampleFreq = sampleFreq * 0.5f;
    reset();
}

void Mahony::setGains(float kp, float ki)
{
    _kp = kp;
    _ki = ki;
}

void Mahony::reset()
{
    _q0 = 1.0f;  _q1 = 0.0f;  _q2 = 0.0f;  _q3 = 0.0f;
    _ibx = 0.0f; _iby = 0.0f; _ibz = 0.0f;
    _lastUs = 0;
    _timed  = false;
}

/* ================================================================
 * 核心: 单步更新
 * ================================================================ */

void Mahony::update(float ax, float ay, float az,
                    float gx, float gy, float gz,
                    float dt)
{
    /* ── 0. 时间步长保护 ── */
    if (dt <= 0.0f || dt > 1.0f) return;

    /* ── 1. 归一化加速度计 ── */
    float norm = std::sqrt(ax * ax + ay * ay + az * az);
    if (norm < 1e-6f) return;   /* 自由落体/静止, 不更新 */
    float invNorm = 1.0f / norm;
    ax *= invNorm;
    ay *= invNorm;
    az *= invNorm;

    /* ── 2. 从四元数求重力方向 (世界坐标系 Z 轴在机体坐标系的投影) ── */
    float vx = 2.0f * (_q1 * _q3 - _q0 * _q2);
    float vy = 2.0f * (_q0 * _q1 + _q2 * _q3);
    float vz = _q0 * _q0 - _q1 * _q1 - _q2 * _q2 + _q3 * _q3;

    /* ── 3. 误差 = 加速度计 × 重力投影 (叉积) ── */
    float ex = (ay * vz - az * vy);
    float ey = (az * vx - ax * vz);
    float ez = (ax * vy - ay * vx);

    /* ── 4. PI 控制器 ── */
    /* 积分项 (补偿陀螺仪零偏漂移) */
    if (_ki > 0.0f) {
        _ibx += _ki * ex * dt;
        _iby += _ki * ey * dt;
        _ibz += _ki * ez * dt;
    }

    /* 修正后的角速度 = 原始值 + 比例修正 + 积分零偏补偿 */
    gx += _kp * ex + _ibx;
    gy += _kp * ey + _iby;
    gz += _kp * ez + _ibz;

    /* ── 5. 四元数积分 (一阶) ── */
    float halfDt = dt * 0.5f;
    _q0 += (-_q1 * gx - _q2 * gy - _q3 * gz) * halfDt;
    _q1 += ( _q0 * gx + _q2 * gz - _q3 * gy) * halfDt;
    _q2 += ( _q0 * gy - _q1 * gz + _q3 * gx) * halfDt;
    _q3 += ( _q0 * gz + _q1 * gy - _q2 * gx) * halfDt;

    /* ── 6. 四元数归一化 ── */
    norm = std::sqrt(_q0 * _q0 + _q1 * _q1 + _q2 * _q2 + _q3 * _q3);
    if (norm < 1e-6f) { reset(); return; }
    invNorm = 1.0f / norm;
    _q0 *= invNorm;
    _q1 *= invNorm;
    _q2 *= invNorm;
    _q3 *= invNorm;
}

/* ================================================================
 * 便捷接口: 自动 dt (使用 esp_timer)
 * ================================================================ */

void Mahony::updateIMU(float ax, float ay, float az,
                       float gx, float gy, float gz)
{
    int64_t now = esp_timer_get_time();  /* μs */

    if (!_timed) {
        _lastUs = now;
        _timed  = true;
        return;   /* 首次调用仅记录时间 */
    }

    float dt = (float)(now - _lastUs) * 1e-6f;  /* μs → s */
    _lastUs = now;

    update(ax, ay, az, gx, gy, gz, dt);
}

/* ================================================================
 * 四元数 → 欧拉角 (rad)
 *   roll  = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1² + q2²))
 *   pitch = asin(2*(q0*q2 - q3*q1))
 *   yaw   = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2² + q3²))
 * ================================================================ */

float Mahony::roll() const
{
    return std::atan2(2.0f * (_q0 * _q1 + _q2 * _q3),
                      1.0f - 2.0f * (_q1 * _q1 + _q2 * _q2));
}

float Mahony::pitch() const
{
    float v = 2.0f * (_q0 * _q2 - _q3 * _q1);
    /* 限幅防止 asin 越界 */
    if (v > 1.0f)  v = 1.0f;
    if (v < -1.0f) v = -1.0f;
    return std::asin(v);
}

float Mahony::yaw() const
{
    return std::atan2(2.0f * (_q0 * _q3 + _q1 * _q2),
                      1.0f - 2.0f * (_q2 * _q2 + _q3 * _q3));
}
