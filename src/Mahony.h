#pragma once
#include <cstdint>
/* ================================================================
 * Mahony AHRS — 互补滤波器 (加速度计 + 陀螺仪 → 四元数)
 *
 * 参考: Mahony, Hamerton, et al. "Nonlinear Complementary Filters
 *        on the Special Orthogonal Group" (IEEE TAC, 2008)
 *
 * 原理:
 *   1. 加速度计测量重力方向 → 与当前姿态重力投影求叉积误差
 *   2. PI 控制器驱动误差 → 修正陀螺仪角速度
 *   3. 修正后的角速度积分 → 更新四元数
 *
 * 特点: 轻量级 (<2KB), 无矩阵运算, 适合实时嵌入式
 *
 * 用法:
 *   Mahony mh;
 *   mh.begin(200.0f, 0.5f, 0.0f);   // 200Hz, Kp=0.5, Ki=0
 *   loop {
 *       mh.update(ax, ay, az,        // 加速度 (任意单位, 自动归一化)
 *                 gx, gy, gz, dt);   // 角速度 (rad/s), 时间步长 (s)
 *       float r = mh.roll();         // 欧拉角 (rad)
 *       float p = mh.pitch();
 *       float y = mh.yaw();
 *   }
 * ================================================================ */

class Mahony {
public:
    Mahony();

    /* 初始化
     *   sampleFreq: 滤波器采样频率 (Hz), 仅用于设定 Ki 参考
     *   kp:         比例增益 (默认 0.5, 越大收敛越快但噪声越敏感)
     *   ki:         积分增益 (默认 0.0, 用于补偿陀螺仪零偏漂移)
     */
    void begin(float sampleFreq, float kp = 0.5f, float ki = 0.0f);

    /* 设置滤波器增益 (运行时调整) */
    void setGains(float kp, float ki);

    /* 单步更新
     *   ax, ay, az: 加速度计 (任意单位, 内部归一化)
     *   gx, gy, gz: 陀螺仪 (rad/s)
     *   dt:         距离上次更新的时间 (s)
     */
    void update(float ax, float ay, float az,
                float gx, float gy, float gz,
                float dt);

    /* 便捷接口: 自动使用内部计时 (调用前需先 begin) */
    void updateIMU(float ax, float ay, float az,
                   float gx, float gy, float gz);

    /* ── 四元数输出 ── */
    float q0() const { return _q0; }   /* W (标量) */
    float q1() const { return _q1; }   /* X */
    float q2() const { return _q2; }   /* Y */
    float q3() const { return _q3; }   /* Z */

    void getQuaternion(float &qw, float &qx, float &qy, float &qz) const
    { qw = _q0; qx = _q1; qy = _q2; qz = _q3; }

    /* ── 欧拉角输出 (rad) ── */
    float roll()  const;    /* 绕 X (横滚) */
    float pitch() const;    /* 绕 Y (俯仰) */
    float yaw()   const;    /* 绕 Z (航向, 无磁力计时仅陀螺仪积分) */

    /* ── 陀螺仪零偏估计 (rad/s) ── */
    float biasX() const { return _ibx; }
    float biasY() const { return _iby; }
    float biasZ() const { return _ibz; }

    /* 重置 */
    void reset();

private:
    float _q0, _q1, _q2, _q3;   /* 四元数 */

    float _kp, _ki;              /* PI 增益 */
    float _halfSampleFreq;       /* 0.5 * sampleFreq, 用于 Ki 缩放 */

    float _ibx, _iby, _ibz;      /* 陀螺仪零偏积分项 */

    /* 内部计时 (用于 updateIMU) */
    int64_t _lastUs;             /* 上次更新时间 (us) */
    bool    _timed;
};
