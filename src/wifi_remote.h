#pragma once
#include "DRV8870.h"

/* ================================================================
 * WiFi 遥控 — TCP Server
 *
 * ESP32 启动 TCP Server (端口 8888), 等待手机/PC 连接
 * 手机通过 TCP 发送文本指令控制电机:
 *
 *   Fxxx → 前进  Bxxx → 后退  Lxxx → 左转  Rxxx → 右转  S → 刹车
 *
 * 用法:
 *   WiFiRemote remote(g_motor);
 *   remote.init();                // 启动 TCP server
 *   remote.process();             // 在 Core1 循环中调用
 * ================================================================ */

#define WIFI_REMOTE_PORT            8888


class WiFiRemote {
public:
    WiFiRemote(MotorDriver &motor);

    bool init(uint16_t port = WIFI_REMOTE_PORT);
    bool isConnected() const { return _connected; }

    /* 主循环调用: 接收 + 解析 + 执行指令 */
    void process();

private:
    MotorDriver &_motor;

    int  _server_fd;      /* 监听 socket */
    int  _client_fd;      /* 客户端 socket (-1=无连接) */
    bool _connected;

    char _buf[64];
    int  _pos;
    int  _speed, _turn;

    void _accept();
    void _recv();
    void _parseCommand(const char *cmd);
    void _closeClient();
};
