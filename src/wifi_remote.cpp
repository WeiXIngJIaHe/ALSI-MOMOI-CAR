#include "wifi_remote.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include <cstdlib>
#include <cstring>
#include <cctype>
#include <errno.h>

static const char *TAG = "WiFiRemote";

/* ================================================================ */

WiFiRemote::WiFiRemote(MotorDriver &motor)
    : _motor(motor), _server_fd(-1), _client_fd(-1),
      _connected(false), _pos(0), _speed(0), _turn(0)
{
    memset(_buf, 0, sizeof(_buf));
}

bool WiFiRemote::init(uint16_t port)
{
    /* 创建 TCP socket */
    _server_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (_server_fd < 0) {
        ESP_LOGE(TAG, "socket() 失败");
        return false;
    }

    /* 设置 SO_REUSEADDR (快速重启) */
    int opt = 1;
    setsockopt(_server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    /* 设置为非阻塞 */
    int flags = fcntl(_server_fd, F_GETFL, 0);
    fcntl(_server_fd, F_SETFL, flags | O_NONBLOCK);

    /* 绑定 */
    struct sockaddr_in addr = {};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(port);

    if (bind(_server_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind() 失败, 端口=%d", port);
        close(_server_fd); _server_fd = -1;
        return false;
    }

    /* 监听 */
    if (listen(_server_fd, 1) < 0) {
        ESP_LOGE(TAG, "listen() 失败");
        close(_server_fd); _server_fd = -1;
        return false;
    }

    ESP_LOGI(TAG, "TCP Server 就绪, 端口 %d — 用手机 TCP 客户端连接", port);
    return true;
}

/* ── 主循环 ── */

void WiFiRemote::process()
{
    /* 没有客户端 → 尝试接受 */
    if (_client_fd < 0) {
        _accept();
        return;
    }

    /* 有客户端 → 接收数据 */
    _recv();
}

/* ── 接受连接 ── */

void WiFiRemote::_accept()
{
    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);

    int fd = accept(_server_fd, (struct sockaddr *)&client_addr, &addr_len);
    if (fd < 0) {
        /* EAGAIN/EWOULDBLOCK = 暂无连接, 正常 */
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            ESP_LOGE(TAG, "accept() 错误: %d", errno);
        }
        return;
    }

    /* 新客户端: 设为非阻塞 */
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    _client_fd = fd;
    _connected = true;
    _pos = 0;
    ESP_LOGI(TAG, "客户端已连接");
}

/* ── 接收数据 ── */

void WiFiRemote::_recv()
{
    char tmp[64];
    int n = recv(_client_fd, tmp, sizeof(tmp) - 1, 0);

    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return;
        ESP_LOGW(TAG, "连接断开");
        _closeClient();
        return;
    }

    if (n == 0) {
        /* 客户端主动关闭 */
        ESP_LOGI(TAG, "客户端断开");
        _closeClient();
        _motor.allBrake();
        return;
    }

    /* 逐字节解析: 遇 '\n' 执行指令 */
    for (int i = 0; i < n; i++) {
        char c = tmp[i];
        if (c == '\n' || c == '\r') {
            if (_pos > 0) {
                _buf[_pos] = '\0';
                _parseCommand(_buf);
                _pos = 0;
            }
        } else if (_pos < (int)(sizeof(_buf) - 1)) {
            _buf[_pos++] = c;
        }
    }
}

void WiFiRemote::_closeClient()
{
    if (_client_fd >= 0) { close(_client_fd); _client_fd = -1; }
    _connected = false;
    _pos = 0;
}

/* ================================================================ */
/* 指令解析: F500 B200 L300 R400 S X400Y600                           */
/* ================================================================ */

void WiFiRemote::_parseCommand(const char *cmd)
{
    if (!cmd || !*cmd) return;

    char op = (char)toupper(cmd[0]);

    switch (op) {
    case 'F': _speed = atoi(cmd + 1); _turn = 0; break;
    case 'B': _speed = -atoi(cmd + 1); _turn = 0; break;
    case 'L': _turn =  atoi(cmd + 1); break;
    case 'R': _turn = -atoi(cmd + 1); break;
    case 'S': _speed = 0; _turn = 0; _motor.allBrake(); return;

    case 'X': {
        const char *y = strchr(cmd + 1, 'Y');
        int l = atoi(cmd + 1);
        int r = y ? atoi(y + 1) : l;
        _motor.setLeft(l); _motor.setRight(r);
        return;
    }

    default:
        ESP_LOGW(TAG, "未知指令: %s", cmd);
        return;
    }

    _motor.drive(_speed, _turn);
}
