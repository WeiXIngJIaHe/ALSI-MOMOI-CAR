# car_core

基于 ESP32-S3 的双核实时自平衡小车固件，C++ OOP 架构，运行在 ESP-IDF v5.5 框架上。

## 硬件平台

| 组件 | 型号 | 接口 | 引脚/地址 |
|------|------|------|-----------|
| 主控 | ESP32-S3-DevKitC1-N4R2 (Xtensa LX7, 240MHz) | — | — |
| IMU (6轴) | LSM6DSV16X | SPI2 | SCLK=9, MOSI=8, MISO=7, CS=10, INT1=6 |
| LED 驱动 | IS31FL3239 (36通道恒流) | I2C | 0x3C |
| IO 扩展 | TCA6408A (8位 GPIO) | I2C | 0x20 |
| 字符液晶 | LCD1602 + PCF8574 背板 | I2C | 0x27 |
| OLED 显示屏 | SSD1306 128×64 单色 | I2C | 0x3C |
| 电机驱动 ×2 | DRV8870 H桥 | GPIO + LEDC PWM | M1: IN1=21, IN2=22 (CH0); M2: IN1=23, IN2=24 (CH1) |
| 串口通信 | UART1 | UART | TX=43, RX=44 |

> **I2C 总线共享：** IS31FL3239、TCA6408A、LCD1602、SSD1306 共用 I2C_NUM_0，频率 400kHz。

## 系统架构

```
┌─────────────────────────────────────────────────────┐
│                    硬件初始化 (hw_init)                 │
│  NVS → I2C设备 → LED/按键/电机 → UART → WiFi → IMU   │
└────────────────────┬────────────────────────────────┘
                     │ INIT_BIT_HW_READY
          ┌──────────┴──────────┐
          ▼                     ▼
   ┌─────────────┐      ┌─────────────┐
   │  Core0 (100Hz)│      │   Core1 (控制) │
   │  传感器 + 融合  │      │  PID + IO 管理  │
   └──────┬──────┘      └──────▲──────┘
          │   FreeRTOS Queue     │
          └──────────────────────┘
```

**Core0（传感器，高优先级）：**
- SPI DMA 突发读取 IMU 6轴原始数据
- SFLP 芯片内置 6-DoF 四元数姿态解算（120Hz）
- Mahony 互补滤波软件融合
- 输出四元数 + 欧拉角（Roll/Pitch/Yaw）
- 每 500ms 输出诊断日志

**Core1（控制，次高优先级）：**
- 从队列接收姿态数据
- 双 PID 控制器（平衡 + 转向）→ 电机差速输出
- 按键扫描（4键，消抖，长按/双击检测）
- WiFi 远程控制 TCP Server（端口 8888）
- MQTT 消息轮询
- UART 二进制帧协议解析
- LED 灯效引擎驱动
- LCD1602 状态显示

## 功能模块

### 1. 姿态传感与融合 (`IMU.cpp` / `Mahony.cpp`)

- **LSM6DSV16X SPI 驱动：** ST MEMS SPI 协议（Mode 3），6MHz，DMA 14字节突发读取
- **SFLP 片上融合：** 6-DoF Game Rotation Vector 四元数，半精度浮点转 float32
- **Mahony 互补滤波：** PI 控制器消除陀螺零漂，融合加速度计重力参考
- **陀螺零偏校准：** 上电自动采集 200 样本（丢弃前 10 个）
- **寄存器诊断：** 初始化时全寄存器扫描打印
- **线程安全：** FreeRTOS 互斥锁 + DRAM 对齐 DMA 缓冲区

### 2. PID 控制 (`pid_controller.cpp`)

- 位置式 PID，双控制器（平衡 + 转向）
- 三重抗积分饱和：积分分离 + 微分先行 + 输出钳位
- 运行时可调增益和限幅
- **默认参数：**
  - 平衡 PID：Kp=0.8, Ki=0.05, Kd=0.03, 输出 [-1023, 1023]
  - 转向 PID：Kp=0.5, Ki=0.02, Kd=0.01, 输出 [-512, 512]

### 3. 电机驱动 (`DRV8870.cpp`)

- DRV8870 H桥，20kHz PWM（LEDC，10位精度，高于人耳听觉范围）
- 差速驱动模型：`left = speed + turn`, `right = speed - turn`
- 4种模式：正转 / 反转 / 刹车（短接制动）/ 惯性滑行
- PID 友好接口：`setSpeed(int16_t speed)` 映射 [-1023, +1023]
- ±20 死区自动刹车

### 4. LED 灯效系统 (`LED.cpp` / `IS31FL3239.cpp`)

- 12 颗 RGB LED（占用 36 通道），每颗亮度独立可调
- IS31FL3239 恒流驱动：双缓冲 PWM 寄存器，同步锁存更新
- **灯效引擎：** 呼吸（正弦）、跑马灯（追逐）、彩虹（HSV 扫描）、关闭
- 软件关断功能

### 5. 按键子系统 (`button.cpp` / `TCA6408.cpp`)

- TCA6408A IO 扩展器：8位 I2C GPIO，P0-P3 接 4 个按键（低电平有效）
- **事件检测：**
  - 短按 / 释放（消抖 50ms，扫描周期 10ms）
  - 长按（持续 1000ms）
  - 双击（400ms 窗口内两次按下）
- 非阻塞事件轮询接口 `getEvent()`

### 6. 显示系统 (`LCD.cpp`)

- **LCD1602（当前使用）：** HD44780 + PCF8574 I2C 背板，16×2 字符，背光控制，光标闪烁控制
- **SSD1306 OLED（驱动已实现，待启用）：** 128×64 单色，帧缓冲 1024 字节，支持点/线/矩形/圆/位图绘制，6×8 ASCII 字库

### 7. UART 通信协议 (`urat.cpp`)

- UART1 115200bps，RX 缓冲 2048 字节，TX 缓冲 512 字节
- **二进制帧格式：** `[STX(0xAA)] [CMD(1B)] [LEN(1B)] [DATA(N)] [XOR(1B)] [ETX(0x55)]`
- **支持指令：**

| 指令 | 功能 |
|------|------|
| `0x01` | 查询 IMU 数据（返回 10 个 float：加速度/角速度/欧拉角/温度） |
| `0x10` | 心跳（回显） |
| `0xFF` | 系统重启（`esp_restart()`） |

### 8. WiFi 管理 (`wifi.cpp`)

- ESP-IDF v5.5 标准 WiFi STA 模式
- 自动重连（最多 5 次）
- WPA2 最低安全要求
- **射频诊断：** 主动 AP 扫描 + RSSI 信号质量评估（优秀/良好/一般/差）
- 事件驱动的连接状态管理

### 9. MQTT 遥测 (`mqtt.cpp`)

- ESP-IDF `esp_mqtt` 客户端
- 订阅 `/car/cmd` 主题
- 消息通过 FreeRTOS 队列（容量 16）投递到 Core1 处理
- 非阻塞消息消费

### 10. WiFi 远程控制 (`wifi_remote.cpp`)

- lwIP TCP Server，监听端口 8888
- 单客户端连接（断线自动接受新连接）
- 文本指令协议：

| 指令 | 功能 |
|------|------|
| `Fxxx` | 前进，速度 xxx |
| `Bxxx` | 后退，速度 xxx |
| `Lxxx` | 左转 |
| `Rxxx` | 右转 |
| `S` | 紧急刹车 |
| `XxxxYyyy` | 直接控制左右电机速度 |

## 开发环境

| 工具 | 版本/配置 |
|------|----------|
| 构建系统 | PlatformIO |
| 框架 | ESP-IDF v5.5.2 |
| 编译器 | xtensa-esp32s3-elf-gcc (C17 / C++23) |
| IDE | VS Code + PlatformIO 扩展 |
| 调试器 | USB Serial/JTAG (UART0) |

### 快速开始

```bash
# 克隆项目
git clone <repo_url>
cd car_core

# 安装依赖（ESP-IDF 工具链 + PlatformIO）
# 参考：https://docs.platformio.org/en/latest/core/installation.html

# 编译
pio run

# 烧录 + 串口监视
pio run -t upload -t monitor
```

### 项目配置

核心配置位于 [platformio.ini](platformio.ini)：

- 串口：`upload_port = COM3`, `monitor_speed = 115200`
- 日志级别：`CORE_DEBUG_LEVEL=4`（详细）
- 优化级别：`-O2`
- 并行编译任务数：4

## 数据流

```
Core0 (100Hz 定时循环)
  │
  ├── IMU::readAll()          SPI DMA 读取加速度 + 角速度
  ├── IMU::sflpRead()          SFLP 片上四元数
  ├── Mahony::update()         软件融合 → Quaternion + Euler
  │
  └── Queue ──────────────────►  Core1 (事件驱动循环)
                                    │
                                    ├── PID::compute()      平衡 + 转向
                                    ├── MotorDriver::drive()  差速 PWM 输出
                                    ├── Button::getEvent()  按键事件处理
                                    ├── UART::receive()     协议解析 + 应答
                                    ├── wifi_remote::loop() TCP 遥控指令
                                    ├── mqtt::receive()     MQTT 消息
                                    └── LED::handle()       灯效刷新
```

## 项目结构

```
car_core/
├── platformio.ini              # PlatformIO 构建配置
├── CMakeLists.txt              # CMake 包装（IDE集成用）
├── sdkconfig.defaults          # ESP-IDF SDK 默认配置
├── src/
│   ├── main.cpp                # 入口点 + 双核任务架构
│   ├── IMU.h / IMU.cpp         # LSM6DSV16X SPI 驱动 + SFLP
│   ├── Mahony.h / Mahony.cpp   # Mahony AHRS 互补滤波
│   ├── pid_controller.h/.cpp   # 位置式 PID（抗积分饱和）
│   ├── DRV8870.h / DRV8870.cpp # 双路 H 桥电机驱动
│   ├── IS31FL3239.h/.cpp       # 36通道 I2C LED 驱动 IC
│   ├── LED.h / LED.cpp         # RGB LED 灯效引擎
│   ├── TCA6408.h / TCA6408.cpp # 8位 I2C IO 扩展器
│   ├── button.h / button.cpp   # 按键事件检测（消抖/长按/双击）
│   ├── LCD.h / LCD.cpp         # LCD1602 + SSD1306 OLED 驱动
│   ├── URAT.h / urat.cpp       # UART 二进制帧协议
│   ├── wifi.h / wifi.cpp       # WiFi STA 管理 + 射频诊断
│   ├── mqtt.h / mqtt.cpp       # MQTT 客户端
│   └── wifi_remote.h/.cpp      # TCP Server 远程控制
├── include/                    # 公共头文件目录（预留）
├── lib/                        # 私有库目录（预留）
└── test/                       # 单元测试目录（预留）
```

## 许可

MIT License
