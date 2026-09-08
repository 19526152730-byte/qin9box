# Face-Tracking Gimbal · 人脸跟踪视觉云台

基于 **STM32F407 + FreeRTOS + ESP32-S3** 的视觉伺服双轴云台系统：摄像头端完成人脸检测与单目视觉测距，主控端以多任务架构解析目标并驱动云台 PID 实时跟踪，状态经 MQTT 上报 OneNET 云平台。

## 系统架构

```
ESP32-S3 CAM（人脸检测 + 单目测距, 10~15Hz）
        │  USART3 115200   $x1,y1,x2,y2#
        ▼
STM32F407VGT6 · FreeRTOS（6 任务 + 消息队列）
        ├── gimbalTask    TIM4 PWM×2 ──► SG90 二自由度云台（PID 闭环, 50Hz）
        ├── distTask      8 点滑动平均滤波 + 链路超时判活
        ├── oledTask      I2C ──► SSD1306（目标有无 / 距离）
        ├── esp8266Task   USART6 AT ──► MQTT + JSON ──► OneNET
        └── ledTask       心跳巡检（任务级故障定位）
```

## 核心设计

| 模块 | 方案 |
|------|------|
| 单目测距 | 针孔模型 `dist = K / w`，人脸宽度先验，单次标定 K=68800，毫米级输出 |
| 串口链路 | 中断仅收字节入队，任务内状态机拼帧解析，检测数据流零丢失 |
| 云台控制 | PID + 死区防抖 + 输出限幅，失联 1s 缓慢回中，20ms 控制周期 |
| 可靠性 | 环形缓冲滑动平均滤波；心跳打卡 + LED 巡检机制 |

## 硬件清单

| 模块 | 型号 | 说明 |
|------|------|------|
| 主控 | 立创·天空星 STM32F407VGT6 | Cortex-M4 168MHz，FreeRTOS（CMSIS-RTOS v2） |
| 视觉 | 亚博 ESP32-S3 CAM Lite | OV2640 + ESP-WHO 人脸检测，独立供电 |
| 上云 | ESP8266-12F | AT 接入 WiFi，MQTT 上报 OneNET |
| 执行 | SG90 ×2 + 二自由度支架 | TIM4 双路 PWM 50Hz，5V 独立供电 |
| 显示 | 0.96" SSD1306 OLED | I2C 400kHz |
| 调试 | 板载 DAPLink / ST-Link V2 | SWD 烧录 + 虚拟串口 |

## 目录结构

```
├── rader_go/        # STM32 Keil 工程（应用层源码）
│   ├── Core/        # main / freertos.c —— 6 任务、帧解析、PID
│   ├── Bsp/         # SSD1306 OLED 驱动移植
│   └── MDK-ARM/     # Keil 工程文件（rader.uvprojx）
├── docs/            # 设计文档
└── rader.ioc        # CubeMX 配置（HAL / FreeRTOS 内核由其生成）
```

> HAL 与 FreeRTOS 内核为 CubeMX 生成文件，未纳入版本管理；克隆后用 `rader.ioc` 重新生成即可。

## 设计文档

- [系统架构与任务设计](docs/01-架构与任务设计.md)
- [串口帧协议与解析](docs/02-串口协议.md)
- [单目测距与标定](docs/03-单目测距标定.md)
- [硬件接线与供电](docs/04-硬件接线.md)
