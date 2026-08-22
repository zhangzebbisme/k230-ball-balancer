# k230-ball-balancer

<img src="https://raw.githubusercontent.com/zhangzebbisme/k230-ball-balancer/main/images/finished.jpg" width="360" alt="成品实物">

<img src="https://raw.githubusercontent.com/zhangzebbisme/k230-ball-balancer/main/images/finished2.jpg" width="360" alt="调试截图">

K230 (CanMV) 钢珠平衡控制系统 —— OpenCV 视觉识别（≥60FPS）+ PID 控制 + Wi-Fi 网页图传

## 功能

1. OpenCV 识别钢珠（≥60FPS），PID 控制自动稳定，并响应串口多任务指令（任务 3/4/5/6）
2. 开启 Wi-Fi AP 热点（SSID: `GK-TFBOSS-ds`）
3. 浏览器访问 `http://192.168.169.1:8081` 查看实时灰度局部视频流，支持网页录像

## 控制原理

- 视觉识别：OpenCV 灰度阈值+轮廓检测提取钢珠，识别帧率 ≥60FPS，尺寸/像素/宽高比过滤反光与边缘误识别
- 控制算法：PID 位置控制，ROI 区域（800x480 画面中 40px 高凹槽带）
- 图传优化：仅截取中间 800x128 凹槽区域编码 MJPEG，节约带宽

## 硬件接线

### K230（天猛星 CanMV）与张大头闭环步进电机驱动

| K230 引脚 | 连接 | 张大头闭环驱动 |
|---|---|---|
| 5V（下载口背面） | ← | 电源 5V |
| GND（下载口背面） | ← | 电源 GND |
| TX1 (IO3) | → | R / A / H（电机指令串口） |
| RX1 (IO4) | → | B2（备用/反馈） |
| — | — | T / B / L 不接 |

### 张大头闭环驱动供电（轮趣双路驱动）

| 张大头闭环驱动 | 轮趣双路驱动 |
|---|---|
| V+ | → VOUT+ |
| G | → VOUT- |

说明：K230 通过 UART1（115200 波特率）向张大头闭环驱动发送绝对位置指令帧，驱动步进电机转动丝杆，改变管道倾角实现钢珠平衡。

## 程序详解

单文件 `k230_ball_balancer.py`（约 990 行），**双线程架构**：后台线程做视觉检测 + PID 控制，主线程跑 Wi-Fi AP 和 MJPEG 网页图传。

### 代码结构

| 区块 | 内容 |
|---|---|
| §1 图传与网络参数 | AP_SSID、HTTP_PORT=8081、JPEG_QUALITY=35、TARGET_FPS=15 |
| §2 视觉检测与流媒体区域 | 灰度阈值 (71,91)、ROI 40px 凹槽带、图传 800x128 裁剪、小球几何过滤 |
| §3 PD 控制与目标参数 | CENTER_X=378、KP=0.5、KD=60、KI=0.6、死区 30px、控制周期 50ms |
| §4 运行状态定义 | 任务 3/4/5/6 有限状态机的全部状态 |
| §5 全局共享变量 | 线程间共享的最新图像与控制状态 |
| §6 电机与任务控制函数 | 张大头协议帧、PID（balanceX）、任务切换 |
| §7 核心控制子线程 | `control_thread`：抓帧→找球→PID→发电机 主循环 |
| §8 Web 服务与图传 | Wi-Fi AP、MJPEG 流、网页控制 |
| §9 系统入口 | 硬件初始化 → 开 AP → 启线程 → HTTP 服务器 |

### 双线程架构

```
主线程:  main() → 传感器/显示初始化 → Wi-Fi AP → run_http_server()
              └── MJPEG 图传 + 网页 (800x128 灰度流, 15FPS)

子线程:  control_thread() 每 50ms:
              sensor 抓帧 → 灰度阈值+轮廓检测 → 几何过滤找球
              → balanceX() 算 PID → control_motor_absolute_position()
              → UART1 发张大头指令帧
```

### 张大头电机协议帧（UART1, 115200）

绝对位置指令（13 字节）：

```
0x04 0xFD dir sH sL 0x0F p3 p2 p1 p0 0x01 0x00 0x6B
      │    │   │    │    │    └─ 绝对位置 32bit（大端）
      │    │   │    └────── 速度 16bit（RPM）
      │    └─────── 方向: 0x00 正向 / 0x01 反向
      └──────────── 功能码
```

停止指令（5 字节）：

```
0x04 0xFE 0x98 0x00 0x6B
```

### 上位机串口协议

帧格式：`0xFF 任务ID 0x0D`（K230 从 UART1 接收）

| 任务ID | 功能 |
|---|---|
| 0x03 | 任务3：回中心 → +5cm → -5cm → 保持 |

| 0x04 | 任务4：发射补偿启动 |
| 0x14 | 任务4：停止补偿 |

| 0x05 | 任务5：减速补偿启动 |
| 0x15 | 任务5：减速补偿 |
| 0x18 | 任务4/5：停止补偿 |


| 0x06 | 任务6：捕获位置保持 |
| 0x16 | 任务6：预备 |
| 0x17 | 任务6：取消预备 |

### PID 控制（balanceX）

- 偏差 `bias = 球心X - 目标X`
- P：`KP * bias`（KP=0.5）
- D：`KD * (bias - 上次bias)`（KD=60），目标切换时重置避免微分冲击
- I：低频积分，每 10 个周期更新一次，限幅 ±500（KI=0.6）
- 输出限幅 ±220，映射为电机绝对位置 `MOTOR_MID_POSITION + output`

### 状态机（任务 3/4/5/6）

任务3：`TO_CENTER → TO_POSITIVE(+5cm) → TO_NEGATIVE(-5cm) → HOLD_NEGATIVE`先到+5cm再到-5cm然后保持
任务4：`LAUNCH_COMPENSATION → COMPENSATION_RELEASE → STOP_COMPENSATION`保持中心
任务5：`DECEL_COMPENSATION`保持中心
任务6：`CAPTURE_POSITION → HOLD_POSITION`随机一个位置保持

## 运行环境

-  CanMV K230嘉立创庐山派1G开发板
- 张大头闭环步进电机驱动 

## 参数速查

| 参数 | 值 |
|---|---|
| AP SSID | GK-TFBOSS-ds |
| 视频地址 | 192.168.169.1:8081 |
| JPEG 质量 | 35 |
| 目标帧率 | 15 FPS |
| 识别帧率 | ≥60 FPS |
| UART | UART1, 115200, TX=IO3, RX=IO4 |
| 控制周期 | 50 ms |

## 文件结构

- `k230_ball_balancer.py` — 主程序（单文件，直接运行）
