# M2 System — 农业机器人 LAN 控制系统

通过局域网（LAN）对 farm-ng Amiga 农业机器人进行远程键盘控制、视频流传输、**手机网页摇杆控制**，以及**自主 GPS 航点导航**。

---

## 系统架构

```
远程 PC (01_remote_side/)
├── 主线程：cv2.imshow 视频显示
├── 线程 A：VideoCapture 从 HTTP:8080 拉帧
├── 线程 B：pynput 键盘监听
└── 线程 C：TCP:9000 命令发送 + 心跳
        │
        │  TCP :9000（控制）   HTTP :8080（视频）
        ▼
  机器人端 (00_robot_side/)
  ├── robot_receiver.py   → 串口 → Feather M4 CAN → CAN 总线 → Amiga Dashboard
  └── camera/camera_streamer.py  ← FrameSource（可插拔 Pipeline）

手机浏览器（同 LAN）
└── HTTP :8888 → index.html（nipplejs 摇杆 + IMU HUD + RTK 面板 + NAV 面板）
        │
        │  WebSocket :8889
        ▼
  机器人端 (00_robot_side/)
  └── web_controller.py → 串口 → Feather M4 CAN → CAN 总线 → Amiga Dashboard
                        ← IMU  OAK-D BNO085（20 Hz 广播）
                        ← RTK  Emlid RS+（1 Hz 广播）
                        ← ODOM Amiga TPDO1（实测速度/角速度/控制状态/电量，20 Hz 广播）
                        → CSV  data_log/（浏览器手动 REC/STOP）

QGIS（CSV 航点）或 CoveragePlanner（自动生成覆盖路径）
        │ 通过浏览器上传  或  通过浏览器 COV 面板生成
        ▼
    NavigationEngine（位于 web_controller.py 内）
    ├── CoveragePlanner   — Boustrophedon 蛇形覆盖路径生成器
    ├── WaypointManager   — CSV 航点序列 + 自适应到达容忍半径
    ├── GPS 滤波器        — MovingAverageFilter 或 KalmanFilter（IMU + 里程计辅助）
    ├── 控制器            — P2PController 或 PurePursuitController
    └── 20 Hz 控制循环 → V 命令 → 串口 → Feather M4 CAN
                         ← O: 里程计回报（meas_speed, meas_ang_rate, state, SOC，约 20 Hz）
```

---

## 目录结构

```
m2_system/
├── 00_robot_side/                  # 机器人端（Mac Mini / Linux）
│   ├── config.py                   # 所有参数（串口/TCP/相机/Web/导航），支持环境变量覆盖
│   ├── core/                       # 基础设施包
│   │   ├── serial_writer.py        # 线程安全串口封装，命令白名单过滤
│   │   └── watchdog.py             # 看门狗定时器，超时自动急停
│   ├── sensors/                    # 传感器层
│   │   ├── imu_reader.py           # IMUReader 守护线程 + quaternion_to_compass（OAK-D）
│   │   ├── esp32_imu_reader.py     # ESP32IMUReader — BNO085 串口 "roll,pitch,yaw,acc\n"
│   │   └── rtk_reader.py           # RTKReader 守护线程 — NMEA GGA/RMC 解析（Emlid RS+）
│   ├── navigation/                 # 导航算法层
│   │   ├── geo_utils.py            # 纯函数：Haversine、方位角、normalize_angle、投影
│   │   ├── waypoint.py             # Waypoint 数据类 + WaypointManager（自适应容忍半径）
│   │   ├── gps_filter.py           # MovingAverageFilter + KalmanFilter（4D，IMU+里程计辅助）
│   │   ├── controller.py           # PIDController、P2PController、PurePursuitController
│   │   ├── nav_engine.py           # NavigationEngine 状态机（NavState/NavMode/FilterMode）
│   │   ├── coverage_planner.py     # CoveragePlanner — Boustrophedon 蛇形覆盖路径生成器
│   │   └── field_boundary.py       # 边界提取工具：CSV 轨迹凸包、手动输入
│   ├── camera/                     # 视频层
│   │   ├── frame_source.py         # FrameSource ABC + SimpleColorSource（OAK-D）
│   │   └── camera_streamer.py      # MJPEGServer：将 FrameSource 推流为 HTTP MJPEG
│   ├── robot_receiver.py           # TCP 服务端 + 看门狗 + 串口转发
│   ├── local_controller.py         # 本地键盘直连串口（无需 TCP）
│   ├── web_controller.py           # Web 摇杆 + 自主导航 + 覆盖规划：HTTP :8888 + WS :8889
│   ├── data_recorder.py            # DataRecorder — IMU+RTK+指令 CSV 写入
│   ├── web_static/
│   │   ├── index.html              # 单页 HUD（摇杆 + 速度滑块 + 罗盘 + IMU + RTK + NAV）
│   │   └── nipplejs.min.js         # nipplejs 本地副本（LAN 无需 CDN）
│   ├── main.py                     # 交互式启动菜单（推荐入口）
│   ├── log/                        # 运行日志（自动创建）
│   ├── data_log/                   # CSV 录制文件（首次点击 REC 自动创建）
│   └── cam_demo/                   # OAK-D 相机示例脚本
├── 01_remote_side/                 # 远程端（开发 PC / 操作站）
│   ├── config.py                   # ROBOT_HOST、TCP/STREAM 端口、重连延迟
│   ├── remote_sender.py            # pynput 键盘 + TCP 客户端 + 心跳（可独立运行）
│   ├── remote_viewer.py            # MJPEG 拉流 + cv2.imshow + 断流重连（可独立运行）
│   ├── main.py                     # 一键启动：sender（后台线程）+ viewer（主线程）
│   └── log/
├── CIRCUITPY/                      # Feather M4 CAN 固件（CircuitPython）
│   ├── code.py                     # 解析串口（WASD + V 速度命令）；输出 O: 里程计 + S: 状态
│   └── lib/farm_ng/                # farm-ng Amiga 协议库
├── CLAUDE.md
├── README.md                       # 英文文档
└── README_zh.md                    # 本文件（中文）
```

---

## 操作模式

### 模式 A：本地直连控制

适用于现场操控，操作员在机器人所在的电脑上操作。

```
本地 PC (pynput) ──► local_controller.py ──串口──► Feather M4 CAN
```

- 无需 TCP，直接持有串口；无看门狗（操作员在现场）
- **注意：不可与 `robot_receiver.py` 或 `web_controller.py` 同时运行（串口冲突）**

### 模式 B：远程 TCP 控制

适用于远程操控，操作员在另一台电脑上控制机器人。

```
远程 PC (pynput) ──TCP:9000──► robot_receiver.py ──串口──► Feather M4 CAN
```

- 远程端发送键盘命令 + 心跳包；机器人端看门狗 2 秒无心跳自动急停

### 模式 C：远程 TCP 控制 + 视频流（推荐）

同时提供键盘控制与实时视频，操作员在远程 PC 上一键启动。

```
远程 PC ──TCP:9000──► robot_receiver.py ──串口──► Feather M4 CAN
        ◄─HTTP:8080── camera/camera_streamer.py ◄── FrameSource（OAK-D / YOLO / ...）
```

### 模式 D：Web 摇杆控制（手机/平板友好）

用同局域网内的任意手机或平板浏览器控制机器人。
支持**比例控制**（对角线运动）、实时 IMU / 罗盘 HUD、RTK GPS 面板，以及手动 CSV 数据录制。

```
手机浏览器 ──HTTP:8888──► web_static/index.html（摇杆 + 速度滑块 + IMU + RTK + NAV 面板）
          ──WS:8889────► web_controller.py ──串口──► Feather M4 CAN
          ◄─WS:8889───── web_controller.py ◄── OAK-D BNO085 IMU（20 Hz）
                                           ◄── Emlid RS+ RTK GPS（1 Hz）
                                           ──► data_log/*.csv（录制时）
```

与模式 B 的主要区别：
- **比例控制**：摇杆直接映射到绝对速度，不再是增量步进
- **对角线运动**：线速度和角速度同时设定，一条命令完成
- **速度比例滑块**：拖动 **SPEED** 滑块（10%–100%，默认 50%）可实时缩放摇杆输出——满速过于灵敏时可降低比例以精准操控，纯前端缩放，服务端速度钳位依然生效
- **IMU HUD**：线加速度（已去除重力分量）、陀螺仪、磁力罗盘实时显示于浏览器
- **RTK GPS 面板**：实时显示经纬度、高度、定位质量徽章（NO FIX / GPS / DGPS / RTK FIXED / RTK FLOAT）、卫星数、HDOP、速度
- **CSV 录制**：点击 **● REC** 开始记录，点击 **■ STOP** 关闭文件，每次生成一个带时间戳的文件保存在 `data_log/`
- 无需安装任何 App，现代手机浏览器直接访问

### 模式 E：自主 GPS 航点导航

在 QGIS 中绘制路径并导出为 CSV，在浏览器上传后按 **▶ AUTO**。
机器人利用 RTK GPS + IMU 融合自主依次到达各航点。

```
QGIS → 导出 CSV 航点
          │ 通过浏览器上传（WebSocket）
          ▼
web_controller.py
    └── NavigationEngine
         ├── WaypointManager   — 自适应到达容忍半径，连续帧确认到达
         ├── GPS 滤波器        — MovingAverageFilter（10 帧滑动窗口）或
         │                       KalmanFilter（4D 状态：位置+速度，20 Hz IMU 辅助）
         ├── P2PController     — 点对点直接导航，PID 朝向控制 + 减速坡
         └── PurePursuitController — 前视点路径跟踪（曲线平滑）
              │
              └── V 命令（20 Hz）→ 串口 → Feather M4 CAN → Amiga Dashboard
```

自主导航期间摇杆自动禁用。浏览器实时显示航点进度、距目标距离、方位角和定位质量。
罗盘 HUD 在任意校准级别（UNCAL / LOW / GOOD / PRECISE）下均实时显示方位角，精度标签同步更新。

> **罗盘 UNCAL 解决方案**：若 BNO085 磁力计无法完成画8字校准（摄像头固定在机器人上时），设置 `COMPASS_MIN_ACCURACY=0` 可接受未校准的方位角数据。如果方位角存在已知固定偏差，可用 `COMPASS_OFFSET_DEG` 进行静态补偿。

> **导航诊断日志**：导航过程中，终端每 5 秒输出一行 `[NAV STATUS]`，显示当前航点、距离、方位误差和发出的速度命令。若机器人静止不动，每 5 秒会输出 `WARNING` 说明原因（罗盘精度不足、GPS 滤波器预热中等）。

### 模式 F：覆盖路径规划（Boustrophedon 蛇形）

自动生成"割草机"式往返覆盖路径，无需在 QGIS 中手动放置航点。

```
浏览器 COV 面板
  ├── boundary: [[lat,lon], …]  （输入田地角点坐标，或从录制 CSV 提取凸包）
  ├── row_spacing: 1.0 m        （行间距）
  ├── direction_deg: 0          （0 = N-S 行，90 = E-W 行）
  └── overlap: 0 %              （行重叠率）
          │ WebSocket: generate_coverage
          ▼
    CoveragePlanner（Boustrophedon 算法）
      1. 将 GPS 边界投影到本地 ENU 平面（以重心为原点）
      2. 旋转到扫描坐标系（x' = 行进方向，y' = 行间距方向）
      3. 扫描线 y' = const 与多边形求交
      4. S 形连接各行端点（逐行翻转方向）
      5. 反投影回 WGS-84 经纬度
          │ 自动加载到 NavigationEngine
          ▼
    WaypointManager → 20 Hz 控制循环 → 串口 → Feather M4 CAN
```

**使用流程：**

1. 点击底栏 **COV** 按钮，打开覆盖路径规划对话框
2. 逐行输入田地边界坐标（格式：`lat,lon`，至少 3 个顶点）
3. 设置行间距、行进方向和重叠率
4. 点击 **⚡ GENERATE PATH** — 航点自动加载到导航引擎
5. 确认航点数量后，点击 **▶ AUTO** 开始执行

> 也可在代码中使用 `field_boundary.py` 从录制的 `data_log/*.csv` 提取凸包，再传给 `CoveragePlanner` 生成路径。

---

## 串口协议

### WASD（原有，单字节增量）

| 字节    | 动作                                       |
|---------|--------------------------------------------|
| `w`     | `cmd_speed += 0.1`                         |
| `s`     | `cmd_speed -= 0.1`                         |
| `a`     | `cmd_ang_rate += 0.1`                      |
| `d`     | `cmd_ang_rate -= 0.1`                      |
| `空格`  | 急停（`cmd_speed = cmd_ang_rate = 0`）     |
| `\r`    | 切换 AUTO_READY ↔ AUTO_ACTIVE；固件回报 `S:ACTIVE\n` 或 `S:READY\n` |

### 固件状态回报（上位机 ← 固件）

收到 `\r` 后，Feather M4 回复以下之一：

```
S:ACTIVE\n   — request_state 已设为 AUTO_ACTIVE
S:READY\n    — request_state 已设为 AUTO_READY
```

固件启动时也会主动发送一次 `S:READY\n`，供上位机同步初始状态。
`web_controller.py` 通过独立的 `SerialReader` 守护线程解析这些回报行，
并向所有已连接的浏览器客户端广播 `state_status` WebSocket 消息。

### O 命令 — 里程计回报（固件 → 上位机，约 20 Hz）

Feather M4 在每次收到 Amiga TPDO1 帧时广播实测轮速、控制状态和电池电量：

```
格式：  "O:{meas_speed:.3f},{meas_ang_rate:.3f},{state_int},{soc}\n"
示例：  "O:0.253,-0.012,3,85\n"   →  线速 0.253 m/s，角速 -0.012 rad/s，AUTO_ACTIVE(3)，电量 85%
        "O:0.000,0.000,1,72\n"    →  静止，AUTO_READY(1)，电量 72%
```

`web_controller.py` 在 SerialReader 线程中解析 `O:` 行（向后兼容：2 字段旧格式也可正常接收）。
解析结果存入 `_last_odom` 后：
- 调用 `NavigationEngine.on_odometry(v, w)` — 启用 **Kalman** 滤波器时，实测速度旋转到 NED 坐标系作为速度观测，抑制两次 GPS 更新之间的漂移。
- 以 20 Hz 广播 `{"type": "odom", v, w, state, soc, ts}` 到所有浏览器客户端，更新 HUD 的 **ODOM** 面板（SPD / ROT / BAT）。
- 向每行 CSV 记录中写入 `odom_speed`、`odom_angrate`、`amiga_soc` 三列。

**AmigaControlState 枚举值**（来自 `CIRCUITPY/packet.py`）：

| 整数值 | 状态名                  | 含义                     |
|--------|-------------------------|--------------------------|
| 0      | `STATE_STOPPED`         | 固件已停止               |
| 1      | `STATE_AUTO_READY`      | 待激活                   |
| 2      | `STATE_AUTO_ACTIVE`     | 正在执行指令             |
| 3–7    | 其他状态                | 错误 / 过渡状态          |

### V 命令（新增，绝对速度）

```
格式：  "V{speed:.2f},{angular:.2f}\n"
示例：  "V0.50,-0.30\n"   →  前进 0.5 m/s，右转 0.3 rad/s
        "V0.00,0.00\n"    →  急停
        "V-0.30,0.20\n"   →  后退 + 左转（对角线运动）
```

速度值在固件侧钳位到 `[-1.0, 1.0]`。两种协议同时有效。

---

## WebSocket 协议（浏览器 ↔ web_controller.py）

### 客户端 → 服务端

| `type`              | 载荷                                                | 说明                           |
|---------------------|-----------------------------------------------------|--------------------------------|
| `heartbeat`         | —                                                   | 维持看门狗心跳                 |
| `joystick`          | `{linear, angular, force}`                          | 比例速度指令                   |
| `toggle_state`      | —                                                   | 切换 AUTO_READY ↔ AUTO_ACTIVE  |
| `toggle_record`     | —                                                   | 开始 / 停止 CSV 录制           |
| `upload_waypoints`  | `{csv: "id,lat,lon,tolerance_m,max_speed\n…"}`     | 上传 QGIS 航点 CSV             |
| `nav_start`         | —                                                   | 开始自主导航（需 GPS fix ≥ 1） |
| `nav_start_force`   | —                                                   | 强制启动导航（跳过 GPS fix 检查）|
| `nav_stop`          | —                                                   | 停止自主导航                   |
| `nav_mode`          | `{mode: "p2p" \| "pure_pursuit"}`                  | 切换导航算法                   |
| `filter_mode`       | `{mode: "moving_avg" \| "kalman"}`                 | 切换 GPS 滤波器                |
| `generate_coverage` | `{boundary:[[lat,lon],…], row_spacing, direction_deg, overlap, tolerance_m, max_speed}` | 生成并加载 Boustrophedon 覆盖路径 |

### 服务端 → 客户端

| `type`             | 关键字段                                                                        | 说明                     |
|--------------------|---------------------------------------------------------------------------------|--------------------------|
| `imu`              | `accel, gyro, compass`                                                          | 20 Hz IMU 广播           |
| `rtk`              | `lat, lon, alt, fix_quality, num_sats, hdop`                                   | 1 Hz RTK GPS 广播        |
| `odom`             | `{v, w, state, soc, ts}`                                                        | 20 Hz 里程计广播：实测线速度 (m/s)、角速度 (rad/s)、AmigaControlState 枚举值、电池电量 (%) |
| `state_status`     | `{active: bool}`                                                                | 固件 AUTO 状态变更       |
| `record_status`    | `{recording, filename}`                                                         | CSV 录制状态变更         |
| `status`           | `{serial_ok, imu_ok, rtk_ok, recording}`                                       | 2 Hz 系统健康状态        |
| `waypoints_loaded` | `{count: N}`                                                                    | CSV 解析结果             |
| `nav_status`       | `{state, progress:[i,n], distance_m, target_bearing, nav_mode, filter_mode, tolerance_m}` | ~4 Hz 导航状态  |
| `nav_complete`     | `{total_wp: N}`                                                                 | 全部航点到达             |
| `nav_warning`      | `{msg: "GPS timeout"}`                                                          | GPS 丢失，导航暂停       |
| `coverage_ready`   | `{count: N}` 或 `{count:0, error:"…"}`                                         | 覆盖路径已生成并加载     |

---

## 键位说明（键盘模式）

| 按键    | 功能                                    |
|---------|-----------------------------------------|
| `W`     | 前进（速度 +0.1 m/s）                   |
| `S`     | 后退（速度 −0.1 m/s）                   |
| `A`     | 左转（角速度 +0.1 rad/s）               |
| `D`     | 右转（角速度 −0.1 rad/s）               |
| `空格`  | 紧急停止（松开所有按键后 ≤100 ms 内自动触发）    |
| `Enter` | 切换状态：AUTO_READY ↔ AUTO_ACTIVE      |
| `Q`     | 退出程序                                |

> 方向键持续发送（10 Hz），松开所有方向键后 repeat loop 在 ≤100 ms 内自动发送急停（Linux evdev 模式下可避免长按时产生伪 release 事件导致卡顿）。

---

## 安装依赖

```bash
# 机器人端（Mac Mini / Linux）
pip install pyserial depthai opencv-python websockets numpy

# 远程端
pip install pynput opencv-python
```

---

## 配置参数

### 机器人端（`00_robot_side/config.py`）

| 参数                  | 默认值（macOS）            | 默认值（Linux）    | 说明                         |
|-----------------------|----------------------------|--------------------|------------------------------|
| `FEATHER_PORT`        | `/dev/cu.usbmodem2301`     | `/dev/ttyACM0`     | Feather M4 CAN 串口路径      |
| `SERIAL_BAUD`         | `115200`                   | 同左               | 串口波特率                   |
| `TCP_PORT`            | `9000`                     | 同左               | TCP 监听端口                 |
| `WATCHDOG_TIMEOUT`    | `2.0` 秒                   | 同左               | 看门狗超时时间               |
| `KEY_REPEAT_INTERVAL` | `0.1` 秒（10 Hz）          | 同左               | 按键重复发送间隔             |
| `CAM1_IP`             | `10.95.76.10`              | 同左               | OAK-D PoE 相机 1 IP          |
| `CAM2_IP`             | `10.95.76.11`              | 同左               | OAK-D PoE 相机 2 IP          |
| `CAM1_STREAM_PORT`    | `8080`                     | 同左               | 相机 1 MJPEG 流端口          |
| `CAM2_STREAM_PORT`    | `8081`                     | 同左               | 相机 2 MJPEG 流端口          |
| `FRONT_CAM_IP`        | （= `CAM1_IP`）            | 同左               | 前置摄像头 IP；`CAM_SELECTION=both` 时始终分配到 8080 端口 |
| `MJPEG_QUALITY`       | `80`                       | 同左               | JPEG 编码质量（1–100）       |
| `LOCAL_DISPLAY`       | `0`（关）                  | 同左               | `1` 开启机器人端本地预览     |
| `WEB_HTTP_PORT`       | `8888`                     | 同左               | Web 摇杆 HTTP 端口           |
| `WEB_WS_PORT`         | `8889`                     | 同左               | Web 摇杆 WebSocket 端口      |
| `MAX_LINEAR_VEL`      | `1.0` m/s                  | 同左               | 最大线速度                   |
| `MAX_ANGULAR_VEL`     | `1.0` rad/s                | 同左               | 最大角速度                   |
| `COORD_SYSTEM`        | `NED`                      | 同左               | IMU 坐标系：`NED`（x=北）或 `ENU`（x=东） |
| `IMU_SOURCE`          | `esp32`                    | 同左               | IMU 后端：`esp32`（BNO085 串口）或 `oakd`（OAK-D depthai） |
| `ESP32_IMU_PORT`      | `/dev/ttyUSB0`             | 同左               | ESP32 串口路径（`IMU_SOURCE=esp32` 时有效） |
| `ESP32_IMU_BAUD`      | `115200`                   | 同左               | ESP32 串口波特率             |
| `RTK_PORT`            | `/dev/cu.usbmodem2403`     | 同左               | Emlid RS+ 串口路径           |
| `RTK_BAUD`            | `9600`                     | 同左               | RTK GPS 波特率               |
| `RTK_TIMEOUT`         | `1.0` 秒                   | 同左               | 串口 readline 超时时间       |
| `RTK_ENABLED`         | `1`（开）                  | 同左               | 设为 `0` 完全禁用 RTK        |
| `DATA_LOG_DIR`        | `data_log`                 | 同左               | CSV 录制文件存放目录         |
| `NAV_LOOKAHEAD_M`     | `2.0` m                    | 同左               | Pure Pursuit 前视距离        |
| `NAV_DECEL_RADIUS_M`  | `3.0` m                    | 同左               | 开始减速的距离门限           |
| `NAV_ARRIVE_FRAMES`   | `5`                        | 同左               | 连续 N 帧在容忍半径内确认到达 |
| `NAV_GPS_TIMEOUT_S`   | `5.0` 秒                   | 同左               | GPS 超时此时长后暂停导航     |
| `NAV_PID_KP`          | `0.8`                      | 同左               | 朝向 PID 比例增益            |
| `NAV_PID_KI`          | `0.01`                     | 同左               | 朝向 PID 积分增益            |
| `NAV_PID_KD`          | `0.05`                     | 同左               | 朝向 PID 微分增益            |
| `NAV_MA_WINDOW`       | `10`                       | 同左               | 移动均值 GPS 滤波器窗口大小  |
| `COMPASS_MIN_ACCURACY`| `2`                        | 同左               | BNO085 磁力计精度阈值（0–3）。摄像头固定在机器人上无法做画8字校准时，设为 `0` 接受未校准数据 |
| `COMPASS_OFFSET_DEG`  | `0.0`                      | 同左               | 静态罗盘偏置补偿（度）。朝北读取当前 bearing，设为 `−bearing` 做手动对齐 |

### 远程端（`01_remote_side/config.py`）

| 参数                    | 默认值       | 说明                             |
|-------------------------|--------------|----------------------------------|
| `ROBOT_HOST`            | **必须设置** | 机器人 IP 地址                   |
| `TCP_PORT`              | `9000`       | 控制信道端口                     |
| `STREAM_PORT`           | `8080`       | 视频流端口                       |
| `HEARTBEAT_INTERVAL`    | `0.5` 秒     | 心跳发送间隔                     |
| `KEY_REPEAT_INTERVAL`   | `0.1` 秒     | 按键重复发送间隔                 |
| `TCP_RECONNECT_DELAY`   | `2.0` 秒     | TCP 断线后重连等待时间           |
| `STREAM_RECONNECT_DELAY`| `3.0` 秒     | 视频流断流后重连等待时间         |
| `STREAM_STALE_TIMEOUT`  | `3.0` 秒     | 超过此时间无新帧则判定为断流     |

---

## 快速启动

### 机器人端

```bash
cd m2_system/00_robot_side
python main.py
```

```
=======================================================
   Farm Robot — Robot-side Launcher
=======================================================
  1. Local control
  2. Local control + camera
  3. Remote TCP control
  4. Remote TCP control + camera stream
  5. Local camera test
  6. Web joystick control (HTTP :8888, WS :8889)

  q. Quit
=======================================================
```

- 选项 **6**：启动 `web_controller.py`。手机访问 `http://<机器人IP>:8888/`，摇杆控制、IMU/RTK HUD、CSV 录制和自主导航均集成在同一页面。

### Web 摇杆 + 自主导航（模式 D / E）

```bash
# 机器人端
cd m2_system/00_robot_side
python web_controller.py

# 手机 / 平板 — 同局域网下用浏览器打开
http://<机器人IP>:8888/
```

**手动控制（模式 D）：**

```
force < 0.15      → 死区，机器人停止
摇杆向上          → 前进（linear = +force × MAX_LINEAR_VEL × 速度比例）
摇杆右上 45°      → 前进 + 右转（对角线运动）
松开摇杆          → 立即发送急停
断开/无心跳 2 秒  → 看门狗触发急停
```

**自主导航（模式 E）：**

1. 在 QGIS 中绘制路径并导出 CSV，列格式：`id,lat,lon,tolerance_m,max_speed`
2. 在浏览器中点击 **📂 UPLOAD CSV** 并选择文件
3. 确认按钮旁显示的航点数量
4. 选择导航算法（**P2P** 或 **PURSUIT**）和 GPS 滤波器（**MOV-AVG** 或 **KALMAN**）
5. 在 RTK 面板确认定位质量 ≥ 1；若 GPS 尚未定位，可点击 **FORCE** 按钮跳过 fix 检查强制启动
6. 点击 **▶ AUTO** — 摇杆区显示"AUTO MODE"并禁用
7. 在导航状态面板监控进度（航点序号、距离、方位角）
8. 完成后机器人自动停车；随时点击 **■ STOP** 可中止

**CSV 航点格式：**

```csv
id,lat,lon,tolerance_m,max_speed
0,30.12345,120.98765,1.0,0.5
1,30.12400,120.98800,1.0,0.5
2,30.12460,120.98840,1.5,0.3
```

| 列              | 单位 | 说明                                                           |
|-----------------|------|----------------------------------------------------------------|
| `id`            | —    | 从 0 开始的序号                                                |
| `lat` / `lon`   | °    | WGS-84 十进制度                                                |
| `tolerance_m`   | m    | 到达半径（RTK Fixed 时自动收紧到 0.5 m，Float 时 2.0 m）      |
| `max_speed`     | m/s  | 该航点段的最大前进速度                                         |

**导航算法：**

| 模式        | 行为                                                                     |
|-------------|--------------------------------------------------------------------------|
| **P2P**     | 直接朝向每个航点；PID 朝向控制 + 减速坡                                 |
| **PURSUIT** | 跟踪路径段上的前视点；产生更平滑的曲线                                   |

**GPS 滤波器：**

| 滤波器      | 行为                                                                              |
|-------------|-----------------------------------------------------------------------------------|
| **MOV-AVG** | 简单滑动窗口均值（窗口 = `NAV_MA_WINDOW`，需 10 个 GPS 样本预热）               |
| **KALMAN**  | 4D 卡尔曼滤波（位置 + 速度）；IMU 加速度计以 20 Hz 辅助预测步骤                |

### 远程端（一键启动）

```bash
export ROBOT_HOST=192.168.x.x   # 机器人 IP
cd m2_system/01_remote_side
python main.py
```

- cv2 窗口显示实时视频；键盘焦点在终端时 `wasd` 控制机器人
- 按 `q`（终端）或关闭视频窗口退出

---

## 视频流接口扩展（FrameSource）

`camera/camera_streamer.py` 通过 `FrameSource` 接口解耦了"帧内容"与"传输协议"。
替换 Pipeline 只需新建一个子类，**MJPEGServer 和远程端代码完全不用改动**：

```python
# 00_robot_side/camera/frame_source.py
class FrameSource(ABC):
    def open(self) -> None: ...
    def close(self) -> None: ...
    def get_frame(self) -> Optional[np.ndarray]: ...   # 返回 BGR 帧

# 内置实现
class SimpleColorSource(FrameSource): ...   # OAK-D 彩色帧（默认）

# 未来扩展示例
class YOLODetectionSource(FrameSource): ... # 叠加 bounding box
class DepthAlignSource(FrameSource): ...    # 彩色 + 深度拼图
```

---

## Feather M4 CAN 固件

- **路径**：`CIRCUITPY/code.py`
- **环境**：CircuitPython 7.3.2
- **协议库**：`lib/farm_ng/`（farm-ng Amiga Dev Kit）

工作流程：

1. 监听 USB 串口（115200 baud）
2. 同时解析两种协议：
   - **WASD**（单字节）：`w/s/a/d/空格/\r` → 增量速度调整
   - **V 命令**（多字节行）：`V{speed},{angular}\n` → 绝对速度设定（Web 摇杆 / 自主导航使用）
3. 以 20 Hz 发送 CAN RPDO1 帧，携带当前 `cmd_speed` + `cmd_ang_rate`
4. 接收 Amiga Dashboard 的 TPDO1 状态帧，同步控制状态
5. **响应 `\r`**：回报 `S:ACTIVE\n` 或 `S:READY\n`，让上位机始终获知真实 AUTO 状态；启动时发送一次 `S:READY\n` 以完成初始同步
6. **广播 `O:` 里程计**：每收到一次 TPDO1 帧，输出 `O:{meas_speed:.3f},{meas_ang_rate:.3f},{state_int},{soc}\n`（约 20 Hz），将实测轮速、控制状态枚举值和电池电量转发给上位机，用于卡尔曼滤波速度更新、HUD 实时显示和 CSV 录制

---

## 安全机制

| 机制                    | 说明                                                                         |
|-------------------------|------------------------------------------------------------------------------|
| 看门狗定时器            | 2 秒内无任何命令（含心跳）→ 自动发送急停                                     |
| 命令白名单              | `SerialWriter` 仅允许 `w/s/a/d/空格/\r` 通过                                 |
| 松键急停                | 所有方向键释放后，repeat loop 在 ≤100 ms 内发送急停                         |
| TCP 断线急停            | 客户端断开时 `robot_receiver.py` 立即发送急停                                |
| WS 断线急停             | 浏览器断开时 `web_controller.py` 立即发送 `V0.00,0.00\n`                     |
| 摇杆死区                | `force < 0.15` → 发送零速度命令                                              |
| 速度钳位                | 固件将 V 命令值钳位到 `[-1.0, 1.0]`                                          |
| 导航中禁用摇杆          | 自主导航模式下摇杆消息不传递到串口                                           |
| GPS 超时停车            | 超过 `NAV_GPS_TIMEOUT_S`（5 秒）无有效 GPS → 导航暂停并停车                 |
| 固件状态同步            | AUTO 切换由固件串口回报（`S:ACTIVE`/`S:READY`）确认，UI 状态始终反映固件真实状态 |
| 异常日志                | 所有异常均记录到 logger，禁止静默吞异常                                      |

---

## 日志

| 脚本 / 模块                  | 日志文件                                    |
|------------------------------|---------------------------------------------|
| `main.py`（机器人端）        | `00_robot_side/log/robot_main.log`          |
| `local_controller.py`        | `00_robot_side/log/local_controller.log`    |
| `robot_receiver.py`          | `00_robot_side/log/robot_receiver.log`      |
| `camera/camera_streamer.py`  | `00_robot_side/log/camera_streamer.log`     |
| `web_controller.py`          | `00_robot_side/log/web_controller.log`      |
| `sensors/rtk_reader.py`      | （通过根 logger 输出到 web_controller.log） |
| `navigation/*`               | （通过根 logger 输出到 web_controller.log） |
| `data_recorder.py`           | `00_robot_side/log/data_recorder.log`       |
| `main.py`（远程端）          | `01_remote_side/log/main.log`               |
| `remote_sender.py`           | `01_remote_side/log/remote_sender.log`      |
| `remote_viewer.py`           | `01_remote_side/log/remote_viewer.log`      |

日志格式：

```
2025-01-01 12:00:00,000 [INFO]    TCP server listening on 0.0.0.0:9000
2025-01-01 12:00:01,500 [INFO]    NavigationEngine: navigation started, mode=p2p, filter=moving_avg, waypoints=3
2025-01-01 12:00:02,100 [WARNING] NavigationEngine: navigating but compass accuracy is insufficient (accuracy=0/3). Robot motion stopped. Set COMPASS_MIN_ACCURACY=0 to accept uncalibrated data.
2025-01-01 12:00:02,200 [WARNING] NavigationEngine: waiting for GPS filter readiness (MovingAvg window not full), navigation paused
2025-01-01 12:00:06,300 [INFO]    [NAV STATUS] Target: WP#1/3 (30.123450,120.987650), Distance: 3.2m, Bearing error: +12.3°, Linear vel: 0.35, Angular vel: 0.18, Filter: MA(ready), GPS quality: 4
2025-01-01 12:00:15,200 [INFO]    WaypointManager: arrived at waypoint 0 (dist=0.48m, tol=0.50m)
2025-01-01 12:00:15,300 [INFO]    CoveragePlanner: generated 48 waypoints, row_spacing=1.00m, overlap=0%, direction=0°
2025-01-01 12:00:16,000 [INFO]    SerialReader: firmware state -> ACTIVE
```

> **注**：代码内部（logger、注释、docstring）全部使用英文，符合团队协作规范。

python3 -m http.server 8888 --directory 00_robot_side/web_static
