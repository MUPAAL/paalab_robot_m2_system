# M2 System — 农业机器人 LAN 控制系统

通过局域网（LAN）对 farm-ng Amiga 农业机器人进行远程键盘控制、视频流传输，以及**手机网页摇杆控制**。

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
  └── camera_streamer.py  ← FrameSource（可插拔 Pipeline）

手机浏览器（同 LAN）
└── HTTP :8888 → index.html（nipplejs 摇杆 + IMU HUD）
        │
        │  WebSocket :8889
        ▼
  机器人端 (00_robot_side/)
  └── web_controller.py → 串口 → Feather M4 CAN → CAN 总线 → Amiga Dashboard
                        ← OAK-D BNO085 IMU（20 Hz 广播）
```

---

## 目录结构

```
m2_system/
├── 00_robot_side/                  # 机器人端（Mac Mini / Linux）
│   ├── config.py                   # 串口/TCP/看门狗/相机/Web 参数（支持环境变量覆盖）
│   ├── serial_writer.py            # 线程安全串口封装，命令白名单过滤
│   ├── watchdog.py                 # 看门狗定时器，超时自动急停
│   ├── robot_receiver.py           # TCP 服务端 + 看门狗 + 串口转发
│   ├── local_controller.py         # 本地键盘直连串口（无需 TCP）
│   ├── frame_source.py             # FrameSource ABC + SimpleColorSource（OAK-D）
│   ├── camera_streamer.py          # MJPEGServer：将 FrameSource 推流为 HTTP MJPEG
│   ├── web_controller.py           # Web 摇杆：HTTP :8888 + WebSocket :8889 + IMU 广播
│   ├── web_static/
│   │   ├── index.html              # 单页 HUD（nipplejs 摇杆 + 罗盘 + IMU 数据）
│   │   └── nipplejs.min.js         # nipplejs 本地副本（LAN 无需 CDN）
│   ├── main.py                     # 交互式启动菜单（推荐入口）
│   ├── log/                        # 运行日志（自动创建）
│   └── cam_demo/                   # OAK-D 相机示例脚本
│       ├── camera_viewer.py
│       ├── Camera_output.py
│       ├── Camera_multiple_outputs.py
│       ├── Display_all_cameras.py
│       ├── Depth_Align.py
│       ├── Detection_network.py
│       ├── Detection_network_Remap.py
│       ├── Feature_Tracker.py
│       └── IMU.py
├── 01_remote_side/                 # 远程端（开发 PC / 操作站）
│   ├── config.py                   # ROBOT_HOST、TCP/STREAM 端口、重连延迟
│   ├── remote_sender.py            # pynput 键盘 + TCP 客户端 + 心跳（可独立运行）
│   ├── remote_viewer.py            # MJPEG 拉流 + cv2.imshow + 断流重连（可独立运行）
│   ├── main.py                     # 一键启动：sender（后台线程）+ viewer（主线程）
│   └── log/                        # 运行日志（自动创建）
├── CIRCUITPY/                      # Feather M4 CAN 固件（CircuitPython）
│   ├── code.py                     # 解析串口命令（WASD + V 速度命令）→ CAN 帧
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
        ◄─HTTP:8080── camera_streamer.py ◄── FrameSource（OAK-D / YOLO / ...）
```

### 模式 D：Web 摇杆控制（手机/平板友好）

用同局域网内的任意手机或平板浏览器控制机器人。
支持**比例控制**（对角线运动）和实时 IMU / 罗盘 HUD 显示。

```
手机浏览器 ──HTTP:8888──► web_static/index.html（nipplejs 摇杆 + IMU HUD）
          ──WS:8889────► web_controller.py ──串口──► Feather M4 CAN
          ◄─WS:8889───── web_controller.py ◄── OAK-D BNO085 IMU（20 Hz）
```

与模式 B 的主要区别：
- **比例控制**：摇杆直接映射到绝对速度，不再是增量步进
- **对角线运动**：线速度和角速度同时设定，一条命令完成
- **IMU HUD**：线加速度（已去除重力分量）、陀螺仪、磁力罗盘实时显示于浏览器
- 无需安装任何 App，现代手机浏览器直接访问

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
| `\r`    | 切换 AUTO_READY ↔ AUTO_ACTIVE              |

### V 命令（新增，绝对速度）

```
格式：  "V{speed:.2f},{angular:.2f}\n"
示例：  "V0.50,-0.30\n"   →  前进 0.5 m/s，右转 0.3 rad/s
        "V0.00,0.00\n"    →  急停
        "V-0.30,0.20\n"   →  后退 + 左转（对角线运动）
```

速度值在固件侧钳位到 `[-1.0, 1.0]`。两种协议同时有效。

---

## 键位说明（键盘模式）

| 按键    | 功能                                    |
|---------|-----------------------------------------|
| `W`     | 前进（速度 +0.1 m/s）                   |
| `S`     | 后退（速度 −0.1 m/s）                   |
| `A`     | 左转（角速度 +0.1 rad/s）               |
| `D`     | 右转（角速度 −0.1 rad/s）               |
| `空格`  | 紧急停止（松开所有按键时也自动触发）    |
| `Enter` | 切换状态：AUTO_READY ↔ AUTO_ACTIVE      |
| `Q`     | 退出程序                                |

> 方向键持续发送（10 Hz），松开所有方向键后自动发送急停。

---

## 安装依赖

```bash
# 机器人端（Mac Mini / Linux）
pip install pyserial depthai opencv-python websockets

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
| `MJPEG_QUALITY`       | `80`                       | 同左               | JPEG 编码质量（1–100）       |
| `LOCAL_DISPLAY`       | `0`（关）                  | 同左               | `1` 开启机器人端本地预览     |
| `CAM_SELECTION`       | `1`                        | 同左               | 相机选择：`"1"`、`"2"` 或 `"both"`（由启动菜单注入） |
| `DEVICE_IP`           | *（自动检测）*             | 同左               | 指定 cam_demo 脚本连接的 OAK-D PoE IP（由启动菜单注入） |
| `WEB_HTTP_PORT`       | `8888`                     | 同左               | Web 摇杆 HTTP 端口           |
| `WEB_WS_PORT`         | `8889`                     | 同左               | Web 摇杆 WebSocket 端口      |
| `MAX_LINEAR_VEL`      | `1.0` m/s                  | 同左               | 摇杆最大线速度               |
| `MAX_ANGULAR_VEL`     | `1.0` rad/s                | 同左               | 摇杆最大角速度               |
| `COORD_SYSTEM`        | `NED`                      | 同左               | IMU 坐标系：`NED`（x=北）或 `ENU`（x=东），影响罗盘方向转换 |

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

- 选项 **2**：弹出相机选择菜单，再启动 `local_controller.py` + `Camera_multiple_outputs.py`。选择**两台相机**时，并行开启两个 `Camera_multiple_outputs.py` 窗口（各自注入对应相机的 `DEVICE_IP`）
- 选项 **4**：弹出相机选择菜单，再同时启动 `robot_receiver.py` + `camera_streamer.py`，按 `Ctrl+C` 两者一起停止
- 选项 **5**：弹出相机选择菜单，再进入本地相机测试子菜单。选择**两台相机**时，所选 demo 会并行启动两个进程——各对应一台相机，各自注入 `DEVICE_IP`
- 选项 **6**：启动 `web_controller.py`，手机访问 `http://<机器人IP>:8888/`

**相机选择提示（选项 2 / 4 / 5）：**

```
────────────────────────────
  请选择相机 / Select Camera
────────────────────────────
  1. 相机 1  (IP: 10.95.76.10)
  2. 相机 2  (IP: 10.95.76.11)
  3. 两台相机
────────────────────────────
  选择 [1/2/3]:
```

| 选择 | `camera_streamer.py` 端口        | 注入的 `DEVICE_IP`                             |
|------|----------------------------------|------------------------------------------------|
| 1    | 相机 1 → :8080                   | `10.95.76.10`                                  |
| 2    | 相机 2 → :8080                   | `10.95.76.11`                                  |
| both | 相机 1 → :8080，相机 2 → :8081   | 按进程各自注入：进程 1 用 CAM1_IP，进程 2 用 CAM2_IP |

### Web 摇杆（模式 D）

```bash
# 机器人端
cd m2_system/00_robot_side
python web_controller.py
# 日志会打印机器人的局域网 IP 地址

# 手机 / 平板 — 同局域网下用浏览器打开
http://<机器人IP>:8888/
```

摇杆速度映射规则：

```
force < 0.15      → 死区，机器人停止
摇杆向上          → 前进（linear = +force × MAX_LINEAR_VEL）
摇杆右上 45°      → 前进 + 右转（对角线运动）
松开摇杆          → 立即发送急停
断开/无心跳 2 秒  → 看门狗触发急停
```

### 远程端（一键启动）

```bash
export ROBOT_HOST=192.168.x.x   # 机器人 IP
cd m2_system/01_remote_side
python main.py
```

- cv2 窗口显示实时视频；键盘焦点在终端时 `wasd` 控制机器人
- 按 `q`（终端）或关闭视频窗口退出

### 远程端（分开运行）

```bash
export ROBOT_HOST=192.168.x.x

# 仅控制
python remote_sender.py

# 仅视频
python remote_viewer.py
```

---

## 视频流接口扩展（FrameSource）

`camera_streamer.py` 通过 `FrameSource` 接口解耦了"帧内容"与"传输协议"。
替换 Pipeline 只需新建一个子类，**MJPEGServer 和远程端代码完全不用改动**：

```python
# 00_robot_side/frame_source.py
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

## 相机测试（菜单选项 5）

选择选项 **5** 后，先弹出相机选择提示，再进入本地相机测试子菜单。
所选相机的 IP 会通过 `DEVICE_IP` 环境变量注入每个 demo 脚本。

| 选项 | 功能                              |
|------|-----------------------------------|
| 1    | 简单查看器（300×300, CAM_A）      |
| 2    | 全分辨率多路显示                  |
| 3    | 多输出（300×300, CAM_A/B/C）      |
| 4    | 深度对齐（Depth Align）           |
| 5    | YOLO 目标检测                     |

`cam_demo/` 中所有脚本均读取 `DEVICE_IP` 环境变量：
- 单台相机（选 1 或 2）→ 启动一个进程，`DEVICE_IP` 设为所选相机的 IP
- **两台相机**→ 并行启动两个进程，分别注入 `CAM1_IP` / `CAM2_IP`，各自弹出独立显示窗口

其他 demo 脚本位于 `cam_demo/` 目录，包含 IMU 数据读取、特征追踪等功能。

---

## Feather M4 CAN 固件

- **路径**：`CIRCUITPY/code.py`
- **环境**：CircuitPython 7.3.2
- **协议库**：`lib/farm_ng/`（farm-ng Amiga Dev Kit）

工作流程：

1. 监听 USB 串口（115200 baud）
2. 同时解析两种协议：
   - **WASD**（单字节）：`w/s/a/d/空格/\r` → 增量速度调整
   - **V 命令**（多字节行）：`V{speed},{angular}\n` → 绝对速度设定（Web 摇杆使用）
3. 以 20 Hz 发送 CAN RPDO1 帧，携带当前 `cmd_speed` + `cmd_ang_rate`
4. 接收 Amiga Dashboard 的 TPDO1 状态帧，同步控制状态

---

## 安全机制

| 机制                | 说明                                                       |
|---------------------|------------------------------------------------------------|
| 看门狗定时器        | 2 秒内无任何命令（含心跳）→ 自动发送急停                   |
| 命令白名单          | `SerialWriter` 仅允许 `w/s/a/d/空格/\r` 通过               |
| 松键急停            | 所有方向键释放后立即发送急停字符                           |
| TCP 断线急停        | 客户端断开时 `robot_receiver.py` 立即发送急停              |
| WS 断线急停         | 浏览器断开时 `web_controller.py` 立即发送 `V0.00,0.00\n`   |
| 摇杆死区            | `force < 0.15` → 发送零速度命令                            |
| 速度钳位            | 固件将 V 命令值钳位到 `[-1.0, 1.0]`                        |
| 异常日志            | 所有异常均记录到 logger，禁止静默吞异常                    |

---

## 日志

| 脚本                    | 日志文件                                  |
|-------------------------|-------------------------------------------|
| `main.py`（机器人端）   | `00_robot_side/log/robot_main.log`        |
| `local_controller.py`   | `00_robot_side/log/local_controller.log`  |
| `robot_receiver.py`     | `00_robot_side/log/robot_receiver.log`    |
| `camera_streamer.py`    | `00_robot_side/log/camera_streamer.log`   |
| `web_controller.py`     | `00_robot_side/log/web_controller.log`    |
| `main.py`（远程端）     | `01_remote_side/log/main.log`             |
| `remote_sender.py`      | `01_remote_side/log/remote_sender.log`    |
| `remote_viewer.py`      | `01_remote_side/log/remote_viewer.log`    |

日志格式：

```
2025-01-01 12:00:00,000 [INFO] TCP server listening on 0.0.0.0:9000
2025-01-01 12:00:01,500 [INFO] Remote client connected: ('192.168.1.50', 54321)
2025-01-01 12:00:01,600 [INFO] MJPEG server started → http://0.0.0.0:8080/
```
