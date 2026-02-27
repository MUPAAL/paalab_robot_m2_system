# M2 System — Farm Robot LAN Control

Remote keyboard control, live video streaming, and **mobile web joystick control** for the farm-ng Amiga agricultural robot over a local area network (LAN).

> Chinese documentation: [README_zh.md](README_zh.md)

---

## Architecture

```
Remote PC (01_remote_side/)
├── Main thread  : cv2.imshow video display
├── Thread A     : VideoCapture pulls frames from HTTP:8080
├── Thread B     : pynput keyboard listener
└── Thread C     : TCP:9000 command sender + heartbeat
        │
        │  TCP :9000 (control)   HTTP :8080 (video)
        ▼
Robot side (00_robot_side/)
├── robot_receiver.py  → serial → Feather M4 CAN → CAN bus → Amiga Dashboard
└── camera_streamer.py ← FrameSource (swappable pipeline)

Mobile phone (browser, same LAN)
└── HTTP :8888 → index.html (nipplejs joystick + IMU HUD + RTK panel + REC button)
        │
        │  WebSocket :8889
        ▼
Robot side (00_robot_side/)
└── web_controller.py → serial → Feather M4 CAN → CAN bus → Amiga Dashboard
                      ← IMU  (OAK-D BNO085, 20 Hz broadcast)
                      ← RTK  (Emlid RS+, 1 Hz broadcast)
                      → CSV  (data_log/, manual start/stop via browser)
```

---

## Repository Layout

```
m2_system/
├── 00_robot_side/                  # Robot PC (Mac Mini / Linux)
│   ├── config.py                   # Serial / TCP / watchdog / camera / web params (env-overridable)
│   ├── serial_writer.py            # Thread-safe serial wrapper with command whitelist
│   ├── watchdog.py                 # Watchdog timer — triggers emergency stop on timeout
│   ├── robot_receiver.py           # TCP server + watchdog + serial forwarding
│   ├── local_controller.py         # Local keyboard → serial (no TCP required)
│   ├── frame_source.py             # FrameSource ABC + SimpleColorSource (OAK-D)
│   ├── camera_streamer.py          # MJPEGServer: streams any FrameSource over HTTP
│   ├── web_controller.py           # Web joystick: HTTP :8888 + WebSocket :8889 + IMU/RTK broadcast + CSV recorder
│   ├── rtk_reader.py               # RTKReader daemon thread — NMEA GGA/RMC parser (Emlid RS+)
│   ├── data_recorder.py            # DataRecorder — IMU+RTK+cmd CSV writer (start/stop via browser)
│   ├── web_static/
│   │   ├── index.html              # Single-page HUD (nipplejs joystick + compass + IMU + RTK panel + REC button)
│   │   └── nipplejs.min.js         # nipplejs local copy (no CDN required on LAN)
│   ├── main.py                     # Interactive launcher menu (recommended entry point)
│   ├── log/                        # Runtime logs (auto-created)
│   ├── data_log/                   # CSV data recordings (auto-created on first REC)
│   └── cam_demo/                   # OAK-D camera demo scripts
│       ├── camera_viewer.py
│       ├── Camera_output.py
│       ├── Camera_multiple_outputs.py
│       ├── Display_all_cameras.py
│       ├── Depth_Align.py
│       ├── Detection_network.py
│       ├── Detection_network_Remap.py
│       ├── Feature_Tracker.py
│       └── IMU.py
├── 01_remote_side/                 # Operator PC
│   ├── config.py                   # ROBOT_HOST, TCP/stream ports, reconnect delays
│   ├── remote_sender.py            # pynput + TCP client + heartbeat (standalone-capable)
│   ├── remote_viewer.py            # MJPEG pull + cv2.imshow + auto-reconnect (standalone-capable)
│   ├── main.py                     # One-shot launcher: sender (daemon thread) + viewer (main thread)
│   └── log/                        # Runtime logs (auto-created)
├── CIRCUITPY/                      # Feather M4 CAN firmware (CircuitPython)
│   ├── code.py                     # Parses serial commands (WASD + V velocity) → CAN frames
│   └── lib/farm_ng/                # farm-ng Amiga protocol library
├── CLAUDE.md
├── README.md                       # This file (English)
└── README_zh.md                    # Chinese documentation
```

---

## Operating Modes

### Mode A — Local control

Operator sits at the robot PC and controls it directly via keyboard.

```
Local PC (pynput) ──► local_controller.py ──serial──► Feather M4 CAN
```

- No TCP required; serial port is held directly.
- No watchdog (operator is physically present).
- **Cannot run simultaneously with `robot_receiver.py` or `web_controller.py` (serial port conflict).**

### Mode B — Remote TCP control

Operator controls the robot from a separate machine over LAN.

```
Remote PC (pynput) ──TCP:9000──► robot_receiver.py ──serial──► Feather M4 CAN
```

- Remote side sends motion commands + periodic heartbeats.
- Robot-side watchdog triggers an emergency stop if no message is received within 2 s.

### Mode C — Remote TCP control + camera stream (recommended)

Full remote operation: keyboard control and live video in a single command.

```
Remote PC ──TCP:9000──► robot_receiver.py ──serial──► Feather M4 CAN
          ◄─HTTP:8080── camera_streamer.py ◄── FrameSource (OAK-D / YOLO / …)
```

### Mode D — Web joystick control (mobile-friendly)

Control the robot from any smartphone or tablet browser on the same LAN.
Provides proportional joystick input (diagonal motion supported), a live IMU / compass HUD,
RTK GPS display, and manual CSV data recording.

```
Phone browser ──HTTP:8888──► web_static/index.html   (nipplejs joystick + IMU HUD + RTK + REC)
              ──WS:8889────► web_controller.py ──serial──► Feather M4 CAN
              ◄─WS:8889───── web_controller.py ◄── OAK-D BNO085 IMU (20 Hz)
                                               ◄── Emlid RS+ RTK GPS  (1 Hz)
                                               ──► data_log/*.csv     (on REC)
```

Key differences from Mode B:
- **Proportional control**: joystick maps directly to absolute speed values — no incremental steps.
- **Diagonal motion**: linear and angular velocity set simultaneously in a single command.
- **IMU HUD**: linear acceleration (gravity-compensated), gyroscope, and magnetic compass rendered in the browser.
- **RTK GPS panel**: live lat/lon/alt, fix quality badge (NO FIX / GPS / DGPS / RTK FIXED / RTK FLOAT), satellite count, HDOP, speed.
- **CSV recording**: tap **● REC** in the browser to start logging; tap **■ STOP** to close the file. Each recording session creates a timestamped file in `data_log/`.
- No dedicated app required — works in any modern mobile browser.

---

## Serial Protocol

### WASD (legacy, single-byte incremental)

| Byte    | Action                                   |
|---------|------------------------------------------|
| `w`     | `cmd_speed += 0.1`                       |
| `s`     | `cmd_speed -= 0.1`                       |
| `a`     | `cmd_ang_rate += 0.1`                    |
| `d`     | `cmd_ang_rate -= 0.1`                    |
| `Space` | Emergency stop (`cmd_speed = cmd_ang_rate = 0`) |
| `\r`    | Toggle AUTO_READY ↔ AUTO_ACTIVE; firmware replies `S:ACTIVE\n` or `S:READY\n` |

### Firmware state report (host ← firmware)

After receiving `\r`, the Feather M4 replies with one of:

```
S:ACTIVE\n   — request_state set to AUTO_ACTIVE
S:READY\n    — request_state set to AUTO_READY
```

On startup the firmware also sends `S:READY\n` once so the host can synchronise.
`web_controller.py` parses these lines in a dedicated `SerialReader` daemon thread and
broadcasts a `state_status` WebSocket message to all connected browser clients.

### V command (new, absolute velocity)

```
Format:  "V{speed:.2f},{angular:.2f}\n"
Example: "V0.50,-0.30\n"   →  forward 0.5 m/s, turn right 0.3 rad/s
         "V0.00,0.00\n"    →  emergency stop
         "V-0.30,0.20\n"   →  reverse + turn left (diagonal motion)
```

Values are clamped to `[-1.0, 1.0]` on the firmware side. Both protocols are active simultaneously.

---

## Key Bindings (keyboard modes)

| Key     | Action                                             |
|---------|----------------------------------------------------|
| `W`     | Forward (+0.1 m/s)                                 |
| `S`     | Backward (−0.1 m/s)                                |
| `A`     | Turn left (+0.1 rad/s)                             |
| `D`     | Turn right (−0.1 rad/s)                            |
| `Space` | Emergency stop (also sent automatically on key release) |
| `Enter` | Toggle state: AUTO_READY ↔ AUTO_ACTIVE             |
| `Q`     | Quit                                               |

> Direction keys are sent repeatedly at 10 Hz. Releasing all direction keys immediately sends an emergency stop.

---

## Installation

```bash
# Robot side (Mac Mini / Linux)
pip install pyserial depthai opencv-python websockets

# Remote side (operator PC)
pip install pynput opencv-python
```

---

## Configuration

### Robot side (`00_robot_side/config.py`)

| Parameter             | Default (macOS)            | Default (Linux)    | Description                        |
|-----------------------|----------------------------|--------------------|------------------------------------|
| `FEATHER_PORT`        | `/dev/cu.usbmodem2301`     | `/dev/ttyACM0`     | Feather M4 CAN serial port         |
| `SERIAL_BAUD`         | `115200`                   | same               | Serial baud rate                   |
| `TCP_PORT`            | `9000`                     | same               | TCP listening port                 |
| `WATCHDOG_TIMEOUT`    | `2.0` s                    | same               | Watchdog timeout                   |
| `KEY_REPEAT_INTERVAL` | `0.1` s (10 Hz)            | same               | Key repeat interval                |
| `CAM1_IP`             | `10.95.76.10`              | same               | OAK-D PoE camera 1 IP              |
| `CAM2_IP`             | `10.95.76.11`              | same               | OAK-D PoE camera 2 IP              |
| `CAM1_STREAM_PORT`    | `8080`                     | same               | Camera 1 MJPEG stream port         |
| `CAM2_STREAM_PORT`    | `8081`                     | same               | Camera 2 MJPEG stream port         |
| `MJPEG_QUALITY`       | `80`                       | same               | JPEG encoding quality (1–100)      |
| `LOCAL_DISPLAY`       | `0` (off)                  | same               | Set `1` for local preview window   |
| `CAM_SELECTION`       | `1`                        | same               | Camera selection: `"1"`, `"2"`, or `"both"` (set by launcher menu) |
| `DEVICE_IP`           | *(auto-detect)*            | same               | Force cam_demo scripts to connect to a specific OAK-D PoE IP (set by launcher menu) |
| `WEB_HTTP_PORT`       | `8888`                     | same               | Web joystick HTTP port             |
| `WEB_WS_PORT`         | `8889`                     | same               | Web joystick WebSocket port        |
| `MAX_LINEAR_VEL`      | `1.0` m/s                  | same               | Joystick maximum linear velocity   |
| `MAX_ANGULAR_VEL`     | `1.0` rad/s                | same               | Joystick maximum angular velocity  |
| `COORD_SYSTEM`        | `NED`                      | same               | IMU coordinate frame for compass: `NED` (x=North) or `ENU` (x=East) |
| `RTK_PORT`            | `/dev/cu.usbmodem2403`     | same               | Emlid RS+ serial port              |
| `RTK_BAUD`            | `9600`                     | same               | RTK GPS baud rate                  |
| `RTK_TIMEOUT`         | `1.0` s                    | same               | Serial readline timeout            |
| `RTK_ENABLED`         | `1` (on)                   | same               | Set `0` to disable RTK entirely    |
| `DATA_LOG_DIR`        | `data_log`                 | same               | Directory for CSV recordings       |

### Remote side (`01_remote_side/config.py`)

| Parameter               | Default      | Description                                   |
|-------------------------|--------------|-----------------------------------------------|
| `ROBOT_HOST`            | **required** | Robot IP address                              |
| `TCP_PORT`              | `9000`       | Control channel port                          |
| `STREAM_PORT`           | `8080`       | Video stream port                             |
| `HEARTBEAT_INTERVAL`    | `0.5` s      | Heartbeat send interval                       |
| `KEY_REPEAT_INTERVAL`   | `0.1` s      | Key repeat interval                           |
| `TCP_RECONNECT_DELAY`   | `2.0` s      | Wait time before retrying TCP connection      |
| `STREAM_RECONNECT_DELAY`| `3.0` s      | Wait time before retrying stream connection   |
| `STREAM_STALE_TIMEOUT`  | `3.0` s      | No-new-frame threshold for stale detection    |

---

## Quick Start

### Robot side

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

- Option **2**: prompts for camera selection, then starts `local_controller.py` + `Camera_multiple_outputs.py`. With **both**, opens two `Camera_multiple_outputs.py` windows in parallel (one per camera, each with its own `DEVICE_IP`).
- Option **4**: prompts for camera selection, then starts `robot_receiver.py` + `camera_streamer.py` in parallel. Press `Ctrl+C` to stop both.
- Option **5**: prompts for camera selection, then enters the local camera test sub-menu. With **both**, any selected demo launches two parallel processes—one for CAM1, one for CAM2.
- Option **6**: starts `web_controller.py`. Open `http://<robot-ip>:8888/` on your phone.

**Camera selection prompt (options 2 / 4 / 5):**

```
────────────────────────────
  Select Camera
────────────────────────────
  1. Camera 1  (IP: 10.95.76.10)
  2. Camera 2  (IP: 10.95.76.11)
  3. Both cameras
────────────────────────────
  Choice [1/2/3]:
```

| Selection | `camera_streamer.py` ports  | `DEVICE_IP` injected                              |
|-----------|-----------------------------|---------------------------------------------------|
| 1         | CAM1 → :8080                | `10.95.76.10`                                     |
| 2         | CAM2 → :8080                | `10.95.76.11`                                     |
| both      | CAM1 → :8080, CAM2 → :8081 | per-process: CAM1_IP to process 1, CAM2_IP to process 2 |

### Web joystick (Mode D)

```bash
# Robot side
cd m2_system/00_robot_side
python web_controller.py
# Logs will print the robot's LAN IP address

# Phone / tablet — open in browser (same LAN)
http://<robot-ip>:8888/
```

The joystick maps to proportional speed:

```
force < 0.15              → dead zone, robot stops
joystick up               → forward  (linear = +force × MAX_LINEAR_VEL)
joystick right + up       → forward + turn right (diagonal motion)
release joystick          → immediate stop
disconnect / no heartbeat → watchdog stops robot after 2 s
```

RTK GPS and data recording:

```
RTK panel (top of page)   → shows lat / lon / alt / fix quality / sats / HDOP / speed
● REC button              → starts a new CSV in data_log/robot_data_YYYYMMDD_HHMMSS.csv
■ STOP button             → closes the file
```

CSV columns: `timestamp, accel_x/y/z, gyro_x/y/z, compass_bearing, lat, lon, alt, fix_quality, num_sats, hdop, linear_cmd, angular_cmd`

### Remote side — all-in-one

```bash
export ROBOT_HOST=192.168.x.x    # robot IP
cd m2_system/01_remote_side
python main.py
```

- The cv2 window shows the live camera feed.
- With keyboard focus on the terminal, use `wasd` to drive the robot.
- Press `q` in the terminal, or close the video window, to quit everything.

### Remote side — individual components

```bash
export ROBOT_HOST=192.168.x.x

# Control only
python remote_sender.py

# Video only
python remote_viewer.py
```

---

## Extending the Video Pipeline (FrameSource)

`camera_streamer.py` decouples *frame content* from *transport*.
Swap in a new pipeline by subclassing `FrameSource` — the `MJPEGServer` and all remote-side code stay unchanged.

```python
# 00_robot_side/frame_source.py
class FrameSource(ABC):
    def open(self) -> None: ...
    def close(self) -> None: ...
    def get_frame(self) -> Optional[np.ndarray]: ...  # returns BGR frame

# Built-in implementation
class SimpleColorSource(FrameSource): ...   # raw OAK-D colour frame (default)

# Example future sources
class YOLODetectionSource(FrameSource): ... # colour frame + bounding-box overlay
class DepthAlignSource(FrameSource): ...    # colour + depth side-by-side
```

---

## Local Camera Tests (menu option 5)

Selecting option **5** first shows the camera selection prompt, then enters the sub-menu.
The chosen camera's IP is injected as `DEVICE_IP` into each demo script.

| Option | Function                                    |
|--------|---------------------------------------------|
| 1      | Simple viewer (300×300, CAM_A)              |
| 2      | All cameras, full resolution                |
| 3      | Multi-output (300×300, CAM_A / B / C)       |
| 4      | Depth Align demo                            |
| 5      | YOLO object detection demo                  |

All scripts in `cam_demo/` read `DEVICE_IP` from the environment:
- Single camera (1 or 2) → one process launched with `DEVICE_IP` set to the chosen camera's IP.
- **Both cameras** → two processes launched in parallel; each receives its own `DEVICE_IP` (`CAM1_IP` / `CAM2_IP`), producing two independent display windows.

Additional demo scripts are in the `cam_demo/` directory (IMU, feature tracking, etc.).

---

## Feather M4 CAN Firmware

- **Path**: `CIRCUITPY/code.py`
- **Runtime**: CircuitPython 7.3.2
- **Protocol library**: `lib/farm_ng/` (farm-ng Amiga Dev Kit)

Workflow:

1. Listen on USB serial (115200 baud).
2. Parse commands from two protocols simultaneously:
   - **WASD** (single-byte): `w/s/a/d/space/\r` → incremental speed adjustment
   - **V command** (multi-byte line): `V{speed},{angular}\n` → absolute velocity (set by web joystick)
3. Send CAN RPDO1 frame at 20 Hz with current `cmd_speed` + `cmd_ang_rate`.
4. Receive TPDO1 status frames from the Amiga Dashboard to sync control state.
5. **Reply to `\r`** with `S:ACTIVE\n` or `S:READY\n` so the host always knows the actual AUTO state.
   On startup, send `S:READY\n` once to initialise the host-side state.

---

## Safety

| Mechanism               | Description                                                          |
|-------------------------|----------------------------------------------------------------------|
| Watchdog timer          | No command (including heartbeat) for 2 s → automatic emergency stop  |
| Command whitelist       | `SerialWriter` only passes `w/s/a/d/space/\r`                        |
| Key-release stop        | All direction keys released → emergency stop sent immediately         |
| TCP disconnect stop     | `robot_receiver.py` sends emergency stop when client disconnects      |
| WS disconnect stop      | `web_controller.py` sends `V0.00,0.00\n` when browser disconnects    |
| Joystick dead zone      | `force < 0.15` → zero velocity command sent                           |
| Velocity clamp          | Firmware clamps V command values to `[-1.0, 1.0]`                    |
| Firmware state sync     | AUTO toggle is confirmed by firmware serial reply (`S:ACTIVE`/`S:READY`); UI state always reflects actual firmware state. Firmware restart auto-resynchronises via `S:READY\n` on boot. |
| Exception logging       | All exceptions are logged; silent swallowing is forbidden             |

---

## Logs

| Script                      | Log file                                    |
|-----------------------------|---------------------------------------------|
| `main.py` (robot side)      | `00_robot_side/log/robot_main.log`          |
| `local_controller.py`       | `00_robot_side/log/local_controller.log`    |
| `robot_receiver.py`         | `00_robot_side/log/robot_receiver.log`      |
| `camera_streamer.py`        | `00_robot_side/log/camera_streamer.log`     |
| `web_controller.py`         | `00_robot_side/log/web_controller.log`      |
| `rtk_reader.py`             | `00_robot_side/log/rtk_reader.log`          |
| `data_recorder.py`          | `00_robot_side/log/data_recorder.log`       |
| `main.py` (remote side)     | `01_remote_side/log/main.log`               |
| `remote_sender.py`          | `01_remote_side/log/remote_sender.log`      |
| `remote_viewer.py`          | `01_remote_side/log/remote_viewer.log`      |

Log format:

```
2025-01-01 12:00:00,000 [INFO] TCP server listening on 0.0.0.0:9000
2025-01-01 12:00:01,500 [INFO] Remote client connected: ('192.168.1.50', 54321)
2025-01-01 12:00:01,600 [INFO] MJPEG server started → http://0.0.0.0:8080/
```
