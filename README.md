# M2 System — Farm Robot LAN Control

Remote keyboard control, live video streaming, **mobile web joystick control**, and **autonomous GPS waypoint navigation** for the farm-ng Amiga agricultural robot over a local area network (LAN).

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
└── camera/camera_streamer.py ← FrameSource (swappable pipeline)

Mobile phone / browser (same LAN)
└── HTTP :8888 → index.html (nipplejs joystick + IMU HUD + RTK panel + NAV panel)
        │
        │  WebSocket :8889
        ▼
Robot side (00_robot_side/)
└── web_controller.py → serial → Feather M4 CAN → CAN bus → Amiga Dashboard
                      ← IMU  (OAK-D BNO085, 20 Hz broadcast)
                      ← RTK  (Emlid RS+, 1 Hz broadcast)
                      ← ODOM (Amiga TPDO1: speed, ang_rate, state, SOC — 20 Hz broadcast)
                      → CSV  (data_log/, manual start/stop via browser)

QGIS (CSV waypoints) or CoveragePlanner (auto-generated)
        │ upload via browser  OR  generate via browser COV panel
        ▼
    NavigationEngine (inside web_controller.py)
    ├── CoveragePlanner   — Boustrophedon row-by-row coverage path generator
    ├── WaypointManager   — CSV waypoint sequence + adaptive arrival tolerance
    ├── GPS filter        — MovingAverageFilter or KalmanFilter (IMU+odometry-aided)
    ├── Controller        — P2PController or PurePursuitController
    └── 20 Hz control loop → V commands → serial → Feather M4 CAN
                         ← O: odometry feedback (meas_speed, meas_ang_rate, state, SOC @ ~20 Hz)
```

---

## Repository Layout

```
m2_system/
├── 00_robot_side/                  # Robot PC (Mac Mini / Linux)
│   ├── config.py                   # All parameters (serial/TCP/cam/web/nav), env-overridable
│   ├── core/                       # Infrastructure package
│   │   ├── serial_writer.py        # Thread-safe serial wrapper with command whitelist
│   │   └── watchdog.py             # Watchdog timer — triggers emergency stop on timeout
│   ├── sensors/                    # Sensor layer
│   │   ├── imu_reader.py           # IMUReader daemon thread + quaternion_to_compass (OAK-D)
│   │   ├── esp32_imu_reader.py     # ESP32IMUReader — BNO085 via serial "roll,pitch,yaw,acc\n"
│   │   └── rtk_reader.py           # RTKReader daemon thread — NMEA GGA/RMC (Emlid RS+)
│   ├── navigation/                 # Navigation algorithm layer
│   │   ├── geo_utils.py            # Pure functions: Haversine, bearing, normalize_angle, projection
│   │   ├── waypoint.py             # Waypoint dataclass + WaypointManager (adaptive tolerance)
│   │   ├── gps_filter.py           # MovingAverageFilter + KalmanFilter (4D, IMU+odom-aided)
│   │   ├── controller.py           # PIDController, P2PController, PurePursuitController
│   │   ├── nav_engine.py           # NavigationEngine state machine (NavState/NavMode/FilterMode)
│   │   ├── coverage_planner.py     # CoveragePlanner — Boustrophedon row coverage path generator
│   │   └── field_boundary.py       # Field boundary tools: CSV extraction, convex hull, manual input
│   ├── camera/                     # Video layer
│   │   ├── frame_source.py         # FrameSource ABC + SimpleColorSource (OAK-D)
│   │   └── camera_streamer.py      # MJPEGServer: streams any FrameSource over HTTP
│   ├── robot_receiver.py           # TCP server + watchdog + serial forwarding
│   ├── local_controller.py         # Local keyboard → serial (no TCP required)
│   ├── web_controller.py           # Web joystick + autonomous nav + coverage planner: HTTP :8888 + WS :8889
│   ├── data_recorder.py            # DataRecorder — IMU+RTK+cmd CSV writer
│   ├── web_static/
│   │   ├── index.html              # Single-page HUD: joystick + speed + compass + IMU + RTK + NAV
│   │   └── nipplejs.min.js         # nipplejs local copy (no CDN required on LAN)
│   ├── main.py                     # Interactive launcher menu (recommended entry point)
│   ├── log/                        # Runtime logs (auto-created)
│   ├── data_log/                   # CSV data recordings (auto-created on first REC)
│   └── cam_demo/                   # OAK-D camera demo scripts
├── 01_remote_side/                 # Operator PC
│   ├── config.py                   # ROBOT_HOST, TCP/stream ports, reconnect delays
│   ├── remote_sender.py            # pynput + TCP client + heartbeat
│   ├── remote_viewer.py            # MJPEG pull + cv2.imshow + auto-reconnect
│   ├── main.py                     # One-shot launcher: sender (daemon) + viewer (main thread)
│   └── log/
├── CIRCUITPY/                      # Feather M4 CAN firmware (CircuitPython)
│   ├── code.py                     # Parses serial (WASD + V velocity); outputs O: odometry + S: state
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
          ◄─HTTP:8080── camera/camera_streamer.py ◄── FrameSource (OAK-D / YOLO / …)
```

### Mode D — Web joystick control (mobile-friendly)

Control the robot from any smartphone or tablet browser on the same LAN.
Provides proportional joystick input (diagonal motion supported), a live IMU / compass HUD,
RTK GPS display, and manual CSV data recording.

```
Phone browser ──HTTP:8888──► web_static/index.html   (joystick + speed slider + IMU + RTK + NAV)
              ──WS:8889────► web_controller.py ──serial──► Feather M4 CAN
              ◄─WS:8889───── web_controller.py ◄── OAK-D BNO085 IMU (20 Hz)
                                               ◄── Emlid RS+ RTK GPS  (1 Hz)
                                               ──► data_log/*.csv     (on REC)
```

### Mode E — Autonomous GPS waypoint navigation

Draw a path in QGIS, export as CSV, upload in the browser, press **▶ AUTO**.
The robot navigates each waypoint autonomously using RTK GPS + IMU fusion.

```
QGIS → export CSV waypoints
          │ upload via browser (WebSocket)
          ▼
web_controller.py
    └── NavigationEngine
         ├── WaypointManager   — adaptive arrival tolerance, consecutive-frame detection
         ├── GPS filter        — MovingAverageFilter (10-frame window) or
         │                       KalmanFilter (4D state: position + velocity, IMU-aided at 20 Hz)
         ├── P2PController     — direct point-to-point with PID heading control + decel ramp
         └── PurePursuitController — lookahead-point path tracking (smooth curves)
              │
              └── V commands (20 Hz) → serial → Feather M4 CAN → Amiga Dashboard
```

Joystick is automatically disabled during autonomous navigation.
The browser shows real-time progress: waypoint index, distance to target, bearing, and fix quality.
The compass HUD displays live bearing at all calibration levels (UNCAL / LOW / GOOD / PRECISE); the accuracy label updates in real-time.

> **Compass UNCAL workaround**: If the BNO085 compass cannot be calibrated (robot-mounted camera cannot rotate freely for figure-8 calibration), set `COMPASS_MIN_ACCURACY=0` to accept uncalibrated heading data. Use `COMPASS_OFFSET_DEG` to add a fixed offset if the raw bearing has a known bias.

> **Navigation diagnostics**: While navigating, the terminal prints a `[NAV STATUS]` line every 5 seconds with current waypoint, distance, bearing error, and commanded velocities. If the robot is not moving, warning messages are emitted every 5 seconds indicating the reason (compass not ready, GPS filter warming up, etc.).

### Mode F — Coverage path planning (Boustrophedon)

Automatically generate a lawnmower-style coverage path from a field boundary polygon, without manually placing waypoints in QGIS.

```
Browser COV panel
  ├── boundary: [[lat,lon], …]   (enter corner coordinates or extract from recorded CSV)
  ├── row_spacing: 1.0 m
  ├── direction_deg: 0           (0 = N-S rows, 90 = E-W rows)
  └── overlap: 0 %
          │ WebSocket: generate_coverage
          ▼
    CoveragePlanner (Boustrophedon algorithm)
      1. Project GPS boundary → local ENU plane (centroid as origin)
      2. Rotate to scan frame: x' = travel direction, y' = row spacing direction
      3. Clip scan lines (y' = const) against the polygon
      4. Connect endpoints in S-shape (alternating direction per row)
      5. Back-project scan frame → ENU → WGS-84
          │ auto-load into NavigationEngine
          ▼
    WaypointManager → 20 Hz control loop → serial → Feather M4 CAN
```

**Coverage workflow:**

1. On the browser, tap the **COV** button in the bottom bar
2. Enter field boundary coordinates (one `lat,lon` per line, at least 3 points)
3. Set row spacing, travel direction, and overlap percentage
4. Tap **⚡ GENERATE PATH** — waypoints are loaded automatically
5. Confirm the waypoint count, then tap **▶ AUTO** to start navigation

> Alternatively, use `field_boundary.py` to extract a convex hull from a previously recorded `data_log/*.csv` file and pass it to `CoveragePlanner` programmatically.

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

Also sent once at boot so the host always knows the initial state.

### O command — odometry feedback (firmware → host, ~20 Hz)

The Feather M4 broadcasts measured wheel speed, control state, and battery SOC from each Amiga TPDO1 frame:

```
Format:  "O:{meas_speed:.3f},{meas_ang_rate:.3f},{state_int},{soc}\n"
Example: "O:0.253,-0.012,3,85\n"   →  linear 0.253 m/s, angular -0.012 rad/s, AUTO_ACTIVE (3), battery 85%
         "O:0.000,0.000,1,72\n"    →  robot stationary, AUTO_READY (1), battery 72%
```

`web_controller.py` parses `O:` lines in the serial reader thread (backward-compatible: 2-field format also accepted).
It stores the snapshot in `_last_odom` and:
- Calls `NavigationEngine.on_odometry(v, w)` — In **Kalman** filter mode, measured velocity is rotated to NED and used as a velocity observation to tighten the Kalman state, reducing GPS drift between fixes.
- Broadcasts `{"type": "odom", v, w, state, soc, ts}` to all browser clients at 20 Hz.
- Writes `odom_speed`, `odom_angrate`, and `amiga_soc` columns to every CSV recording row.

**AmigaControlState values** (from `CIRCUITPY/packet.py`):

| Integer | State name              | Meaning                    |
|---------|-------------------------|----------------------------|
| 0       | `STATE_STOPPED`         | Firmware stopped           |
| 1       | `STATE_AUTO_READY`      | Ready to activate          |
| 2       | `STATE_AUTO_ACTIVE`     | Actively executing commands|
| 3–7     | other states            | Error / transition states  |

### V command (new, absolute velocity)

```
Format:  "V{speed:.2f},{angular:.2f}\n"
Example: "V0.50,-0.30\n"   →  forward 0.5 m/s, turn right 0.3 rad/s
         "V0.00,0.00\n"    →  emergency stop
         "V-0.30,0.20\n"   →  reverse + turn left
```

Values are clamped to `[-1.0, 1.0]` on the firmware side. Both protocols are active simultaneously.

---

## WebSocket Protocol (browser ↔ web_controller.py)

### Client → Server

| `type`              | Payload                                         | Description                          |
|---------------------|-------------------------------------------------|--------------------------------------|
| `heartbeat`         | —                                               | Keeps watchdog alive                 |
| `joystick`          | `{linear, angular, force}`                      | Proportional velocity command        |
| `toggle_state`      | —                                               | Toggle AUTO_READY ↔ AUTO_ACTIVE      |
| `toggle_record`     | —                                               | Start / stop CSV recording           |
| `upload_waypoints`  | `{csv: "id,lat,lon,tolerance_m,max_speed\n…"}` | Upload QGIS waypoint CSV             |
| `nav_start`         | —                                               | Begin autonomous navigation (requires GPS fix ≥ 1) |
| `nav_start_force`   | —                                               | Force-start navigation ignoring GPS fix check |
| `nav_stop`          | —                                               | Stop autonomous navigation           |
| `nav_mode`          | `{mode: "p2p" \| "pure_pursuit"}`              | Switch navigation algorithm          |
| `filter_mode`       | `{mode: "moving_avg" \| "kalman"}`             | Switch GPS filter                    |
| `generate_coverage` | `{boundary:[[lat,lon],…], row_spacing, direction_deg, overlap, tolerance_m, max_speed}` | Generate and load a Boustrophedon coverage path |

### Server → Client

| `type`             | Key fields                                                      | Description                       |
|--------------------|-----------------------------------------------------------------|-----------------------------------|
| `imu`              | `accel, gyro, compass`                                          | 20 Hz IMU broadcast               |
| `rtk`              | `lat, lon, alt, fix_quality, num_sats, hdop`                   | 1 Hz RTK GPS broadcast            |
| `odom`             | `{v, w, state, soc, ts}`                                        | 20 Hz odometry broadcast: actual wheel speed (m/s), angular rate (rad/s), AmigaControlState integer, battery SOC (%) |
| `state_status`     | `{active: bool}`                                                | Firmware AUTO state change        |
| `record_status`    | `{recording, filename}`                                         | CSV recording state change        |
| `status`           | `{serial_ok, imu_ok, rtk_ok, recording}`                       | 2 Hz system health                |
| `waypoints_loaded` | `{count: N}`                                                    | CSV parse result                  |
| `nav_status`       | `{state, progress:[i,n], distance_m, target_bearing, nav_mode, filter_mode, tolerance_m}` | ~4 Hz navigation status |
| `nav_complete`     | `{total_wp: N}`                                                 | All waypoints reached             |
| `nav_warning`      | `{msg: "GPS timeout"}`                                          | Navigation paused due to GPS loss |
| `coverage_ready`   | `{count: N}` or `{count:0, error:"…"}`                         | Coverage path generated and loaded |

---

## Key Bindings (keyboard modes)

| Key     | Action                                             |
|---------|----------------------------------------------------|
| `W`     | Forward (+0.1 m/s)                                 |
| `S`     | Backward (−0.1 m/s)                                |
| `A`     | Turn left (+0.1 rad/s)                             |
| `D`     | Turn right (−0.1 rad/s)                            |
| `Space` | Emergency stop (also sent automatically within ≤100 ms when all keys released) |
| `Enter` | Toggle state: AUTO_READY ↔ AUTO_ACTIVE             |
| `Q`     | Quit                                               |

> Direction keys are sent repeatedly at 10 Hz. Releasing all direction keys causes the repeat loop to send an emergency stop within ≤100 ms (Linux evdev: avoids false key-release events during long-press).

---

## Installation

```bash
# Robot side (Mac Mini / Linux)
pip install pyserial depthai opencv-python websockets numpy

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
| `FRONT_CAM_IP`        | (= `CAM1_IP`)              | same               | Front camera IP; always routed to port 8080 when `CAM_SELECTION=both` |
| `MJPEG_QUALITY`       | `80`                       | same               | JPEG encoding quality (1–100)      |
| `LOCAL_DISPLAY`       | `0` (off)                  | same               | Set `1` for local preview window   |
| `WEB_HTTP_PORT`       | `8888`                     | same               | Web joystick HTTP port             |
| `WEB_WS_PORT`         | `8889`                     | same               | Web joystick WebSocket port        |
| `MAX_LINEAR_VEL`      | `1.0` m/s                  | same               | Maximum linear velocity            |
| `MAX_ANGULAR_VEL`     | `1.0` rad/s                | same               | Maximum angular velocity           |
| `COORD_SYSTEM`        | `NED`                      | same               | IMU coordinate frame: `NED` or `ENU` |
| `IMU_SOURCE`          | `esp32`                    | same               | IMU backend: `esp32` (BNO085 via serial) or `oakd` (OAK-D depthai) |
| `ESP32_IMU_PORT`      | `/dev/ttyUSB0`             | same               | ESP32 serial port (when `IMU_SOURCE=esp32`) |
| `ESP32_IMU_BAUD`      | `115200`                   | same               | ESP32 serial baud rate             |
| `RTK_PORT`            | `/dev/cu.usbmodem2403`     | same               | Emlid RS+ serial port              |
| `RTK_BAUD`            | `9600`                     | same               | RTK GPS baud rate                  |
| `RTK_TIMEOUT`         | `1.0` s                    | same               | Serial readline timeout            |
| `RTK_ENABLED`         | `1` (on)                   | same               | Set `0` to disable RTK             |
| `DATA_LOG_DIR`        | `data_log`                 | same               | Directory for CSV recordings       |
| `NAV_LOOKAHEAD_M`     | `2.0` m                    | same               | Pure Pursuit lookahead distance    |
| `NAV_DECEL_RADIUS_M`  | `3.0` m                    | same               | Distance at which robot starts slowing down |
| `NAV_ARRIVE_FRAMES`   | `5`                        | same               | Consecutive frames inside tolerance to confirm arrival |
| `NAV_GPS_TIMEOUT_S`   | `5.0` s                    | same               | Pause navigation if no GPS for this duration |
| `NAV_PID_KP`          | `0.8`                      | same               | Heading PID proportional gain      |
| `NAV_PID_KI`          | `0.01`                     | same               | Heading PID integral gain          |
| `NAV_PID_KD`          | `0.05`                     | same               | Heading PID derivative gain        |
| `NAV_MA_WINDOW`       | `10`                       | same               | Moving-average GPS filter window   |
| `COMPASS_MIN_ACCURACY`| `2`                        | same               | BNO085 accuracy threshold 0–3. Set to `0` to accept uncalibrated data when the camera is robot-mounted and cannot rotate for figure-8 calibration |
| `COMPASS_OFFSET_DEG`  | `0.0`                      | same               | Static compass offset in degrees. Point robot north, read current bearing, set to `−bearing` to compensate |

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

- Option **6**: starts `web_controller.py`. Open `http://<robot-ip>:8888/` on your phone — joystick control, IMU/RTK HUD, CSV recording, and autonomous navigation all in one page.

### Web joystick + autonomous navigation (Mode D / E)

```bash
# Robot side
cd m2_system/00_robot_side
python web_controller.py

# Browser (same LAN)
http://<robot-ip>:8888/
```

**Manual control (Mode D):**

```
force < 0.15              → dead zone, robot stops
joystick up               → forward  (linear = +force × MAX_LINEAR_VEL × speed_ratio)
joystick up + right       → forward + turn right (diagonal motion)
release joystick          → immediate stop
disconnect / no heartbeat → watchdog stops robot after 2 s
```

**Autonomous navigation (Mode E):**

1. Draw a path in QGIS and export as CSV with columns: `id,lat,lon,tolerance_m,max_speed`
2. In the browser, tap **📂 UPLOAD CSV** and select the file
3. Confirm waypoint count displayed next to the button
4. Choose navigation algorithm (**P2P** or **PURSUIT**) and GPS filter (**MOV-AVG** or **KALMAN**)
5. Verify RTK fix quality ≥ 1 in the RTK panel; if GPS is unavailable, tap **FORCE** instead of **▶ AUTO** to skip the fix check
6. Tap **▶ AUTO** — the joystick zone shows "AUTO MODE" and is disabled
7. Monitor progress in the navigation status panel (waypoint index, distance, bearing)
8. On completion the robot stops automatically; tap **■ STOP** at any time to abort

**CSV waypoint format:**

```csv
id,lat,lon,tolerance_m,max_speed
0,30.12345,120.98765,1.0,0.5
1,30.12400,120.98800,1.0,0.5
2,30.12460,120.98840,1.5,0.3
```

| Column        | Unit | Description                                                       |
|---------------|------|-------------------------------------------------------------------|
| `id`          | —    | Sequential index (0-based)                                        |
| `lat` / `lon` | °    | WGS-84 decimal degrees                                            |
| `tolerance_m` | m    | Arrival radius (auto-tightened to 0.5 m with RTK fix, 2.0 m float) |
| `max_speed`   | m/s  | Maximum forward speed for this waypoint segment                   |

**Navigation algorithms:**

| Mode          | Behaviour                                                                |
|---------------|--------------------------------------------------------------------------|
| **P2P**       | Points directly at each waypoint; PID heading control + deceleration ramp |
| **PURSUIT**   | Tracks a lookahead point on the path segment; produces smoother curves   |

**GPS filters:**

| Filter      | Behaviour                                                                    |
|-------------|------------------------------------------------------------------------------|
| **MOV-AVG** | Simple sliding-window mean (window = `NAV_MA_WINDOW`, requires 10 GPS fixes to warm up) |
| **KALMAN**  | 4D Kalman filter (position + velocity); IMU accelerometer aids prediction at 20 Hz |

### Remote side — all-in-one

```bash
export ROBOT_HOST=192.168.x.x    # robot IP
cd m2_system/01_remote_side
python main.py
```

---

## Extending the Video Pipeline (FrameSource)

`camera/camera_streamer.py` decouples *frame content* from *transport*.
Swap in a new pipeline by subclassing `FrameSource` — the `MJPEGServer` and all remote-side code stay unchanged.

```python
# 00_robot_side/camera/frame_source.py
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

## Feather M4 CAN Firmware

- **Path**: `CIRCUITPY/code.py`
- **Runtime**: CircuitPython 7.3.2
- **Protocol library**: `lib/farm_ng/` (farm-ng Amiga Dev Kit)

Workflow:

1. Listen on USB serial (115200 baud).
2. Parse commands from two protocols simultaneously:
   - **WASD** (single-byte): `w/s/a/d/space/\r` → incremental speed adjustment
   - **V command** (multi-byte line): `V{speed},{angular}\n` → absolute velocity
3. Send CAN RPDO1 frame at 20 Hz with current `cmd_speed` + `cmd_ang_rate`.
4. Receive TPDO1 status frames from the Amiga Dashboard to sync control state.
5. **Reply to `\r`** with `S:ACTIVE\n` or `S:READY\n` so the host always knows the actual AUTO state.
6. **Broadcast `O:` odometry** on every received TPDO1 frame (~20 Hz): `O:{meas_speed:.3f},{meas_ang_rate:.3f},{state_int},{soc}\n` — measured wheel velocities, AmigaControlState integer, and battery SOC forwarded to the host for Kalman filter velocity updates, real-time HUD display, and CSV recording.

---

## Safety

| Mechanism               | Description                                                          |
|-------------------------|----------------------------------------------------------------------|
| Watchdog timer          | No command (including heartbeat) for 2 s → automatic emergency stop  |
| Command whitelist       | `SerialWriter` only passes `w/s/a/d/space/\r`                        |
| Key-release stop        | All direction keys released → repeat loop sends emergency stop within ≤100 ms |
| TCP disconnect stop     | `robot_receiver.py` sends emergency stop when client disconnects      |
| WS disconnect stop      | `web_controller.py` sends `V0.00,0.00\n` when browser disconnects    |
| Joystick dead zone      | `force < 0.15` → zero velocity command sent                           |
| Velocity clamp          | Firmware clamps V command values to `[-1.0, 1.0]`                    |
| Joystick disabled in AUTO | Navigation mode blocks joystick messages from reaching the serial port |
| GPS timeout stop        | If no valid GPS for `NAV_GPS_TIMEOUT_S` (5 s), navigation pauses and robot stops |
| Firmware state sync     | AUTO toggle confirmed by firmware serial reply; UI always reflects actual firmware state |
| Exception logging       | All exceptions are logged; silent swallowing is forbidden             |

---

## Logs

| Script / Module              | Log file                                    |
|------------------------------|---------------------------------------------|
| `main.py` (robot side)       | `00_robot_side/log/robot_main.log`          |
| `local_controller.py`        | `00_robot_side/log/local_controller.log`    |
| `robot_receiver.py`          | `00_robot_side/log/robot_receiver.log`      |
| `camera/camera_streamer.py`  | `00_robot_side/log/camera_streamer.log`     |
| `web_controller.py`          | `00_robot_side/log/web_controller.log`      |
| `sensors/rtk_reader.py`      | (logs via root logger to web_controller.log)|
| `navigation/*`               | (logs via root logger to web_controller.log)|
| `data_recorder.py`           | `00_robot_side/log/data_recorder.log`       |
| `main.py` (remote side)      | `01_remote_side/log/main.log`               |
| `remote_sender.py`           | `01_remote_side/log/remote_sender.log`      |
| `remote_viewer.py`           | `01_remote_side/log/remote_viewer.log`      |

Log format:

```
2025-01-01 12:00:00,000 [INFO]    TCP server listening on 0.0.0.0:9000
2025-01-01 12:00:01,500 [INFO]    NavigationEngine: navigation started, mode=p2p, filter=moving_avg, waypoints=3
2025-01-01 12:00:02,100 [WARNING] NavigationEngine: navigating but compass accuracy is insufficient (accuracy=0/3). Robot motion stopped. Set COMPASS_MIN_ACCURACY=0 to accept uncalibrated data.
2025-01-01 12:00:02,200 [WARNING] NavigationEngine: waiting for GPS filter readiness (MovingAvg window not full), navigation paused
2025-01-01 12:00:06,300 [INFO]    [NAV STATUS] Target: WP#1/3 (30.123450,120.987650), Distance: 3.2m, Bearing error: +12.3°, Linear vel: 0.35, Angular vel: 0.18, Filter: MA(ready), GPS quality: 4
2025-01-01 12:00:15,200 [INFO]    WaypointManager: arrived at waypoint 0 (dist=0.48m, tol=0.50m)
2025-01-01 12:00:15,300 [INFO]    CoveragePlanner: generated 48 waypoints, row_spacing=1.00m, overlap=0%, direction=0°
2025-01-01 12:00:16,000 [INFO]    SerialReader: firmware state -> ACTIVE
2025-01-01 12:00:16,100 [INFO]    KalmanFilter: initialized at origin (30.123450, 120.987650)
```
