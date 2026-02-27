"""
Web Joystick Controller — HTTP static files + WebSocket + IMU + serial velocity output

Architecture:
  Thread-1: asyncio event loop
    ├─ websockets.serve() :WEB_WS_PORT  → _ws_handler()
    │    receives joystick commands → serial.write("V{linear:.2f},{angular:.2f}\n")
    ├─ _imu_broadcast_loop(): pushes IMU + compass at 20 Hz
    └─ _watchdog_loop(): 2 s without heartbeat → sends "V0.00,0.00\n" emergency stop
  Thread-2: ThreadingHTTPServer :WEB_HTTP_PORT (daemon, serves static files)
  Thread-3: IMUReader (depthai daemon thread, reads OAK-D IMU)

Serial port is opened directly via serial.Serial (bypasses SerialWriter whitelist).
Mutually exclusive with robot_receiver.py / local_controller.py (same serial port).

Usage:
    cd m2_system/00_robot_side
    python web_controller.py
"""

import asyncio
import json
import logging
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import serial
import websockets

from config import (
    FEATHER_PORT, SERIAL_BAUD, SERIAL_TIMEOUT,
    WEB_HTTP_PORT, WEB_WS_PORT,
    MAX_LINEAR_VEL, MAX_ANGULAR_VEL,
    WATCHDOG_TIMEOUT,
    RTK_PORT, RTK_BAUD, RTK_ENABLED,
    DATA_LOG_DIR,
)
from sensors.rtk_reader import RTKReader
from sensors.imu_reader import IMUReader
import sensors.imu_reader as _imu_mod
from data_recorder import DataRecorder
from navigation.nav_engine import NavigationEngine, NavMode, FilterMode

# ── Logging ────────────────────────────────────────────────
_py_name = Path(__file__).stem
Path("log").mkdir(exist_ok=True)
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler(f"log/{_py_name}.log", encoding="utf-8"),
        logging.StreamHandler(),
    ],
)
logger = logging.getLogger(__name__)

# ── Static files directory ────────────────────────────────
STATIC_DIR = Path(__file__).parent / "web_static"

# ── RTK reader (global, started in main()) ────────────────
_rtk_reader: RTKReader | None = None

# ── Data recorder (global, started in main()) ─────────────
_data_recorder: DataRecorder | None = None

# ── Last velocity command (protected by lock) ─────────────
_vel_lock   = threading.Lock()
_last_linear:  float = 0.0
_last_angular: float = 0.0


# ── HTTP static file server ───────────────────────────────
class StaticFileHandler(SimpleHTTPRequestHandler):
    """Serves files from STATIC_DIR; injects MAX_LINEAR/MAX_ANGULAR into index.html."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(STATIC_DIR), **kwargs)

    def do_GET(self):
        # Inject velocity config into index.html at request time
        if self.path in ('/', '/index.html'):
            self._serve_index()
        else:
            super().do_GET()

    def _serve_index(self):
        index_path = STATIC_DIR / "index.html"
        try:
            content = index_path.read_text(encoding="utf-8")
            # Inject data attributes into <html> tag so JS can read them
            content = content.replace(
                '<html lang="en">',
                f'<html lang="en" data-max-linear="{MAX_LINEAR_VEL}" data-max-angular="{MAX_ANGULAR_VEL}">'
            )
            encoded = content.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(encoded)))
            self.end_headers()
            self.wfile.write(encoded)
        except Exception as e:
            logger.error(f"HTTP: failed to serve index.html: {e}")
            self.send_error(500)

    def log_message(self, fmt, *args):
        logger.debug(f"HTTP: {fmt % args}")


def _start_http_server() -> None:
    """Start ThreadingHTTPServer in a daemon thread."""
    server = ThreadingHTTPServer(("0.0.0.0", WEB_HTTP_PORT), StaticFileHandler)
    t = threading.Thread(target=server.serve_forever, name="HTTPServer", daemon=True)
    t.start()
    logger.info(f"HTTP server started: http://0.0.0.0:{WEB_HTTP_PORT}/")


# ── WebSocket server ──────────────────────────────────────
class WebController:
    """Manages WebSocket connections, serial velocity output, and watchdog."""

    def __init__(self) -> None:
        self._ser: serial.Serial | None = None
        self._ser_lock = threading.Lock()
        self._clients: set = set()
        self._clients_lock = asyncio.Lock()
        self._last_heartbeat: float = time.time()
        self._serial_ok = False
        self._auto_active = False  # tracks current AUTO state (updated by serial reader thread)
        self._loop: asyncio.AbstractEventLoop | None = None
        self._nav_engine: NavigationEngine | None = None

    # ── Serial ────────────────────────────────────────────
    def open_serial(self) -> None:
        try:
            self._ser = serial.Serial(FEATHER_PORT, SERIAL_BAUD, timeout=SERIAL_TIMEOUT)
            self._serial_ok = True
            logger.info(f"Serial port opened: {FEATHER_PORT} @ {SERIAL_BAUD} baud")
        except serial.SerialException as e:
            logger.error(f"Failed to open serial port [{FEATHER_PORT}]: {e}")
            self._serial_ok = False

    def close_serial(self) -> None:
        with self._ser_lock:
            if self._ser and self._ser.is_open:
                self._ser.close()
                logger.info("Serial port closed")

    def _send_velocity(self, linear: float, angular: float) -> None:
        """Send direct velocity command V{linear:.2f},{angular:.2f}\\n to Feather M4."""
        global _last_linear, _last_angular
        cmd = f"V{linear:.2f},{angular:.2f}\n".encode()
        with self._ser_lock:
            if self._ser is None or not self._ser.is_open:
                logger.warning("Serial port not open, cannot send velocity command")
                return
            try:
                self._ser.write(cmd)
                logger.debug(f"Serial write: {cmd!r}")
            except serial.SerialException as e:
                logger.error(f"Serial write failed: {e}")
                self._serial_ok = False
                return
        # Update last command after successful write
        with _vel_lock:
            _last_linear  = linear
            _last_angular = angular

    def _send_raw(self, data: bytes) -> None:
        """Send raw bytes directly to serial port (e.g. state toggle '\\r')."""
        with self._ser_lock:
            if self._ser is None or not self._ser.is_open:
                logger.warning("Serial port not open, cannot send raw command")
                return
            try:
                self._ser.write(data)
                logger.debug(f"Serial write (raw): {data!r}")
            except serial.SerialException as e:
                logger.error(f"Serial raw write failed: {e}")
                self._serial_ok = False

    # ── Broadcast helper ──────────────────────────────────
    async def _broadcast(self, obj: dict) -> None:
        """Broadcast a JSON message to all connected clients."""
        msg = json.dumps(obj)
        async with self._clients_lock:
            clients = set(self._clients)
        dead = set()
        for ws in clients:
            try:
                await ws.send(msg)
            except Exception:
                dead.add(ws)
        if dead:
            async with self._clients_lock:
                self._clients -= dead

    # ── Serial reader thread ───────────────────────────────
    def _start_serial_reader(self) -> None:
        """Start daemon thread that reads status lines from Feather M4."""
        t = threading.Thread(target=self._serial_reader_thread, name="SerialReader", daemon=True)
        t.start()
        logger.info("SerialReader thread started")

    def _serial_reader_thread(self) -> None:
        """Reads serial output from Feather M4; parses S:ACTIVE / S:READY lines."""
        buf = b""
        while True:
            try:
                with self._ser_lock:
                    if self._ser is None or not self._ser.is_open:
                        buf = b""
                        time.sleep(0.1)
                        continue
                    n = self._ser.in_waiting
                    chunk = self._ser.read(n) if n > 0 else b""
            except serial.SerialException as e:
                logger.error(f"SerialReader: read error: {e}")
                buf = b""
                time.sleep(0.1)
                continue

            if chunk:
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    self._handle_serial_line(line.strip())
            else:
                time.sleep(0.01)

    def _handle_serial_line(self, line: bytes) -> None:
        """Process a status line received from Feather M4."""
        if line == b"S:ACTIVE":
            new_state = True
        elif line == b"S:READY":
            new_state = False
        else:
            return  # ignore unrecognised serial output (e.g. debug prints)
        if self._auto_active == new_state:
            return  # state unchanged, skip broadcast
        self._auto_active = new_state
        logger.info(f"SerialReader: firmware state -> {'ACTIVE' if new_state else 'READY'}")
        if self._loop is not None:
            asyncio.run_coroutine_threadsafe(
                self._broadcast({"type": "state_status", "active": new_state}),
                self._loop,
            )

    # ── WebSocket handler ─────────────────────────────────
    async def _ws_handler(self, websocket) -> None:
        async with self._clients_lock:
            self._clients.add(websocket)
        logger.info(f"WebSocket client connected: {websocket.remote_address}")
        # Push current AUTO state to new client so page refresh does not cause stale UI
        try:
            await websocket.send(json.dumps({"type": "state_status", "active": self._auto_active}))
        except Exception as e:
            logger.warning(f"WebSocket: failed to send initial state_status: {e}")
        try:
            async for raw in websocket:
                try:
                    msg = json.loads(raw)
                except json.JSONDecodeError:
                    logger.warning(f"WebSocket: invalid JSON: {raw!r}")
                    continue

                msg_type = msg.get("type")

                if msg_type == "heartbeat":
                    self._last_heartbeat = time.time()

                elif msg_type == "joystick":
                    self._last_heartbeat = time.time()  # joystick messages also reset watchdog
                    # 自动导航中忽略摇杆
                    nav_active = (
                        self._nav_engine is not None
                        and self._nav_engine.get_status().get("state") == "navigating"
                    )
                    if not nav_active:
                        try:
                            linear  = float(msg.get("linear",  0.0))
                            angular = float(msg.get("angular", 0.0))
                            # Clamp to configured velocity limits
                            linear  = max(-MAX_LINEAR_VEL,  min(MAX_LINEAR_VEL,  linear))
                            angular = max(-MAX_ANGULAR_VEL, min(MAX_ANGULAR_VEL, angular))
                            self._send_velocity(linear, angular)
                        except (TypeError, ValueError) as e:
                            logger.warning(f"WebSocket: malformed joystick message: {e}")

                elif msg_type == "toggle_state":
                    self._last_heartbeat = time.time()
                    self._send_raw(b"\r")
                    # _auto_active is updated by the serial reader thread from firmware reply, not here
                    logger.info("WebSocket: state toggle command sent (\\r), awaiting firmware confirmation")

                elif msg_type == "toggle_record":
                    await self._handle_toggle_record()

                elif msg_type == "upload_waypoints":
                    await self._handle_upload_waypoints(msg)

                elif msg_type == "nav_start":
                    await self._handle_nav_start()

                elif msg_type == "nav_stop":
                    await self._handle_nav_stop()

                elif msg_type == "nav_mode":
                    await self._handle_nav_mode(msg)

                elif msg_type == "filter_mode":
                    await self._handle_filter_mode(msg)

        except websockets.exceptions.ConnectionClosedError:
            pass
        except Exception as e:
            logger.error(f"WebSocket handler error: {e}")
        finally:
            async with self._clients_lock:
                self._clients.discard(websocket)
            logger.info(f"WebSocket client disconnected: {websocket.remote_address}")
            # Send emergency stop immediately on disconnect
            self._send_velocity(0.0, 0.0)

    # ── Toggle record handler ─────────────────────────────
    async def _handle_toggle_record(self) -> None:
        """Start or stop CSV recording and broadcast status to all clients."""
        if _data_recorder is None:
            return
        if _data_recorder.is_recording:
            _data_recorder.stop()
            msg = json.dumps({
                "type": "record_status",
                "recording": False,
                "filename": "",
            })
            logger.info("DataRecorder: stopped via WebSocket toggle")
        else:
            try:
                filename = _data_recorder.start()
                msg = json.dumps({
                    "type": "record_status",
                    "recording": True,
                    "filename": filename,
                })
                logger.info(f"DataRecorder: started via WebSocket toggle → {filename}")
            except OSError:
                msg = json.dumps({
                    "type": "record_status",
                    "recording": False,
                    "filename": "",
                })

        async with self._clients_lock:
            clients = set(self._clients)
        for ws in clients:
            try:
                await ws.send(msg)
            except Exception:
                pass

    # ── Navigation handlers ───────────────────────────────
    async def _handle_upload_waypoints(self, msg: dict) -> None:
        """处理客户端上传的 CSV 航点文本。"""
        if self._nav_engine is None:
            return
        csv_text = msg.get("csv", "")
        if not isinstance(csv_text, str) or not csv_text.strip():
            logger.warning("WebSocket: upload_waypoints: CSV 为空")
            await self._broadcast({"type": "waypoints_loaded", "count": 0, "error": "empty CSV"})
            return
        try:
            count = self._nav_engine.load_waypoints(csv_text)
            await self._broadcast({"type": "waypoints_loaded", "count": count})
            logger.info(f"WebSocket: 航点已加载，共 {count} 个")
        except Exception as e:
            logger.error(f"WebSocket: 航点加载失败: {e}")
            await self._broadcast({"type": "waypoints_loaded", "count": 0, "error": str(e)})

    async def _handle_nav_start(self) -> None:
        """处理导航开始指令。"""
        if self._nav_engine is None:
            return
        ok = self._nav_engine.start()
        if not ok:
            status = self._nav_engine.get_status()
            await self._broadcast({
                "type":  "nav_status",
                "error": "无法启动导航（无航点或 GPS 信号不足）",
                **status,
            })
        else:
            await self._broadcast(self._nav_engine.get_status())

    async def _handle_nav_stop(self) -> None:
        """处理导航停止指令。"""
        if self._nav_engine is None:
            return
        self._nav_engine.stop()
        await self._broadcast(self._nav_engine.get_status())

    async def _handle_nav_mode(self, msg: dict) -> None:
        """切换导航算法模式（p2p / pure_pursuit）。"""
        if self._nav_engine is None:
            return
        mode_str = msg.get("mode", "")
        try:
            mode = NavMode(mode_str)
            self._nav_engine.set_nav_mode(mode)
            await self._broadcast(self._nav_engine.get_status())
        except ValueError:
            logger.warning(f"WebSocket: 未知导航模式: {mode_str!r}")

    async def _handle_filter_mode(self, msg: dict) -> None:
        """切换 GPS 滤波器模式（moving_avg / kalman）。"""
        if self._nav_engine is None:
            return
        mode_str = msg.get("mode", "")
        try:
            mode = FilterMode(mode_str)
            self._nav_engine.set_filter_mode(mode)
            await self._broadcast(self._nav_engine.get_status())
        except ValueError:
            logger.warning(f"WebSocket: 未知滤波器模式: {mode_str!r}")

    # ── IMU broadcast loop (20 Hz) ────────────────────────
    async def _imu_broadcast_loop(self) -> None:
        while True:
            await asyncio.sleep(0.05)  # 20 Hz
            with _imu_mod.imu_lock:
                data = dict(_imu_mod.imu_data)
            msg = json.dumps({
                "type": "imu",
                "ts":    data["ts"],
                "accel": data["accel"],
                "gyro":  data["gyro"],
                "compass": data["compass"],
            })
            async with self._clients_lock:
                clients = set(self._clients)
            if clients:
                dead = set()
                for ws in clients:
                    try:
                        await ws.send(msg)
                    except Exception:
                        dead.add(ws)
                if dead:
                    async with self._clients_lock:
                        self._clients -= dead
            # 导航引擎 IMU 回调（20 Hz 驱动控制循环）
            if self._nav_engine is not None:
                self._nav_engine.on_imu(data)

    # ── Watchdog loop ─────────────────────────────────────
    async def _watchdog_loop(self) -> None:
        while True:
            await asyncio.sleep(0.5)
            elapsed = time.time() - self._last_heartbeat
            if elapsed > WATCHDOG_TIMEOUT:
                logger.warning(f"Watchdog triggered! No heartbeat for {elapsed:.1f}s — sending emergency stop")
                self._send_velocity(0.0, 0.0)
                # On watchdog trigger, send \r to flip firmware back to READY;
                # _auto_active is updated by the serial reader thread once firmware replies S:READY
                if self._auto_active:
                    self._send_raw(b"\r")  # firmware replies S:READY; serial reader handles state + broadcast
                    logger.info("Watchdog: sent \\r to reset AUTO state, awaiting firmware confirmation")
                # Reset timer to avoid flooding logs with repeated stop commands
                self._last_heartbeat = time.time()

    # ── RTK broadcast loop (1 Hz) ─────────────────────────
    async def _rtk_broadcast_loop(self) -> None:
        while True:
            await asyncio.sleep(1.0)  # 1 Hz — GPS updates ~1 Hz
            if _rtk_reader is None:
                continue
            snap = _rtk_reader.get_data()
            msg = json.dumps({
                "type":        "rtk",
                "available":   _rtk_reader.is_available,
                "lat":         snap["lat"],
                "lon":         snap["lon"],
                "alt":         snap["alt"],
                "fix_quality": snap["fix_quality"],
                "num_sats":    snap["num_sats"],
                "hdop":        snap["hdop"],
                "speed_knots": snap["speed_knots"],
                "track_deg":   snap["track_deg"],
            })
            async with self._clients_lock:
                clients = set(self._clients)
            if clients:
                dead = set()
                for ws in clients:
                    try:
                        await ws.send(msg)
                    except Exception:
                        dead.add(ws)
                if dead:
                    async with self._clients_lock:
                        self._clients -= dead
            # 导航引擎 RTK 回调（1 Hz 更新 GPS 滤波器）
            if self._nav_engine is not None:
                self._nav_engine.on_rtk(snap)

    # ── Data record loop (5 Hz) ───────────────────────────
    async def _data_record_loop(self) -> None:
        while True:
            await asyncio.sleep(0.2)  # 5 Hz
            if _data_recorder is None or not _data_recorder.is_recording:
                continue
            with _imu_mod.imu_lock:
                imu_snap = dict(_imu_mod.imu_data)
            rtk_snap = _rtk_reader.get_data() if _rtk_reader is not None else {}
            with _vel_lock:
                linear  = _last_linear
                angular = _last_angular
            _data_recorder.record(imu_snap, rtk_snap, linear, angular)

    # ── Status broadcast loop (low frequency) ─────────────
    async def _status_broadcast_loop(self) -> None:
        while True:
            await asyncio.sleep(2.0)
            rtk_ok    = _rtk_reader.is_available if _rtk_reader is not None else False
            recording = _data_recorder.is_recording if _data_recorder is not None else False
            msg = json.dumps({
                "type":       "status",
                "serial_ok":  self._serial_ok,
                "imu_ok":     _imu_mod.imu_available,
                "rtk_ok":     rtk_ok,
                "recording":  recording,
                "message":    "OK" if (self._serial_ok and _imu_mod.imu_available) else "DEGRADED",
            })
            async with self._clients_lock:
                clients = set(self._clients)
            for ws in clients:
                try:
                    await ws.send(msg)
                except Exception:
                    pass

    # ── Main entry ────────────────────────────────────────
    async def serve(self) -> None:
        self._loop = asyncio.get_running_loop()
        self._last_heartbeat = time.time()
        self._start_serial_reader()  # start daemon thread to read firmware state reports

        # 初始化导航引擎
        self._nav_engine = NavigationEngine(
            send_velocity_fn=self._send_velocity,
            broadcast_fn=self._broadcast,
            loop=self._loop,
        )
        logger.info("NavigationEngine initialized")

        async with websockets.serve(
            self._ws_handler,
            "0.0.0.0",
            WEB_WS_PORT,
            ping_interval=20,
            ping_timeout=10,
        ):
            logger.info(f"WebSocket server started: ws://0.0.0.0:{WEB_WS_PORT}/")
            await asyncio.gather(
                self._imu_broadcast_loop(),
                self._watchdog_loop(),
                self._status_broadcast_loop(),
                self._rtk_broadcast_loop(),
                self._data_record_loop(),
            )


# ── Entry point ───────────────────────────────────────────
def main() -> None:
    global _rtk_reader, _data_recorder

    logger.info("=" * 50)
    logger.info("Web Joystick Controller starting...")
    logger.info(f"  HTTP port : {WEB_HTTP_PORT}")
    logger.info(f"  WS   port : {WEB_WS_PORT}")
    logger.info(f"  Serial    : {FEATHER_PORT}")
    logger.info(f"  Max vel   : linear={MAX_LINEAR_VEL} m/s, angular={MAX_ANGULAR_VEL} rad/s")
    logger.info(f"  Watchdog  : {WATCHDOG_TIMEOUT}s")
    logger.info(f"  RTK GPS   : {'enabled (' + RTK_PORT + ')' if RTK_ENABLED else 'disabled'}")
    logger.info(f"  Data log  : {DATA_LOG_DIR}/")
    logger.info("=" * 50)

    controller = WebController()
    controller.open_serial()

    # Start HTTP static file server (daemon thread)
    _start_http_server()

    # Start IMU reader thread (daemon thread)
    imu_reader = IMUReader()
    imu_reader.start()

    # Start RTK reader thread (daemon thread)
    if RTK_ENABLED:
        _rtk_reader = RTKReader()
        _rtk_reader.start()
        logger.info(f"RTKReader started: {RTK_PORT} @ {RTK_BAUD} baud")
    else:
        logger.info("RTKReader disabled (RTK_ENABLED=0)")

    # Initialize data recorder
    _data_recorder = DataRecorder(DATA_LOG_DIR)
    logger.info(f"DataRecorder initialized: log_dir={DATA_LOG_DIR}")

    # Resolve local IP for user-facing access hint
    try:
        import socket
        hostname = socket.gethostname()
        local_ip = socket.gethostbyname(hostname)
    except Exception:
        local_ip = "localhost"

    logger.info(f"Open on phone: http://{local_ip}:{WEB_HTTP_PORT}/")

    try:
        asyncio.run(controller.serve())
    except KeyboardInterrupt:
        logger.info("Interrupted by user, shutting down...")
    finally:
        controller.close_serial()
        logger.info("Web Controller stopped")


if __name__ == "__main__":
    main()
