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
import math
import os
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
from rtk_reader    import RTKReader
from data_recorder import DataRecorder

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

# ── IMU data store (thread-safe via lock) ─────────────────
_imu_lock = threading.Lock()
_imu_data: dict = {
    "accel": {"x": 0.0, "y": 0.0, "z": 0.0},
    "gyro":  {"x": 0.0, "y": 0.0, "z": 0.0},
    "compass": {
        "bearing": 0.0, "cardinal": "N",
        "calibrated": False,
        "accuracy": 0,
        "quat": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
    },
    "ts": 0.0,
}
_imu_available = False  # True once depthai pipeline is running

# ── RTK reader (global, started in main()) ────────────────
_rtk_reader: RTKReader | None = None

# ── Data recorder (global, started in main()) ─────────────
_data_recorder: DataRecorder | None = None

# ── Last velocity command (protected by lock) ─────────────
_vel_lock   = threading.Lock()
_last_linear:  float = 0.0
_last_angular: float = 0.0


# ── Quaternion → compass bearing ──────────────────────────
def quaternion_to_compass(real: float, i: float, j: float, k: float) -> tuple[float, str]:
    """Convert BNO085 ROTATION_VECTOR quaternion to compass bearing [0, 360).

    0 = magnetic north, clockwise positive.
    Coordinate system selectable via COORD_SYSTEM env var (default: NED).
    """
    coord = os.environ.get("COORD_SYSTEM", "NED").upper()
    yaw_rad = math.atan2(2 * (real * k + i * j), 1 - 2 * (j * j + k * k))
    if coord == "ENU":
        # ENU: yaw is measured counter-clockwise from East; convert to clockwise-from-North bearing
        bearing = (90.0 - math.degrees(yaw_rad)) % 360.0
    else:
        # NED: yaw is already a clockwise-from-North bearing
        bearing = math.degrees(yaw_rad) % 360.0
    cardinals = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
    cardinal  = cardinals[int((bearing + 22.5) / 45.0) % 8]
    return bearing, cardinal


# ── IMU reader thread (depthai OAK-D) ────────────────────
class IMUReader(threading.Thread):
    """Daemon thread: continuously reads IMU packets from OAK-D and updates _imu_data."""

    def __init__(self) -> None:
        super().__init__(name="IMUReader", daemon=True)

    def run(self) -> None:
        global _imu_available
        try:
            import depthai as dai
        except ImportError:
            logger.warning("depthai not installed — IMU unavailable, HUD will show zeros")
            return

        try:
            with dai.Pipeline() as pipeline:
                imu_node = pipeline.create(dai.node.IMU)
                imu_node.enableIMUSensor(dai.IMUSensor.LINEAR_ACCELERATION, 400)
                imu_node.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 400)
                imu_node.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 100)
                imu_node.setBatchReportThreshold(1)
                imu_node.setMaxBatchReports(10)

                imu_queue = imu_node.out.createOutputQueue(maxSize=50, blocking=False)
                pipeline.start()
                _imu_available = True
                logger.info("IMUReader: depthai IMU pipeline started")

                while pipeline.isRunning():
                    try:
                        imu_data_pkt = imu_queue.get()
                        if imu_data_pkt is None:
                            continue
                        for pkt in imu_data_pkt.packets:
                            self._process_packet(pkt)
                    except Exception as e:
                        logger.error(f"IMUReader: failed to read packet: {e}")

        except Exception as e:
            logger.error(f"IMUReader: depthai pipeline failed to start: {e}")
            _imu_available = False

    def _process_packet(self, pkt) -> None:
        global _imu_data
        try:
            accel = pkt.acceleroMeter
            gyro  = pkt.gyroscope
            rot   = pkt.rotationVector

            w, xi, yj, zk = rot.real, rot.i, rot.j, rot.k
            try:
                accuracy = int(rot.accuracy)  # 0-3: BNO085 calibration accuracy
            except (AttributeError, TypeError, ValueError):
                # Fallback: infer from all-zero quaternion check
                accuracy = 0 if (w == 0.0 and xi == 0.0 and yj == 0.0 and zk == 0.0) else 3
            calibrated = accuracy >= 2
            bearing, cardinal = quaternion_to_compass(w, xi, yj, zk) if calibrated else (0.0, "N")

            with _imu_lock:
                _imu_data = {
                    "accel": {"x": accel.x, "y": accel.y, "z": accel.z},
                    "gyro":  {"x": gyro.x,  "y": gyro.y,  "z": gyro.z},
                    "compass": {
                        "bearing": bearing,
                        "cardinal": cardinal,
                        "calibrated": calibrated,
                        "accuracy": accuracy,
                        "quat": {"w": w, "x": xi, "y": yj, "z": zk},
                    },
                    "ts": time.time(),
                }
        except Exception as e:
            logger.error(f"IMUReader: packet processing error: {e}")


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

    # ── IMU broadcast loop (20 Hz) ────────────────────────
    async def _imu_broadcast_loop(self) -> None:
        while True:
            await asyncio.sleep(0.05)  # 20 Hz
            with _imu_lock:
                data = dict(_imu_data)
            msg = json.dumps({
                "type": "imu",
                "ts":    data["ts"],
                "accel": data["accel"],
                "gyro":  data["gyro"],
                "compass": data["compass"],
            })
            async with self._clients_lock:
                clients = set(self._clients)
            if not clients:
                continue
            dead = set()
            for ws in clients:
                try:
                    await ws.send(msg)
                except Exception:
                    dead.add(ws)
            if dead:
                async with self._clients_lock:
                    self._clients -= dead

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
            if not clients:
                continue
            dead = set()
            for ws in clients:
                try:
                    await ws.send(msg)
                except Exception:
                    dead.add(ws)
            if dead:
                async with self._clients_lock:
                    self._clients -= dead

    # ── Data record loop (5 Hz) ───────────────────────────
    async def _data_record_loop(self) -> None:
        while True:
            await asyncio.sleep(0.2)  # 5 Hz
            if _data_recorder is None or not _data_recorder.is_recording:
                continue
            with _imu_lock:
                imu_snap = dict(_imu_data)
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
                "imu_ok":     _imu_available,
                "rtk_ok":     rtk_ok,
                "recording":  recording,
                "message":    "OK" if (self._serial_ok and _imu_available) else "DEGRADED",
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
