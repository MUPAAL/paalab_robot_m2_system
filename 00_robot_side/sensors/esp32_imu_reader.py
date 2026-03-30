"""
ESP32 IMU Reader — BNO085 via serial UART

Serial protocol (ESP32 firmware output):
    Format : JSON object per line
    Example: {"r":[qi,qj,qk,qr,rot_acc],"a":[ax,ay,az],"w":[wx,wy,wz],"c":cal_status}
    Fields :
        r  rotation-vector quaternion + accuracy
        a  accelerometer [x,y,z]
        w  gyroscope [x,y,z]
        c  calibration status 0-3

Public API (aligned with sensors/imu_reader.py):
    ESP32IMUReader(threading.Thread, daemon=True)
        .get_data()    -> dict  — thread-safe latest IMU snapshot
        .is_available  -> bool  — True once first valid line is parsed
"""

import json
import logging
import math
import threading
import time

import serial

from config import (
    COMPASS_MIN_ACCURACY,
    COMPASS_OFFSET_DEG,
    ESP32_IMU_PORT,
    ESP32_IMU_BAUD,
)

logger = logging.getLogger(__name__)

# ── Default snapshot (mirrors imu_reader.py structure) ───────────────────────
_DEFAULT_DATA: dict = {
    "accel":   {"x": 0.0, "y": 0.0, "z": 0.0},
    "gyro":    {"x": 0.0, "y": 0.0, "z": 0.0},
    "compass": {
        "bearing":    0.0,
        "cardinal":   "N",
        "calibrated": False,
        "accuracy":   0,
        # no quaternion from serial; fill with identity so callers never KeyError
        "quat":       {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
    },
    "ts": 0.0,
}

_CARDINALS = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]


def _bearing_to_cardinal(bearing: float) -> str:
    return _CARDINALS[int((bearing + 22.5) / 45.0) % 8]


def _parse_line(raw: str) -> dict | None:
    """Parse one serial line. Returns decoded JSON object or None."""
    try:
        data = json.loads(raw)
    except json.JSONDecodeError:
        return None
    return data if isinstance(data, dict) else None


class ESP32IMUReader(threading.Thread):
    """Daemon thread: reads BNO085 data from ESP32 over serial and exposes
    the latest snapshot via get_data() / is_available.

    Drop-in replacement for sensors.imu_reader.IMUReader.
    """

    def __init__(self) -> None:
        super().__init__(name="ESP32IMUReader", daemon=True)
        self._lock = threading.Lock()
        self._data: dict = dict(_DEFAULT_DATA)
        self._available = False

    @property
    def is_available(self) -> bool:
        """True once the first valid IMU packet has been received."""
        return self._available

    def get_data(self) -> dict:
        """Return a shallow copy of the latest IMU snapshot (thread-safe)."""
        with self._lock:
            return dict(self._data)

    # ── Thread entry ──────────────────────────────────────────────────────────
    def run(self) -> None:
        logger.info(f"ESP32IMUReader: opening {ESP32_IMU_PORT} @ {ESP32_IMU_BAUD} baud")
        try:
            ser = serial.Serial(ESP32_IMU_PORT, ESP32_IMU_BAUD, timeout=2.0)
        except serial.SerialException as e:
            logger.error(f"ESP32IMUReader: failed to open serial port [{ESP32_IMU_PORT}]: {e}")
            return  # is_available stays False → graceful degradation

        logger.info(f"ESP32IMUReader: serial port opened: {ser.name}")
        buf = b""

        while True:
            try:
                chunk = ser.read(ser.in_waiting or 1)
            except serial.SerialException as e:
                logger.error(f"ESP32IMUReader: serial read error: {e}")
                time.sleep(0.1)
                continue

            if not chunk:
                continue

            buf += chunk
            while b"\n" in buf:
                raw_bytes, buf = buf.split(b"\n", 1)
                raw_str = raw_bytes.decode("ascii", errors="replace").strip()
                if not raw_str or raw_str.startswith("#"):
                    # ignore empty lines and firmware diagnostic lines
                    if raw_str.startswith("#"):
                        logger.debug(f"ESP32IMUReader: [fw] {raw_str}")
                    continue
                self._process_line(raw_str)

    def _process_line(self, raw: str) -> None:
        result = None
        try:
            result = _parse_line(raw)
        except Exception as e:
            logger.warning(f"ESP32IMUReader: parse error [{e}] raw={raw!r}")
            return

        if result is None:
            logger.warning(f"ESP32IMUReader: unexpected format (not 3 or 4 fields): {raw!r}")
            return

        try:
            rot = result["r"]
            accel = result["a"]
            gyro = result["w"]
            accuracy = int(result.get("c", 0))
            if not isinstance(rot, list) or len(rot) != 5:
                raise ValueError("r vector invalid")
            if not isinstance(accel, list) or len(accel) != 3:
                raise ValueError("a vector invalid")
            if not isinstance(gyro, list) or len(gyro) != 3:
                raise ValueError("w vector invalid")

            rot = [float(v) for v in rot]
            accel = [float(v) for v in accel]
            gyro = [float(v) for v in gyro]
        except (KeyError, ValueError, TypeError) as e:
            logger.warning(f"ESP32IMUReader: malformed packet [{e}] raw={raw!r}")
            return

        qi, qj, qk, qr, _rot_acc = rot
        yaw_rad = math.atan2(2.0 * (qr * qk + qi * qj), 1.0 - 2.0 * (qj * qj + qk * qk))
        bearing = (90.0 - math.degrees(yaw_rad)) % 360.0
        cardinal = _bearing_to_cardinal(bearing)

        # accuracy = -1 means firmware does not report it; treat as uncalibrated
        eff_accuracy = accuracy if accuracy >= 0 else 0
        calibrated = eff_accuracy >= COMPASS_MIN_ACCURACY

        snap: dict = {
            "accel": {"x": accel[0], "y": accel[1], "z": accel[2]},
            "gyro":  {"x": gyro[0], "y": gyro[1], "z": gyro[2]},
            "compass": {
                "bearing":    bearing,
                "cardinal":   cardinal,
                "calibrated": calibrated,
                "accuracy":   eff_accuracy,
                "quat":       {"w": qr, "x": qi, "y": qj, "z": qk},
            },
            "ts": time.time(),
        }

        with self._lock:
            self._data = snap
            if not self._available:
                self._available = True
                logger.info(
                    f"ESP32IMUReader: first packet received — "
                    f"bearing={bearing:.1f}°  accuracy={accuracy}"
                )
