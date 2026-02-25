"""
Robot-side configuration.
All parameters can be overridden via environment variables.
"""

import os
import platform

# ── Serial port configuration ──────────────────────────────
def _default_serial_port() -> str:
    """Return the default serial port path based on the current OS."""
    system = platform.system()
    if system == "Darwin":   # macOS
        return "/dev/cu.usbmodem2301"
    else:                    # Linux
        return "/dev/ttyACM0"

FEATHER_PORT: str = os.environ.get("FEATHER_PORT", _default_serial_port())
SERIAL_BAUD: int = int(os.environ.get("SERIAL_BAUD", "115200"))
SERIAL_TIMEOUT: float = float(os.environ.get("SERIAL_TIMEOUT", "1.0"))

# ── TCP server configuration ───────────────────────────────
TCP_HOST: str = os.environ.get("TCP_HOST", "0.0.0.0")   # listen on all interfaces
TCP_PORT: int = int(os.environ.get("TCP_PORT", "9000"))

# ── Watchdog configuration ─────────────────────────────────
WATCHDOG_TIMEOUT: float = float(os.environ.get("WATCHDOG_TIMEOUT", "2.0"))

# ── Allowed command characters (serial whitelist) ──────────
ALLOWED_COMMANDS: set = {"w", "s", "a", "d", " "}
HEARTBEAT_CHAR: str = "H"

# ── OAK-D PoE camera addresses ────────────────────────────
CAM1_IP: str = os.environ.get("CAM1_IP", "10.95.76.10")
CAM2_IP: str = os.environ.get("CAM2_IP", "10.95.76.11")

# ── MJPEG HTTP stream ports ────────────────────────────────
CAM1_STREAM_PORT: int = int(os.environ.get("CAM1_STREAM_PORT", "8080"))
CAM2_STREAM_PORT: int = int(os.environ.get("CAM2_STREAM_PORT", "8081"))

# ── Camera parameters ─────────────────────────────────────
CAM_FPS: int       = int(os.environ.get("CAM_FPS", "30"))
CAM_WIDTH: int     = int(os.environ.get("CAM_WIDTH", "1280"))
CAM_HEIGHT: int    = int(os.environ.get("CAM_HEIGHT", "720"))
MJPEG_QUALITY: int = int(os.environ.get("MJPEG_QUALITY", "80"))  # 1-100

# ── Local preview toggle ──────────────────────────────────
LOCAL_DISPLAY: bool = os.environ.get("LOCAL_DISPLAY", "0") == "1"

# ── Key repeat interval (10 Hz) ────────────────────────────
KEY_REPEAT_INTERVAL: float = float(os.environ.get("KEY_REPEAT_INTERVAL", "0.1"))
