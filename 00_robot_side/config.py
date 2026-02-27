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
ALLOWED_COMMANDS: set = {"w", "s", "a", "d", " ", "\r"}
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

# ── Web joystick controller ports ─────────────────────────
WEB_HTTP_PORT: int = int(os.environ.get("WEB_HTTP_PORT", "8888"))
WEB_WS_PORT:   int = int(os.environ.get("WEB_WS_PORT",   "8889"))

# ── Maximum velocity limits for web joystick ──────────────
MAX_LINEAR_VEL:  float = float(os.environ.get("MAX_LINEAR_VEL",  "1.0"))
MAX_ANGULAR_VEL: float = float(os.environ.get("MAX_ANGULAR_VEL", "1.0"))

# ── RTK GPS (Emlid RS+) ────────────────────────────────────
RTK_PORT:    str   = os.environ.get("RTK_PORT",    "/dev/cu.usbmodem2403")
RTK_BAUD:    int   = int(os.environ.get("RTK_BAUD",    "9600"))
RTK_TIMEOUT: float = float(os.environ.get("RTK_TIMEOUT", "1.0"))
RTK_ENABLED: bool  = os.environ.get("RTK_ENABLED", "1") == "1"

# ── Data recording ─────────────────────────────────────────
DATA_LOG_DIR: str = os.environ.get("DATA_LOG_DIR", "data_log")

# ── Navigation ─────────────────────────────────────────────
NAV_LOOKAHEAD_M:    float = float(os.environ.get("NAV_LOOKAHEAD_M",    "2.0"))
NAV_DECEL_RADIUS_M: float = float(os.environ.get("NAV_DECEL_RADIUS_M", "3.0"))
NAV_ARRIVE_FRAMES:  int   = int(os.environ.get("NAV_ARRIVE_FRAMES",    "5"))
NAV_GPS_TIMEOUT_S:  float = float(os.environ.get("NAV_GPS_TIMEOUT_S",  "5.0"))
NAV_PID_KP:         float = float(os.environ.get("NAV_PID_KP",         "0.8"))
NAV_PID_KI:         float = float(os.environ.get("NAV_PID_KI",         "0.01"))
NAV_PID_KD:         float = float(os.environ.get("NAV_PID_KD",         "0.05"))
NAV_MA_WINDOW:      int   = int(os.environ.get("NAV_MA_WINDOW",        "10"))
