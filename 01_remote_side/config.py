"""
Remote-side configuration.
All parameters can be overridden via environment variables.
"""

import os

# ── Connection configuration ───────────────────────────────
ROBOT_HOST: str = os.environ.get("ROBOT_HOST", "192.168.1.100")  # robot-side (Mac Mini) IP
ROBOT_PORT: int = int(os.environ.get("ROBOT_PORT", "9000"))

# ── Heartbeat configuration (must be well below watchdog timeout 2.0s) ────
HEARTBEAT_INTERVAL: float = float(os.environ.get("HEARTBEAT_INTERVAL", "0.5"))

# ── Key repeat interval (10 Hz) ────────────────────────────
KEY_REPEAT_INTERVAL: float = float(os.environ.get("KEY_REPEAT_INTERVAL", "0.1"))

# ── Control key mapping ────────────────────────────────────
# pynput key character -> byte sent to robot
CONTROL_KEYS: dict = {
    "w": "w",
    "s": "s",
    "a": "a",
    "d": "d",
}
STOP_CHAR: str = " "       # emergency stop character
HEARTBEAT_CHAR: str = "H"  # heartbeat character
QUIT_KEY: str = "q"        # quit key

# ── 相机流地址 ─────────────────────────────────────────────
CAM1_URL: str = os.environ.get("CAM1_URL", f"http://{ROBOT_HOST}:8080")
CAM2_URL: str = os.environ.get("CAM2_URL", f"http://{ROBOT_HOST}:8081")
