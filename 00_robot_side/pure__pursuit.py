import asyncio
import math
import json
import traceback
import logging
import signal
import time
from pathlib import Path
import serial

from sensors.rtk_reader import RTKReader
from sensors.esp32_imu_reader import ESP32IMUReader as IMUReader

from navigation.waypoint import Waypoint, WaypointManager
from navigation.nav_engine import NavigationEngine, NavMode

from config import FEATHER_PORT, SERIAL_BAUD, SERIAL_TIMEOUT, KEY_REPEAT_INTERVAL, COMPASS_MIN_ACCURACY

# ── logging ───────────────────────────────────────────────────────────────────
_py_name = Path(__file__).stem
Path("log").mkdir(exist_ok=True)
logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler(f"log/{_py_name}.log", encoding="utf-8"),
        logging.StreamHandler(),
    ],
)
logger = logging.getLogger(__name__)

_stop = False

def _signal_handler(signum, frame):
    global _stop
    logger.info(f"Signal {signum} received, stopping...")
    _stop = True


signal.signal(signal.SIGINT, _signal_handler)
signal.signal(signal.SIGTERM, _signal_handler)


# ── Navigation waypoints (replace with actual route) ───────────────────────────
TARGETS = [(38.9412161, -92.3188177), (38.9412309, -92.3186055)]
WAYPOINT_RADIUS = 2.0  # meters
waypoints = [
    Waypoint(id=i, lat=lat, lon=lon, tolerance_m=WAYPOINT_RADIUS, max_speed=0.6)
    for i, (lat, lon) in enumerate(TARGETS)
]
waypoints_csv = "id,lat,lon,tolerance_m,max_speed\n" + "\n".join(
    f"{wp.id},{wp.lat},{wp.lon},{wp.tolerance_m},{wp.max_speed}" for wp in waypoints
)

# Start sensor readers at module import time
imu_reader = IMUReader()
rtk_reader = RTKReader()
imu_reader.start()
rtk_reader.start()


class PurePursuitCommandDriver:
    def __init__(self) -> None:
        self._ser = None
        self._running = False

        self._lock = __import__('threading').Lock()
        self._repeat_thread = None

        self._current_linear = 0.0
        self._current_angular = 0.0
        self._last_msg_time = 0.0

        self._nav_engine = NavigationEngine(
            send_velocity_fn=self._on_nav_velocity,
            broadcast_fn=self._broadcast_noop,
            loop=asyncio.get_event_loop(),
        )

    async def _broadcast_noop(self, _status: dict) -> None:
        """No-op broadcast; pure_pursuit uses direct serial velocity commands."""
        pass

    def _open_serial(self) -> None:
        self._ser = serial.Serial(FEATHER_PORT, SERIAL_BAUD, timeout=SERIAL_TIMEOUT)
        logger.info(f"Serial port opened: {FEATHER_PORT} @ {SERIAL_BAUD} baud")

    def _close_serial(self) -> None:
        if self._ser is not None and self._ser.is_open:
            self._ser.close()
            logger.info("Serial port closed")

    def _send_velocity(self, linear: float, angular: float) -> None:
        if self._ser is None or not self._ser.is_open:
            logger.warning("Serial port not open, cannot send velocity")
            return

        cmd = f"V{linear:.2f},{angular:.2f}\n"
        logger.info(f"Sending command: {cmd.strip()}")
        cmd = cmd.encode()
        try:
            self._ser.write(cmd)
        except Exception as e:
            logger.error(f"Serial write failed: {e}")
            self._running = False

    def _send_command(self, linear: float, angular: float) -> None:
        # Prefer direct V command for precision; preserve safety stop if needed.
        self._send_velocity(linear, angular)

    def _on_nav_velocity(self, linear: float, angular: float) -> None:
        with self._lock:
            self._current_linear = linear
            self._current_angular = angular
            self._last_msg_time = time.time()
        logger.debug(f"Nav velocity set: linear={linear:.2f}, angular={angular:.2f}")

    def _repeat_loop(self) -> None:
        logger.info(f"Velocity repeat thread started, rate: {1.0 / KEY_REPEAT_INTERVAL:.0f}Hz")
        while self._running and not _stop:
            with self._lock:
                now = time.time()
                if now - self._last_msg_time > 0.5:
                    linear = 0.0
                    angular = 0.0
                else:
                    linear = self._current_linear
                    angular = self._current_angular

            logger.debug(f"Sending velocity: linear={linear:.2f}, angular={angular:.2f}")
            self._send_command(linear, angular)
            time.sleep(KEY_REPEAT_INTERVAL)

    def _initialize_nav(self) -> None:
        self._nav_engine.load_waypoints(waypoints_csv)
        self._nav_engine.set_nav_mode(NavMode.PURE_PURSUIT)
        started = self._nav_engine.start(force=True)
        if not started:
            logger.error("NavigationEngine failed to start")
            raise RuntimeError("NavigationEngine start failed")
        logger.info("NavigationEngine started in PURE_PURSUIT mode")

    def run(self) -> None:
        self._open_serial()
        self._running = True

        self._repeat_thread = __import__('threading').Thread(
            target=self._repeat_loop,
            daemon=True,
            name="cmd_repeat",
        )
        self._repeat_thread.start()

        self._initialize_nav()

        try:
            while self._running and not _stop:
                pos = rtk_reader.get_data()
                if pos is not None:
                    lat, lon = pos.get("lat"), pos.get("lon")
                    precision = pos.get("precision", 0.0)
                    logger.debug(f"RTK pos: lat={lat}, lon={lon}, precision={precision}")
                    if precision <= 2 and not math.isnan(precision):
                        rtk_data = {
                            "lat": lat,
                            "lon": lon,
                            "fix_quality": 4 if precision < 0.5 else (5 if precision < 1.0 else 2),
                        }
                        self._nav_engine.on_rtk(rtk_data)
                        logger.debug("Sent RTK data to nav_engine")
                    else:
                        logger.warning(f"RTK precision too low: {precision}")
                else:
                    logger.warning("No RTK data received")

                imu_data = imu_reader.get_data()
                yaw = imu_data["compass"]["bearing"]
                yaw_accuracy = imu_data["compass"]["accuracy"]
                logger.debug(f"IMU: yaw={yaw}, accuracy={yaw_accuracy}")
                if yaw is not None and yaw_accuracy is not None and yaw_accuracy >= COMPASS_MIN_ACCURACY:
                    imu_data = {
                        "compass": {
                            "bearing": yaw,
                            "accuracy": yaw_accuracy,
                            "calibrated": yaw_accuracy >= COMPASS_MIN_ACCURACY,
                        },
                        "accel": {"x": 0.0, "y": 0.0},
                    }
                    self._nav_engine.on_imu(imu_data)
                    logger.debug("Sent IMU data to nav_engine")
                else:
                    logger.warning(f"IMU data invalid: yaw={yaw}, accuracy={yaw_accuracy}")

                time.sleep(0.05)

        except Exception as e:
            logger.error(f"Pure pursuit loop error: {e}")
            traceback.print_exc()

        finally:
            self.shutdown()

    def shutdown(self) -> None:
        logger.info("Shutting down pure pursuit command driver...")
        self._running = False

        # Force safe stop command to robot (direct velocity mode)
        self._send_command(0.0, 0.0)

        if self._repeat_thread and self._repeat_thread.is_alive():
            self._repeat_thread.join(timeout=2.0)

        self._close_serial()
        logger.info("Pure pursuit driver stopped")


def main() -> None:
    driver = PurePursuitCommandDriver()
    driver.run()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        logger.error(f"Fatal error in pure pursuit: {e}\n{traceback.format_exc()}")
        raise
