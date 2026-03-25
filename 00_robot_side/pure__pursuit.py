import asyncio
import math
import json
import traceback
import logging
import signal
import time
from pathlib import Path
import serial
import websockets

from navigation.waypoint import Waypoint, WaypointManager
from navigation.nav_engine import NavigationEngine, NavMode
from core.serial_writer import SerialWriter

from config import FEATHER_PORT, SERIAL_BAUD, SERIAL_TIMEOUT, KEY_REPEAT_INTERVAL, COMPASS_MIN_ACCURACY, WEB_WS_PORT

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


class PurePursuitCommandDriver:
    def __init__(self) -> None:
        self._serial = SerialWriter(port=FEATHER_PORT)
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

        # Asyncio loop used by thread-safe websocket dispatch (set in run_async)
        self._loop = None

        # WebSocket client for communicating with web controller
        self._ws = None
        self._ws_lock = asyncio.Lock()
        self._ws_connected = False
        self._heartbeat_task = None

    async def _broadcast_noop(self, _status: dict) -> None:
        """Broadcast navigation status to web controller if connected."""
        if self._ws_connected and self._ws:
            try:
                _status["type"] = "nav_status"
                await self._ws.send(json.dumps(_status))
                logger.debug("Sent nav status to web controller")
            except Exception as e:
                logger.warning(f"Failed to send nav status to web controller: {e}")
                self._ws_connected = False

    async def _connect_websocket(self) -> None:
        """Connect to web controller's WebSocket server."""
        try:
            uri = f"ws://localhost:{WEB_WS_PORT}"
            self._ws = await websockets.connect(uri)
            self._ws_connected = True
            logger.info(f"Connected to web controller WebSocket at {uri}")
            
            # Send initial status
            status = self._nav_engine.get_status()
            await self._broadcast_noop(status)
            
            # Start heartbeat task
            self._heartbeat_task = asyncio.create_task(self._send_heartbeat())
            
            # Start message handling task
            self._ws_message_task = asyncio.create_task(self._handle_ws_messages())
            
        except Exception as e:
            logger.warning(f"Failed to connect to web controller WebSocket: {e}")
            self._ws_connected = False

    async def _disconnect_websocket(self) -> None:
        """Disconnect from web controller's WebSocket server."""
        if self._ws_message_task:
            self._ws_message_task.cancel()
            try:
                await self._ws_message_task
            except asyncio.CancelledError:
                pass
            self._ws_message_task = None
            
        if self._heartbeat_task:
            self._heartbeat_task.cancel()
            try:
                await self._heartbeat_task
            except asyncio.CancelledError:
                pass
            self._heartbeat_task = None
            
        if self._ws:
            try:
                await self._ws.close()
            except Exception as e:
                logger.warning(f"Error closing WebSocket: {e}")
            self._ws = None
        self._ws_connected = False

    async def _send_velocity_to_ws(self, linear: float, angular: float) -> None:
        """Send velocity command to web controller for logging."""
        if self._ws_connected and self._ws:
            try:
                await self._ws.send(json.dumps({
                    "type": "velocity_command",
                    "linear": linear,
                    "angular": angular
                }))
            except Exception as e:
                logger.debug(f"Failed to send velocity command to web controller: {e}")
                self._ws_connected = False

    async def _handle_ws_messages(self) -> None:
        """Handle incoming WebSocket messages from web controller."""
        try:
            async for message in self._ws:
                data = json.loads(message)
                if data.get("type") == "nav_start":
                    started = self._nav_engine.start(force=True)
                    if started:
                        logger.info("Navigation started via WS command")
                    else:
                        logger.warning("Failed to start navigation")
                elif data.get("type") == "nav_stop":
                    self._nav_engine.stop()
                    logger.info("Navigation stopped via WS command")
                elif data.get("type") == "imu":
                    self._nav_engine.on_imu(data)
                elif data.get("type") == "rtk":
                    self._nav_engine.on_rtk(data)
                elif data.get("type") == "odom":
                    self._nav_engine.on_odometry(data.get("v", 0.0), data.get("w", 0.0))
        except Exception as e:
            logger.warning(f"WS message handling error: {e}")

    def _open_serial(self) -> None:
        self._serial.open()
        logger.info("SerialWriter opened")

        # Put the robot into auto mode for pure pursuit (same as LocalController Enter key)
        # The board firmware expects this before V commands take effect.
        try:
            # Send \r directly using SerialWriter's internal serial port
            # (Note: SerialWriter doesn't have a method for \r, so we access the port directly with locking)
            if self._serial.is_open:
                import serial as serial_module
                with self._serial._lock:
                    self._serial._ser.write(b"\r")
                logger.info("Auto-ready -> auto-active request sent")
                time.sleep(0.1)
        except Exception as e:
            logger.error(f"Failed to request auto-active state: {e}")

    def _close_serial(self) -> None:
        self._serial.close()
        logger.info("Serial port closed")

    def _send_command(self, linear: float, angular: float) -> None:
        """Send velocity command V{linear:.2f},{angular:.2f}\\n to Feather M4."""
        if not self._serial.is_open:
            logger.warning("Serial port not open, cannot send velocity")
            return

        cmd = f"V{linear:.2f},{angular:.2f}\n"
        logger.debug(f"Sending command: {cmd.strip()}")
        try:
            # Use SerialWriter's internal port with locking for custom V command format
            with self._serial._lock:
                self._serial._ser.write(cmd.encode())
        except Exception as e:
            logger.error(f"Serial write failed: {e}")
            self._running = False
        
        # Also send to web controller for logging
        if self._ws_connected and self._ws and self._loop is not None:
            try:
                asyncio.run_coroutine_threadsafe(
                    self._send_velocity_to_ws(linear, angular),
                    self._loop
                )
            except Exception as e:
                logger.debug(f"Failed to queue velocity command to web controller: {e}")
        elif self._ws_connected and self._ws:
            logger.debug("WebSocket connected but no loop set for thread-safe dispatch")

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
        # Do not start navigation here; wait for nav_start command
        logger.info("NavigationEngine initialized in PURE_PURSUIT mode (waiting for start command)")

    async def run_async(self) -> None:
        # Store the current running loop for run_coroutine_threadsafe from worker thread
        self._loop = asyncio.get_running_loop()

        await self._connect_websocket()
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
            # Wait for WebSocket messages (event-driven)
            await self._ws_message_task
        except Exception as e:
            logger.error(f"Pure pursuit error: {e}")
            traceback.print_exc()

        finally:
            await self._disconnect_websocket()
            self.shutdown()

    def run(self) -> None:
        """Synchronous wrapper for async run."""
        asyncio.run(self.run_async())

    def shutdown(self) -> None:
        logger.info("Shutting down pure pursuit command driver...")
        self._running = False

        # Force safe stop command to robot (direct velocity mode)
        time.sleep(0.1)  # Ensure stop command is sent before thread exits

        if self._repeat_thread and self._repeat_thread.is_alive():
            self._repeat_thread.join(timeout=2.0)

        # Exit auto mode back to ready state
        if self._serial.is_open:
            try:
                with self._serial._lock:
                    self._serial._ser.write(b"\r")
                logger.info("Auto-active -> auto-ready request sent")
                time.sleep(0.1)
                logger.info("Auto-active -> auto-ready request sent")
            except Exception as e:
                logger.error(f"Failed to send auto-ready state request: {e}")

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
