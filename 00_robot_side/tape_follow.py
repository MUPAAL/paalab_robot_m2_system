import json
import logging
import signal
import socket
import sys
import threading
import time
from pathlib import Path

from config import FEATHER_PORT, KEY_REPEAT_INTERVAL
from core.serial_writer import SerialWriter

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

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

_stop = False


def _signal_handler(signum, frame):
    global _stop
    logger.info(f"Signal {signum} received, stopping...")
    _stop = True


signal.signal(signal.SIGINT, _signal_handler)
signal.signal(signal.SIGTERM, _signal_handler)


class TapeFollower:
    def __init__(self) -> None:
        self._serial = SerialWriter(port=FEATHER_PORT)
        self._running = False
        self._current_cmd = " "
        self._cmd_lock = threading.Lock()
        self._repeat_thread: threading.Thread | None = None
        self._last_msg_time = 0.0

        # ── Debounce / hysteresis state ────────────────────────
        # How many consecutive frames a candidate command must be
        # seen before it is actually committed.
        self._DEBOUNCE_FRAMES = 3

        # Dead zone: error must exceed ENTER to start turning,
        # but can drop to EXIT before we stop turning (hysteresis).
        self._DEAD_ZONE_ENTER = 80   # px — start turning
        self._DEAD_ZONE_EXIT  = 40   # px — stop turning (narrower = stops sooner)

        self._candidate_cmd   = " "   # what the latest frame wants
        self._candidate_count = 0     # consecutive frames requesting that cmd
        self._committed_cmd   = " "   # the last command actually committed

    def _send(self, char: str) -> None:
        try:
            self._serial.write_command(char)
        except Exception as e:
            logger.error(f"Serial write error: {e}")
            self._running = False

    def _set_command(self, cmd: str) -> None:
        """Debounced command setter.

        A new command only replaces the current one after it has been
        requested _DEBOUNCE_FRAMES times in a row.  The stop command (' ')
        is always committed immediately so we never get stuck driving.
        """
        with self._cmd_lock:
            if cmd == " ":
                # Stop is always immediate — safety first.
                if self._current_cmd != " ":
                    logger.info(f"Command changed: {repr(self._current_cmd)} -> ' ' (immediate stop)")
                self._current_cmd   = " "
                self._committed_cmd = " "
                self._candidate_cmd   = " "
                self._candidate_count = 0
                return

            if cmd == self._candidate_cmd:
                self._candidate_count += 1
            else:
                # New candidate — reset counter.
                self._candidate_cmd   = cmd
                self._candidate_count = 1

            if self._candidate_count >= self._DEBOUNCE_FRAMES:
                if cmd != self._current_cmd:
                    logger.info(
                        f"Command changed: {repr(self._current_cmd)} -> {repr(cmd)} "
                        f"(after {self._candidate_count} frames)"
                    )
                    self._current_cmd   = cmd
                    self._committed_cmd = cmd

    def _repeat_loop(self) -> None:
        logger.info(f"Command repeat thread started, rate: {1.0 / KEY_REPEAT_INTERVAL:.0f}Hz")
        while self._running and not _stop:
            with self._cmd_lock:
                cmd = self._current_cmd
            self._send(cmd)
            time.sleep(KEY_REPEAT_INTERVAL)

    def _decide_command(self, msg: dict) -> str:
        found = msg.get("found", False)
        error = msg.get("error")
        area  = msg.get("area", 0)

        if not found or error is None:
            return " "

        if area < 40:
            return " "

        # Hysteresis: use a wider band to START turning, narrower to STOP.
        currently_turning = self._committed_cmd in ("a", "d")

        if currently_turning:
            # We are already turning — keep turning until error is small.
            threshold = self._DEAD_ZONE_EXIT
        else:
            # We are going straight — only start turning on a larger error.
            threshold = self._DEAD_ZONE_ENTER

        if error < -threshold:
            return "a"
        elif error > threshold:
            return "d"
        else:
            return "w"

    def run(self) -> None:
        global _stop

        self._serial.open()
        self._running = True

        self._repeat_thread = threading.Thread(
            target=self._repeat_loop,
            daemon=True,
            name="cmd_repeat",
        )
        self._repeat_thread.start()

        logger.info(f"Serial port: {FEATHER_PORT}")

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, UDP_PORT))
        sock.settimeout(0.2)

        logger.info(f"Listening for target data on UDP {UDP_IP}:{UDP_PORT}")

        try:
            while self._running and not _stop:
                try:
                    data, _addr = sock.recvfrom(4096)
                    msg = json.loads(data.decode("utf-8"))
                    self._last_msg_time = time.time()

                    cmd = self._decide_command(msg)
                    self._set_command(cmd)

                except socket.timeout:
                    if time.time() - self._last_msg_time > 0.5:
                        self._set_command(" ")

                except Exception as e:
                    logger.error(f"UDP receive/parse error: {e}")

        finally:
            sock.close()
            self.shutdown()

    def shutdown(self) -> None:
        logger.info("Shutting down tape follower...")
        self._running = False

        if self._serial.is_open:
            try:
                self._serial.emergency_stop()
            except Exception as e:
                logger.error(f"Emergency stop serial error: {e}")
            logger.info("Final emergency stop sent")

        if self._repeat_thread and self._repeat_thread.is_alive():
            self._repeat_thread.join(timeout=2.0)

        self._serial.close()
        logger.info("Tape follower stopped")


def main() -> None:
    follower = TapeFollower()
    follower.run()


if __name__ == "__main__":
    main()