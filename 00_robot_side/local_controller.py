"""
Robot-side local keyboard controller.
Directly controls Feather M4 CAN via serial (no TCP required).

Usage:
    export FEATHER_PORT=/dev/cu.usbmodem2301   # optional override
    cd m2_system/00_robot_side
    python local_controller.py

Controls:
    w / s / a / d  - forward / backward / turn left / turn right
    space          - emergency stop (also sent automatically when all keys released)
    q              - quit

Note:
    This script holds the serial port directly.
    Do NOT run simultaneously with robot_receiver.py (serial port conflict).
    No watchdog — operator is physically present.
"""

import logging
import signal
import sys
import threading
import time
from pathlib import Path

from pynput import keyboard

from config import (
    ALLOWED_COMMANDS,
    FEATHER_PORT,
    KEY_REPEAT_INTERVAL,
)
from serial_writer import SerialWriter

# ── Logging configuration ──────────────────────────────────
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

# ── Key mapping (same as remote_sender.py) ─────────────────
CONTROL_KEYS: dict = {"w": "w", "s": "s", "a": "a", "d": "d"}
STOP_CHAR: str = " "
QUIT_KEY: str = "q"


class LocalController:
    """Local keyboard controller: pynput -> SerialWriter -> Feather M4 CAN."""

    def __init__(self) -> None:
        self._serial = SerialWriter(port=FEATHER_PORT)
        self._running = False

        # Currently pressed control chars, protected by a Lock
        self._pressed_keys: set[str] = set()
        self._keys_lock = threading.Lock()

        # Key repeat thread
        self._repeat_thread: threading.Thread | None = None

    # ── Serial write helpers ────────────────────────────────

    def _send(self, char: str) -> None:
        """Write a single control character to serial."""
        try:
            self._serial.write_command(char)
        except Exception as e:
            logger.error(f"Serial write error: {e}")
            self._running = False

    def _emergency_stop(self) -> None:
        """Send emergency stop via serial."""
        try:
            self._serial.emergency_stop()
        except Exception as e:
            logger.error(f"Emergency stop serial error: {e}")

    # ── Key repeat thread ───────────────────────────────────

    def _key_repeat_loop(self) -> None:
        """
        Continuously send active key at KEY_REPEAT_INTERVAL (10 Hz).
        Sends emergency stop (space) when no key is pressed.
        """
        logger.info(f"Key repeat thread started, rate: {1.0 / KEY_REPEAT_INTERVAL:.0f}Hz")
        while self._running:
            with self._keys_lock:
                active_keys = list(self._pressed_keys)

            if active_keys:
                self._send(active_keys[0])
                logger.debug(f"Repeat send: {repr(active_keys[0])}")
            else:
                self._send(STOP_CHAR)

            time.sleep(KEY_REPEAT_INTERVAL)

    # ── Keyboard listener callbacks ─────────────────────────

    def _on_press(self, key) -> None:
        """Key press callback."""
        char = self._key_to_char(key)
        if char is None:
            return

        if char == QUIT_KEY:
            logger.info("Quit key 'q' pressed, exiting...")
            self._running = False
            return

        if char in CONTROL_KEYS:
            send_char = CONTROL_KEYS[char]
            with self._keys_lock:
                if send_char not in self._pressed_keys:
                    self._pressed_keys.add(send_char)
                    logger.info(f"Key pressed: {repr(char)} -> sending {repr(send_char)}")
            # Immediate send for instant response
            self._send(send_char)

    def _on_release(self, key) -> None:
        """Key release callback."""
        char = self._key_to_char(key)
        if char is None:
            return

        if char in CONTROL_KEYS:
            send_char = CONTROL_KEYS[char]
            with self._keys_lock:
                self._pressed_keys.discard(send_char)
                remaining = len(self._pressed_keys)
            logger.debug(f"Key released: {repr(char)}, keys still held: {remaining}")

            # Send emergency stop immediately when all keys are released
            if remaining == 0:
                self._emergency_stop()
                logger.info("All keys released, emergency stop sent")

    @staticmethod
    def _key_to_char(key) -> str | None:
        """Convert pynput Key object to string; returns None if not applicable."""
        try:
            if hasattr(key, "char") and key.char is not None:
                return key.char
            if key == keyboard.Key.space:
                return " "
        except AttributeError:
            pass
        return None

    # ── Main flow ───────────────────────────────────────────

    def run(self) -> None:
        """Open serial port, start threads, begin keyboard listening."""
        self._serial.open()
        self._running = True

        # Start key repeat thread
        self._repeat_thread = threading.Thread(
            target=self._key_repeat_loop, daemon=True, name="key_repeat"
        )
        self._repeat_thread.start()

        logger.info("Local controller started (wasd to move, space to stop, q to quit)")
        logger.info(f"Serial port: {FEATHER_PORT}")
        logger.info("NOTE: Do NOT start robot_receiver.py at the same time (serial port conflict)")

        # pynput listener blocks until _running=False
        with keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release,
        ) as listener:
            while self._running:
                time.sleep(0.05)
            listener.stop()

        logger.info("Keyboard listener stopped")

    def shutdown(self) -> None:
        """Graceful shutdown: send emergency stop, close serial port."""
        logger.info("Shutting down local controller...")
        self._running = False

        # Final emergency stop
        if self._serial.is_open:
            self._emergency_stop()
            logger.info("Final emergency stop sent")

        # Wait for repeat thread
        if self._repeat_thread and self._repeat_thread.is_alive():
            self._repeat_thread.join(timeout=2.0)

        self._serial.close()
        logger.info("Local controller shut down")


# ── Entry point ─────────────────────────────────────────────

def main() -> None:
    controller = LocalController()

    def _signal_handler(signum, frame):
        logger.info(f"Signal {signum} received, starting graceful shutdown...")
        controller.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    try:
        controller.run()
    except Exception as e:
        logger.error(f"Local controller encountered an error: {e}")
        raise
    finally:
        controller.shutdown()


if __name__ == "__main__":
    main()
