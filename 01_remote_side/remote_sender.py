"""
Remote-side main program.
pynput global keyboard capture + TCP client + heartbeat thread.

Usage:
    export ROBOT_HOST=192.168.x.x   # Mac Mini LAN IP
    python remote_sender.py

Controls:
    w / s / a / d  - forward / backward / turn left / turn right
    space          - emergency stop (also sent automatically when all keys are released)
    Enter          - toggle STATE_AUTO_READY <-> STATE_AUTO_ACTIVE (sent once, no repeat)
    q              - quit
"""

import logging
import signal
import socket
import sys
import threading
import time
from pathlib import Path

from pynput import keyboard

from config import (
    CONTROL_KEYS,
    HEARTBEAT_CHAR,
    HEARTBEAT_INTERVAL,
    KEY_REPEAT_INTERVAL,
    QUIT_KEY,
    ROBOT_HOST,
    ROBOT_PORT,
    STOP_CHAR,
)

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


class RemoteSender:
    """Remote keyboard control sender."""

    def __init__(self) -> None:
        self._sock: socket.socket | None = None
        self._running = False

        # Set of currently pressed control keys, protected by a Lock
        self._pressed_keys: set[str] = set()
        self._keys_lock = threading.Lock()

        # Background threads
        self._heartbeat_thread: threading.Thread | None = None
        self._repeat_thread: threading.Thread | None = None

        # Enter key held state (suppress OS key-repeat events)
        self._enter_held: bool = False

    # ── Network connection ─────────────────────────────────

    def connect(self) -> None:
        """Connect to the robot-side TCP server."""
        logger.info(f"Connecting to robot at {ROBOT_HOST}:{ROBOT_PORT}...")
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.connect((ROBOT_HOST, ROBOT_PORT))
            logger.info(f"Connected to robot: {ROBOT_HOST}:{ROBOT_PORT}")
        except OSError as e:
            logger.error(f"Failed to connect to robot [{ROBOT_HOST}:{ROBOT_PORT}]: {e}")
            raise

    def _send(self, char: str) -> bool:
        """Send a single character; returns True on success, False if connection is lost."""
        if self._sock is None:
            return False
        try:
            self._sock.sendall(char.encode("utf-8"))
            return True
        except OSError as e:
            logger.error(f"Send failed, connection lost: {e}")
            return False

    # ── Background threads ─────────────────────────────────

    def _heartbeat_loop(self) -> None:
        """Heartbeat thread: sends 'H' every HEARTBEAT_INTERVAL seconds."""
        logger.info(f"Heartbeat thread started, interval: {HEARTBEAT_INTERVAL}s")
        while self._running:
            if not self._send(HEARTBEAT_CHAR):
                logger.warning("Heartbeat send failed, connection may be lost")
                self._running = False
                break
            logger.debug("Heartbeat sent: H")
            time.sleep(HEARTBEAT_INTERVAL)

    def _key_repeat_loop(self) -> None:
        """
        Key repeat thread: continuously sends the active control key at 10 Hz.
        Sends emergency stop (space) when no key is pressed.
        """
        logger.info(f"Key repeat thread started, rate: {1.0/KEY_REPEAT_INTERVAL:.0f}Hz")
        while self._running:
            with self._keys_lock:
                active_keys = list(self._pressed_keys)

            if active_keys:
                # Send the highest-priority key (first in list)
                char = active_keys[0]
                if not self._send(char):
                    self._running = False
                    break
                logger.debug(f"Repeat send: {repr(char)}")
            else:
                # No key pressed: keep sending emergency stop
                if not self._send(STOP_CHAR):
                    self._running = False
                    break

            time.sleep(KEY_REPEAT_INTERVAL)

    # ── Keyboard listener callbacks ────────────────────────

    def _on_press(self, key) -> None:
        """Key press callback."""
        char = self._key_to_char(key)
        if char is None:
            return

        if char == QUIT_KEY:
            logger.info("Quit key 'q' pressed, exiting...")
            self._running = False
            return

        if char == "\r":
            if not self._enter_held:           # 只在首次按下时发送，屏蔽 OS 按键重复
                self._enter_held = True
                logger.info("Enter key pressed -> sending state toggle command")
                self._send("\r")
            return

        if char in CONTROL_KEYS:
            send_char = CONTROL_KEYS[char]
            with self._keys_lock:
                if send_char not in self._pressed_keys:
                    self._pressed_keys.add(send_char)
                    logger.info(f"Key pressed: {repr(char)} -> sending {repr(send_char)}")
            # Send immediately for instant response
            self._send(send_char)

    def _on_release(self, key) -> None:
        """Key release callback."""
        char = self._key_to_char(key)
        if char is None:
            return

        if char == "\r":
            self._enter_held = False
            return

        if char in CONTROL_KEYS:
            send_char = CONTROL_KEYS[char]
            with self._keys_lock:
                self._pressed_keys.discard(send_char)
                remaining = len(self._pressed_keys)
            logger.debug(f"Key released: {repr(char)}, keys still held: {remaining}")

            # Send emergency stop immediately when all keys are released
            if remaining == 0:
                self._send(STOP_CHAR)
                logger.info("All keys released, emergency stop sent")

    @staticmethod
    def _key_to_char(key) -> str | None:
        """Convert a pynput Key object to a string; returns None if not applicable."""
        try:
            # Regular character key
            if hasattr(key, "char") and key.char is not None:
                return key.char
            # Space key
            if key == keyboard.Key.space:
                return " "
            # Enter key
            if key == keyboard.Key.enter:
                return "\r"
        except AttributeError:
            pass
        return None

    # ── Main flow ──────────────────────────────────────────

    def run(self) -> None:
        """Start all threads and begin keyboard listening."""
        self._running = True

        # Start heartbeat thread
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop, daemon=True, name="heartbeat"
        )
        self._heartbeat_thread.start()

        # Start key repeat thread
        self._repeat_thread = threading.Thread(
            target=self._key_repeat_loop, daemon=True, name="key_repeat"
        )
        self._repeat_thread.start()

        logger.info("Keyboard listener started (wasd to move, space to stop, Enter to toggle state, q to quit)")
        logger.info(f"Target: {ROBOT_HOST}:{ROBOT_PORT}")

        # pynput listener (blocks until _running=False or exception)
        with keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release,
        ) as listener:
            while self._running:
                time.sleep(0.05)
            listener.stop()

        logger.info("Keyboard listener stopped")

    def shutdown(self) -> None:
        """Graceful shutdown: send emergency stop, wait for threads, close socket."""
        logger.info("Shutting down remote sender...")
        self._running = False

        # Send final emergency stop
        self._send(STOP_CHAR)
        logger.info("Final emergency stop sent")

        # Wait for background threads to finish
        if self._heartbeat_thread and self._heartbeat_thread.is_alive():
            self._heartbeat_thread.join(timeout=2.0)
        if self._repeat_thread and self._repeat_thread.is_alive():
            self._repeat_thread.join(timeout=2.0)

        # Close socket
        if self._sock:
            try:
                self._sock.close()
            except OSError as e:
                logger.warning(f"Error closing socket: {e}")

        logger.info("Remote sender shut down")


# ── Entry point ────────────────────────────────────────────

def main() -> None:
    sender = RemoteSender()

    def _signal_handler(signum, frame):
        logger.info(f"Signal {signum} received, starting graceful shutdown...")
        sender.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    try:
        sender.connect()
        sender.run()
    except Exception as e:
        logger.error(f"Remote sender encountered an error: {e}")
        raise
    finally:
        sender.shutdown()


if __name__ == "__main__":
    main()
