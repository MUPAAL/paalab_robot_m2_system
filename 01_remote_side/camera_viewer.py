"""
Remote-side dual OAK-D camera viewer.
Connects to MJPEG HTTP streams from the robot-side camera_streamer.py
and displays both feeds side-by-side using OpenCV.

Usage:
    export ROBOT_HOST=192.168.x.x   # Mac Mini LAN IP
    cd m2_system/01_remote_side
    python camera_viewer.py

Controls:
    q (in OpenCV window)  - quit

Optional overrides:
    export CAM1_URL=http://192.168.x.x:8080
    export CAM2_URL=http://192.168.x.x:8081
"""

import logging
import signal
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

from config import CAM1_URL, CAM2_URL

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

# ── Display configuration ──────────────────────────────────
DISPLAY_WIDTH: int = 640   # per-camera display width
DISPLAY_HEIGHT: int = 360  # per-camera display height
RECONNECT_DELAY_INIT: float = 1.0
RECONNECT_DELAY_MAX: float = 30.0


# ── Per-camera reader thread ───────────────────────────────

class CameraReader:
    """
    Reads MJPEG frames from a URL in a background thread.
    Stores only the latest frame (drops stale ones to avoid latency buildup).
    Reconnects automatically with exponential backoff on stream failure.
    """

    def __init__(self, label: str, url: str) -> None:
        self.label = label
        self.url = url

        self.latest_frame: np.ndarray | None = None
        self._frame_lock = threading.Lock()

        self._stop_event = threading.Event()
        self._thread = threading.Thread(
            target=self._read_loop,
            daemon=True,
            name=f"reader_{label}",
        )

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()

    def get_frame(self) -> np.ndarray | None:
        with self._frame_lock:
            return self.latest_frame

    def _read_loop(self) -> None:
        """
        Continuously open the MJPEG stream URL via cv2.VideoCapture.
        On disconnect or error, retry with exponential backoff.
        """
        retry_delay = RECONNECT_DELAY_INIT

        while not self._stop_event.is_set():
            logger.info(f"[{self.label}] Connecting to stream: {self.url}")
            cap = cv2.VideoCapture(self.url)

            if not cap.isOpened():
                logger.warning(
                    f"[{self.label}] Failed to open stream. Retrying in {retry_delay:.0f}s..."
                )
                time.sleep(retry_delay)
                retry_delay = min(retry_delay * 2, RECONNECT_DELAY_MAX)
                continue

            logger.info(f"[{self.label}] Stream opened successfully")
            retry_delay = RECONNECT_DELAY_INIT  # reset on success

            try:
                while not self._stop_event.is_set():
                    ret, frame = cap.read()
                    if not ret:
                        logger.warning(f"[{self.label}] Stream read failed, reconnecting...")
                        break

                    # Resize for display
                    frame_resized = cv2.resize(frame, (DISPLAY_WIDTH, DISPLAY_HEIGHT))

                    with self._frame_lock:
                        self.latest_frame = frame_resized

            except Exception as e:
                logger.error(f"[{self.label}] Unexpected read error: {e}")
            finally:
                cap.release()

            if not self._stop_event.is_set():
                logger.info(f"[{self.label}] Reconnecting in {retry_delay:.0f}s...")
                time.sleep(retry_delay)
                retry_delay = min(retry_delay * 2, RECONNECT_DELAY_MAX)

        logger.info(f"[{self.label}] Reader thread exiting")


# ── Main viewer ────────────────────────────────────────────

class CameraViewer:
    """
    Displays two MJPEG camera feeds side-by-side in an OpenCV window.
    Main thread runs the display loop (required for GUI on most systems).
    """

    def __init__(self) -> None:
        self._reader1 = CameraReader("CAM1", CAM1_URL)
        self._reader2 = CameraReader("CAM2", CAM2_URL)
        self._running = False

    def run(self) -> None:
        self._running = True
        self._reader1.start()
        self._reader2.start()

        logger.info(f"Camera viewer started")
        logger.info(f"  CAM1: {CAM1_URL}")
        logger.info(f"  CAM2: {CAM2_URL}")
        logger.info("Press 'q' in the viewer window to quit")

        # Placeholder frame (shown while waiting for connection)
        placeholder = np.zeros((DISPLAY_HEIGHT, DISPLAY_WIDTH, 3), dtype=np.uint8)
        cv2.putText(
            placeholder,
            "Connecting...",
            (DISPLAY_WIDTH // 2 - 80, DISPLAY_HEIGHT // 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (200, 200, 200),
            2,
        )

        while self._running:
            f1 = self._reader1.get_frame()
            f2 = self._reader2.get_frame()

            display1 = f1 if f1 is not None else placeholder.copy()
            display2 = f2 if f2 is not None else placeholder.copy()

            # Add label overlay
            for img, label in [(display1, "CAM1"), (display2, "CAM2")]:
                cv2.putText(
                    img, label, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2,
                )

            # Concatenate side-by-side
            combined = np.hstack([display1, display2])
            cv2.imshow("Camera Viewer (q to quit)", combined)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                logger.info("'q' pressed, exiting camera viewer...")
                self._running = False
                break

            time.sleep(0.01)

        cv2.destroyAllWindows()

    def shutdown(self) -> None:
        logger.info("Shutting down camera viewer...")
        self._running = False
        self._reader1.stop()
        self._reader2.stop()
        cv2.destroyAllWindows()
        logger.info("Camera viewer shut down")


# ── Entry point ─────────────────────────────────────────────

def main() -> None:
    viewer = CameraViewer()

    def _signal_handler(signum, frame):
        logger.info(f"Signal {signum} received, starting graceful shutdown...")
        viewer.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    try:
        viewer.run()
    except Exception as e:
        logger.error(f"Camera viewer encountered an error: {e}")
        raise
    finally:
        viewer.shutdown()


if __name__ == "__main__":
    main()
