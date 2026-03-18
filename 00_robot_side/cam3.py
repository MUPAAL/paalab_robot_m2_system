"""
Robot-side local camera viewer with yellow masking.
Displays:
1. Normal live video
2. Yellow-only view

Usage:
    cd m2_system/00_robot_side
    python cam.py

Controls:
    q - quit
"""

import logging
import os
import signal
from pathlib import Path

import cv2
import depthai as dai
import numpy as np

# Default to Camera 2, since that is now the front camera
_DEVICE_IP = os.environ.get("DEVICE_IP", "10.95.76.11")

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

_stop = False


def _signal_handler(signum, frame):
    global _stop
    logger.info(f"Signal {signum} received, stopping...")
    _stop = True


signal.signal(signal.SIGINT, _signal_handler)
signal.signal(signal.SIGTERM, _signal_handler)


def main():
    global _stop
    logger.info(f"Starting camera viewer with yellow mask using device IP: {_DEVICE_IP}")

    try:
        logger.info(f"Connecting to device IP: {_DEVICE_IP}")
        _device = dai.Device(dai.DeviceInfo(_DEVICE_IP))
        pipeline_cm = dai.Pipeline(_device)

        with pipeline_cm as pipeline:
            cam = pipeline.create(dai.node.Camera).build()
            q = cam.requestOutput((1920, 1080)).createOutputQueue()

            pipeline.start()

            # Make windows resizable and place them side by side
            cv2.namedWindow("Live Camera View", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Yellow Only View", cv2.WINDOW_NORMAL)

            cv2.moveWindow("Live Camera View", 0, 0)
            cv2.moveWindow("Yellow Only View", 650, 0)

            while pipeline.isRunning() and not _stop:
                frame = q.get()

                if frame is not None:
                    img = frame.getCvFrame()

                    # Convert to HSV
                    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

                    # Yellow range
                    lower_yellow = np.array([15, 50, 50])
                    upper_yellow = np.array([37, 255, 255])

                    # Mask yellow
                    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

                    # Morphological cleanup
                    kernel = np.ones((3, 3), np.uint8)
                    yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)

                    # Yellow-only output
                    yellow_only = cv2.bitwise_and(img, img, mask=yellow_mask)

                    # Resize for display only
                    display_img = cv2.resize(img, (640, 360))
                    display_yellow = cv2.resize(yellow_only, (640, 360))

                    # Show both windows
                    cv2.imshow("Live Camera View", display_img)
                    cv2.imshow("Yellow Only View", display_yellow)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    except Exception as e:
        logger.error(f"Camera viewer error: {e}")
        raise

    finally:
        cv2.destroyAllWindows()
        logger.info("Camera viewer stopped")


if __name__ == "__main__":
    main()