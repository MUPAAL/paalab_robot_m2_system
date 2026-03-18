"""
Robot-side local camera viewer with yellow masking.
Displays:
1. Normal 300x300 video from OAK-D CAM_A
2. A separate window where only yellow areas are visible on black

Usage:
    cd m2_system/00_robot_side
    python cam.py

Controls (in OpenCV window):
    q - quit
"""

import logging
import os
import signal
from pathlib import Path

import cv2
import depthai as dai
import numpy as np

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
    logger.info("Starting camera viewer with yellow mask (300x300, CAM_A)")

    try:
        if _DEVICE_IP:
            logger.info(f"Connecting to device IP: {_DEVICE_IP}")
            _device = dai.Device(dai.DeviceInfo(_DEVICE_IP))
            pipeline_cm = dai.Pipeline(_device)
        else:
            pipeline_cm = dai.Pipeline()

        with pipeline_cm as pipeline:
            cam = pipeline.create(dai.node.Camera).build()
            q = cam.requestOutput((300, 300)).createOutputQueue()

            pipeline.start()

            while pipeline.isRunning() and not _stop:
                frame = q.get()

                if frame is not None:
                    img = frame.getCvFrame()

                    # Convert image from BGR to HSV
                    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

                    # Yellow range in HSV
                    # You may need to tune these values in the barn
                    lower_yellow = np.array([20, 100, 100])
                    upper_yellow = np.array([35, 255, 255])

                    # Create mask for yellow
                    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

                    # Optional cleanup to reduce speckles/noise
                    kernel = np.ones((5, 5), np.uint8)
                    yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
                    yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)

                    # Keep only yellow parts, everything else black
                    yellow_only = cv2.bitwise_and(img, img, mask=yellow_mask)

                    # Show original live view
                    cv2.imshow("Live Camera View (press q to quit)", img)

                    # Show yellow-only view
                    cv2.imshow("Yellow Only View", yellow_only)

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