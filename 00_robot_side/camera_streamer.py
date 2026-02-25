"""
Robot-side OAK-D PoE camera streamer.
Captures MJPEG frames from two OAK-D W PoE cameras and serves them
as MJPEG-over-HTTP streams for remote viewing.

Usage:
    export CAM1_IP=192.168.1.101
    export CAM2_IP=192.168.1.102
    export LOCAL_DISPLAY=1   # optional: show preview on Mac Mini
    cd m2_system/00_robot_side
    python camera_streamer.py

Streams:
    Camera 1: http://<mac-mini-ip>:8080
    Camera 2: http://<mac-mini-ip>:8081

Notes:
    - OAK-D VPU handles MJPEG encoding (low CPU load on Mac Mini)
    - Uses latest-frame strategy: skips stale frames, avoids latency buildup
    - LOCAL_DISPLAY=1 requires macOS (cv2.imshow must run on main thread)
    - Runs independently of robot_receiver.py and local_controller.py
"""

import logging
import signal
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import depthai as dai

from config import (
    CAM1_IP,
    CAM1_STREAM_PORT,
    CAM2_IP,
    CAM2_STREAM_PORT,
    CAM_FPS,
    LOCAL_DISPLAY,
    MJPEG_QUALITY,
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


# ── MJPEG HTTP handler ─────────────────────────────────────

class MJPEGHandler(BaseHTTPRequestHandler):
    """HTTP handler that serves a continuous MJPEG stream."""

    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()

        logger.info(f"[HTTP] Client connected: {self.client_address}")
        try:
            while True:
                with self.server.frame_lock:
                    frame = self.server.latest_frame

                if frame is not None:
                    try:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
                    except (BrokenPipeError, ConnectionResetError):
                        logger.info(f"[HTTP] Client disconnected: {self.client_address}")
                        break

                time.sleep(1.0 / CAM_FPS)

        except Exception as e:
            logger.error(f"[HTTP] Stream error for {self.client_address}: {e}")

    def log_message(self, format, *args):
        # Suppress default HTTP access logs (already logged above)
        pass


class FrameHTTPServer(ThreadingHTTPServer):
    """ThreadingHTTPServer that carries the shared latest_frame and lock."""

    def __init__(self, server_address, RequestHandlerClass):
        super().__init__(server_address, RequestHandlerClass)
        self.latest_frame: bytes | None = None
        self.frame_lock = threading.Lock()


# ── Camera capture thread ──────────────────────────────────

def _build_pipeline() -> dai.Pipeline:
    """Build a depthai Pipeline: ColorCamera -> VideoEncoder(MJPEG) -> XLinkOut."""
    pipeline = dai.Pipeline()

    cam = pipeline.create(dai.node.ColorCamera)
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam.setFps(CAM_FPS)
    cam.setInterleaved(False)

    encoder = pipeline.create(dai.node.VideoEncoder)
    encoder.setDefaultProfilePreset(
        CAM_FPS,
        dai.VideoEncoderProperties.Profile.MJPEG,
    )
    encoder.setQuality(MJPEG_QUALITY)

    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("mjpeg")

    cam.video.link(encoder.input)
    encoder.bitstream.link(xout.input)

    return pipeline


def _cam_thread(
    cam_label: str,
    cam_ip: str,
    http_server: FrameHTTPServer,
    stop_event: threading.Event,
) -> None:
    """
    Camera capture thread.
    Connects to an OAK-D PoE camera, grabs MJPEG frames, updates http_server.latest_frame.
    Retries with exponential backoff on failure.
    """
    retry_delay = 2.0
    max_retry_delay = 30.0

    while not stop_event.is_set():
        logger.info(f"[{cam_label}] Connecting to camera at {cam_ip}...")
        try:
            pipeline = _build_pipeline()
            device_info = dai.DeviceInfo(cam_ip)

            with dai.Device(pipeline, device_info) as device:
                logger.info(f"[{cam_label}] Camera connected: {cam_ip}")
                retry_delay = 2.0  # reset on successful connect

                q = device.getOutputQueue(name="mjpeg", maxSize=1, blocking=False)

                while not stop_event.is_set():
                    try:
                        pkt = q.tryGet()
                        if pkt is not None:
                            frame_bytes = bytes(pkt.getData())
                            with http_server.frame_lock:
                                http_server.latest_frame = frame_bytes
                    except Exception as e:
                        logger.error(f"[{cam_label}] Frame read error: {e}")
                        break

                    time.sleep(0.001)  # yield to other threads

        except Exception as e:
            if stop_event.is_set():
                break
            logger.error(f"[{cam_label}] Camera connection failed: {e}. Retrying in {retry_delay:.0f}s...")
            time.sleep(retry_delay)
            retry_delay = min(retry_delay * 2, max_retry_delay)

    logger.info(f"[{cam_label}] Camera thread exiting")


# ── Main CameraStreamer class ──────────────────────────────

class CameraStreamer:
    """
    Manages two OAK-D camera capture threads and two MJPEG HTTP server threads.
    Optionally shows local preview on main thread (required for macOS).
    """

    def __init__(self) -> None:
        self._stop_event = threading.Event()

        # HTTP servers (created in run())
        self._server1: FrameHTTPServer | None = None
        self._server2: FrameHTTPServer | None = None

        # Background threads
        self._threads: list[threading.Thread] = []

    def _start_http_server(
        self,
        port: int,
        label: str,
    ) -> FrameHTTPServer:
        """Create and start a FrameHTTPServer in a daemon thread."""
        server = FrameHTTPServer(("0.0.0.0", port), MJPEGHandler)
        t = threading.Thread(
            target=server.serve_forever,
            daemon=True,
            name=f"http_{label}",
        )
        t.start()
        self._threads.append(t)
        logger.info(f"[{label}] MJPEG HTTP server started on port {port}")
        return server

    def _start_cam_thread(
        self,
        label: str,
        cam_ip: str,
        http_server: FrameHTTPServer,
    ) -> None:
        """Start a camera capture thread."""
        t = threading.Thread(
            target=_cam_thread,
            args=(label, cam_ip, http_server, self._stop_event),
            daemon=True,
            name=f"cam_{label}",
        )
        t.start()
        self._threads.append(t)

    def run(self) -> None:
        """Start all threads; optionally run local preview on main thread."""
        # Start HTTP servers
        self._server1 = self._start_http_server(CAM1_STREAM_PORT, "CAM1")
        self._server2 = self._start_http_server(CAM2_STREAM_PORT, "CAM2")

        # Start camera capture threads
        self._start_cam_thread("CAM1", CAM1_IP, self._server1)
        self._start_cam_thread("CAM2", CAM2_IP, self._server2)

        logger.info(f"Camera streamer running. Streams:")
        logger.info(f"  CAM1: http://0.0.0.0:{CAM1_STREAM_PORT}  (source: {CAM1_IP})")
        logger.info(f"  CAM2: http://0.0.0.0:{CAM2_STREAM_PORT}  (source: {CAM2_IP})")

        if LOCAL_DISPLAY:
            self._local_display_loop()
        else:
            logger.info("Local display disabled. Press Ctrl+C to stop.")
            try:
                self._stop_event.wait()
            except KeyboardInterrupt:
                pass

    def _local_display_loop(self) -> None:
        """
        Show local preview with cv2.imshow (must run on main thread on macOS).
        Press 'q' in the OpenCV window to quit.
        """
        logger.info("Local display enabled. Press 'q' in the preview window to quit.")
        while not self._stop_event.is_set():
            frames = []

            with self._server1.frame_lock:
                raw1 = self._server1.latest_frame
            with self._server2.frame_lock:
                raw2 = self._server2.latest_frame

            for label, raw in [("CAM1", raw1), ("CAM2", raw2)]:
                if raw is not None:
                    try:
                        import numpy as np
                        arr = np.frombuffer(raw, dtype=np.uint8)
                        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                        if img is not None:
                            frames.append((label, img))
                    except Exception as e:
                        logger.warning(f"[{label}] Preview decode error: {e}")

            for label, img in frames:
                cv2.imshow(f"Local Preview - {label}", img)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                logger.info("'q' pressed in preview window, stopping...")
                self._stop_event.set()
                break

            time.sleep(0.01)

        cv2.destroyAllWindows()

    def shutdown(self) -> None:
        """Signal all threads to stop and shut down HTTP servers."""
        logger.info("Shutting down camera streamer...")
        self._stop_event.set()

        if self._server1:
            self._server1.shutdown()
        if self._server2:
            self._server2.shutdown()

        for t in self._threads:
            if t.is_alive():
                t.join(timeout=3.0)

        logger.info("Camera streamer shut down")


# ── Entry point ─────────────────────────────────────────────

def main() -> None:
    streamer = CameraStreamer()

    def _signal_handler(signum, frame):
        logger.info(f"Signal {signum} received, starting graceful shutdown...")
        streamer.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    try:
        streamer.run()
    except Exception as e:
        logger.error(f"Camera streamer encountered an error: {e}")
        raise
    finally:
        streamer.shutdown()


if __name__ == "__main__":
    main()
