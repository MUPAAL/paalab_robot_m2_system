"""
Robot-side launcher (interactive menu).

Usage:
    cd m2_system/00_robot_side
    python main.py

Scenarios:
    1. Local control (serial only)      - local_controller.py
    2. Local control + camera stream    - local_controller.py + camera_streamer.py
    3. Remote TCP control               - robot_receiver.py
    4. Remote TCP + camera stream       - robot_receiver.py  + camera_streamer.py
    5. Local camera test                - camera sub-menu
"""

import logging
import signal
import subprocess
import sys
import time
from pathlib import Path

# ── Logging configuration ──────────────────────────────────
_py_name = Path(__file__).stem
Path("log").mkdir(exist_ok=True)
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler(f"log/robot_{_py_name}.log", encoding="utf-8"),
        logging.StreamHandler(),
    ],
)
logger = logging.getLogger(__name__)

# ── Menu definition ────────────────────────────────────────
MENU = {
    "1": {
        "label": "Local control (serial only)",
        "scripts": ["local_controller.py"],
    },
    "2": {
        "label": "Local control + camera stream",
        "scripts": ["local_controller.py", "camera_streamer.py"],
    },
    "3": {
        "label": "Remote TCP control",
        "scripts": ["robot_receiver.py"],
    },
    "4": {
        "label": "Remote TCP + camera stream",
        "scripts": ["robot_receiver.py", "camera_streamer.py"],
    },
    "5": {
        "label": "Local camera test",
        "scripts": [],          # 由 run_camera_menu() 处理
    },
}

CAMERA_MENU = {
    "1": {
        "label": "Simple viewer       (300×300, CAM_A)",
        "cmd":   [sys.executable, "camera_viewer.py"],
    },
    "2": {
        "label": "All cameras         (full resolution)",
        "cmd":   [sys.executable, "demo/Display_all_cameras.py"],
    },
    "3": {
        "label": "Multi-output        (300×300, CAM_A + CAM_B + CAM_C)",
        "cmd":   [sys.executable, "demo/Camera_multiple_outputs.py",
                  "300", "300", "0", "30", "CAM_A",
                  "300", "300", "0", "30", "CAM_B",
                  "300", "300", "0", "30", "CAM_C"],
    },
    "4": {
        "label": "Depth align demo",
        "cmd":   [sys.executable, "demo/Depth_Align.py"],
    },
    "5": {
        "label": "Detection (YOLO)    demo",
        "cmd":   [sys.executable, "demo/Detection_network.py"],
    },
}


def print_menu() -> None:
    print()
    print("=" * 40)
    print("   Farm Robot — Robot-side Launcher")
    print("=" * 40)
    for key, item in MENU.items():
        print(f"  {key}. {item['label']}")
    print()
    print("  q. Quit")
    print("=" * 40)


def print_camera_menu() -> None:
    print()
    print("=" * 40)
    print("   Camera Test — Local Display")
    print("=" * 40)
    for key, item in CAMERA_MENU.items():
        print(f"  {key}. {item['label']}")
    print()
    print("  b. Back to main menu")
    print("=" * 40)


def _flush_stdin() -> None:
    """丢弃 pynput 会话遗留在 stdin 缓冲区中的按键。"""
    try:
        import termios
        termios.tcflush(sys.stdin, termios.TCIFLUSH)
    except Exception:
        pass


def run_scripts(scripts: list[str]) -> None:
    """
    Launch scripts as subprocesses and wait until all exit.
    Ctrl+C / SIGTERM will terminate all child processes.
    """
    procs: list[subprocess.Popen] = []

    for script in scripts:
        logger.info(f"Starting: {script}")
        try:
            p = subprocess.Popen([sys.executable, script])
            procs.append(p)
        except Exception as e:
            logger.error(f"Failed to start {script}: {e}")
            # Terminate already-started processes
            for running in procs:
                running.terminate()
            raise

    def _terminate_all(signum=None, frame=None) -> None:
        logger.info("Terminating all child processes...")
        for p in procs:
            if p.poll() is None:
                p.terminate()

    signal.signal(signal.SIGINT, _terminate_all)
    signal.signal(signal.SIGTERM, _terminate_all)

    logger.info(f"Running: {', '.join(scripts)} — press Ctrl+C to stop all")

    try:
        while True:
            # Check if any process exited
            for p in procs:
                ret = p.poll()
                if ret is not None:
                    script_name = scripts[procs.index(p)]
                    logger.info(f"{script_name} exited (code {ret}), terminating others...")
                    _terminate_all()
                    # Wait briefly for others to clean up
                    for other in procs:
                        try:
                            other.wait(timeout=3.0)
                        except subprocess.TimeoutExpired:
                            other.kill()
                    return

            time.sleep(0.2)

    except KeyboardInterrupt:
        _terminate_all()
        for p in procs:
            try:
                p.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                p.kill()

    logger.info("All processes stopped")


def run_single_cmd(cmd: list[str]) -> None:
    """运行一个命令（含参数），等待其退出，支持 Ctrl+C 打断。"""
    logger.info(f"Starting: {' '.join(cmd[1:])}")
    try:
        p = subprocess.Popen(cmd)
    except Exception as e:
        logger.error(f"Failed to start: {e}")
        return

    try:
        p.wait()
    except KeyboardInterrupt:
        p.terminate()
        try:
            p.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            p.kill()
    logger.info("Camera demo exited")


def run_camera_menu() -> None:
    """相机测试子菜单循环。"""
    print_camera_menu()
    while True:
        try:
            choice = input("Select [1-5 / b]: ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            return

        if choice == "b":
            return

        if choice not in CAMERA_MENU:
            print(f"  Invalid option '{choice}', enter 1-5 or b")
            continue

        item = CAMERA_MENU[choice]
        logger.info(f"Camera test selected: [{choice}] {item['label']}")
        print(f"\n>>> Starting: {item['label']}\n")
        run_single_cmd(item["cmd"])
        print_camera_menu()


# ── Entry point ────────────────────────────────────────────

def main() -> None:
    print_menu()

    while True:
        try:
            choice = input("Select [1-5 / q]: ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            logger.info("Launcher exiting")
            sys.exit(0)

        if choice == "q":
            logger.info("Launcher exiting")
            sys.exit(0)

        if choice == "5":
            run_camera_menu()
            print_menu()
            continue

        if choice not in MENU:
            print(f"  Invalid option '{choice}', enter 1-5 or q")
            continue

        item = MENU[choice]
        logger.info(f"Selected: [{choice}] {item['label']}")
        print(f"\n>>> Starting: {item['label']}\n")

        try:
            run_scripts(item["scripts"])
        except Exception as e:
            logger.error(f"Launch error: {e}")

        # 冲洗 pynput 会话遗留在 stdin 缓冲区中的按键，防止 Invalid option 刷屏
        _flush_stdin()

        # Back to menu after processes exit
        print_menu()


if __name__ == "__main__":
    main()
