"""
Robot-side launcher (interactive menu).

Usage:
    cd m2_system/00_robot_side
    python main.py

Scenarios:
    1. Local control                      - local_controller.py
    2. Local control + camera             - local_controller.py + Camera_multiple_outputs.py
    3. Remote TCP control                 - robot_receiver.py
    4. Remote TCP control + camera stream - robot_receiver.py + camera_streamer.py
    5. Local camera test                  - camera sub-menu
    6. Web joystick control               - web_controller.py (HTTP :8888, WS :8889)
"""

import logging
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional

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

CAMERA_MULTI_CMD = [
    sys.executable, "cam_demo/Camera_multiple_outputs.py",
    "300", "300", "0", "30", "CAM_A",
    "300", "300", "0", "30", "CAM_B",
    "300", "300", "0", "30", "CAM_C",
]

MENU = {
    "1": {"label": "Local control",                      "cmds": [[sys.executable, "local_controller.py"]]},
    "2": {"label": "Local control + camera",             "cmds": [[sys.executable, "local_controller.py"], CAMERA_MULTI_CMD]},
    "3": {"label": "Remote TCP control",                 "cmds": [[sys.executable, "robot_receiver.py"]]},
    "4": {"label": "Remote TCP control + camera stream", "cmds": [[sys.executable, "robot_receiver.py"],
                                                                   [sys.executable, "camera_streamer.py"]]},
    "5": {"label": "Local camera test",                  "cmds": None},
    "6": {"label": "Web joystick control (HTTP :8888, WS :8889)", "cmds": [[sys.executable, "web_controller.py"]]},
}

CAMERA_MENU = {
    "1": {"label": "Simple viewer       (300×300, CAM_A)",
          "cmd": [sys.executable, "cam_demo/camera_viewer.py"]},
    "2": {"label": "All cameras         (full resolution)",
          "cmd": [sys.executable, "cam_demo/Display_all_cameras.py"]},
    "3": {"label": "Multi-output        (300×300, CAM_A + CAM_B + CAM_C)",
          "cmd": [sys.executable, "cam_demo/Camera_multiple_outputs.py",
                  "300", "300", "0", "30", "CAM_A",
                  "300", "300", "0", "30", "CAM_B",
                  "300", "300", "0", "30", "CAM_C"]},
    "4": {"label": "Depth align demo",
          "cmd": [sys.executable, "cam_demo/Depth_Align.py"]},
    "5": {"label": "Detection (YOLO)    demo",
          "cmd": [sys.executable, "cam_demo/Detection_network.py"]},
}


def ask_camera_selection() -> dict:
    """弹出相机选择菜单，返回要注入子进程的环境变量 dict。"""
    from config import CAM1_IP, CAM2_IP
    print()
    print("─" * 40)
    print("  请选择相机 / Select Camera")
    print("─" * 40)
    print(f"  1. 相机 1  (IP: {CAM1_IP})")
    print(f"  2. 相机 2  (IP: {CAM2_IP})")
    print("  3. 两台相机")
    print("─" * 40)
    while True:
        try:
            choice = input("  选择 [1/2/3]: ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            choice = "1"
        if choice == "1":
            return {"CAM_SELECTION": "1", "DEVICE_IP": CAM1_IP}
        elif choice == "2":
            return {"CAM_SELECTION": "2", "DEVICE_IP": CAM2_IP}
        elif choice == "3":
            return {"CAM_SELECTION": "both"}  # DEVICE_IP 不设置，auto-detect
        print("  请输入 1、2 或 3")


def print_menu() -> None:
    print()
    print("=" * 55)
    print("   Farm Robot — Robot-side Launcher")
    print("=" * 55)
    for key, item in MENU.items():
        print(f"  {key}. {item['label']}")
    print()
    print("  q. Quit")
    print("=" * 55)


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
    try:
        import termios
        termios.tcflush(sys.stdin, termios.TCIFLUSH)
    except Exception:
        pass


def run_scripts(cmds: list[list[str]], env_extra: Optional[dict] = None) -> None:
    """Launch commands as subprocesses and wait until all exit."""
    procs: list[subprocess.Popen] = []
    env = {**os.environ, **(env_extra or {})}

    for cmd in cmds:
        logger.info(f"Starting: {cmd[1]}")
        try:
            p = subprocess.Popen(cmd, env=env)
            procs.append(p)
        except Exception as e:
            logger.error(f"Failed to start {cmd[1]}: {e}")
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

    names = ", ".join(cmd[1] for cmd in cmds)
    logger.info(f"Running: {names} — press Ctrl+C to stop all")

    try:
        while True:
            for i, p in enumerate(procs):
                ret = p.poll()
                if ret is not None:
                    logger.info(f"{cmds[i][1]} exited (code {ret}), terminating others...")
                    _terminate_all()
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

    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.SIG_DFL)
    logger.info("All processes stopped")


def run_single_cmd(cmd: list[str], env_extra: Optional[dict] = None) -> None:
    run_scripts([cmd], env_extra=env_extra)


def run_camera_menu(env_extra: Optional[dict] = None) -> None:
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
        run_single_cmd(item["cmd"], env_extra=env_extra)
        print_camera_menu()


def main() -> None:
    print_menu()

    while True:
        try:
            choice = input("Select [1-6 / q]: ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            logger.info("Launcher exiting")
            sys.exit(0)

        if choice == "q":
            logger.info("Launcher exiting")
            sys.exit(0)

        if choice == "5":
            cam_env = ask_camera_selection()
            run_camera_menu(env_extra=cam_env)
            print_menu()
            continue

        if choice not in MENU:
            print(f"  Invalid option '{choice}', enter 1-6 or q")
            continue

        item = MENU[choice]
        logger.info(f"Selected: [{choice}] {item['label']}")
        print(f"\n>>> Starting: {item['label']}\n")

        cam_env: Optional[dict] = None
        if choice in ("2", "4"):
            cam_env = ask_camera_selection()

        try:
            run_scripts(item["cmds"], env_extra=cam_env)
        except Exception as e:
            logger.error(f"Launch error: {e}")

        _flush_stdin()
        print_menu()


if __name__ == "__main__":
    main()
