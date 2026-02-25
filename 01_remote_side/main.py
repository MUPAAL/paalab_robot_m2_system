"""
Remote-side launcher (interactive menu).

Usage:
    cd m2_system/01_remote_side
    python main.py

Scenarios:
    1. 仅键盘控制       - remote_sender.py
    2. 键盘控制 + 相机  - remote_sender.py + camera_viewer.py
    3. 仅看相机         - camera_viewer.py
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
        logging.FileHandler(f"log/{_py_name}.log", encoding="utf-8"),
        logging.StreamHandler(),
    ],
)
logger = logging.getLogger(__name__)

# ── Menu definition ────────────────────────────────────────
MENU = {
    "1": {
        "label": "仅键盘控制",
        "scripts": ["remote_sender.py"],
    },
    "2": {
        "label": "键盘控制 + 相机画面",
        "scripts": ["remote_sender.py", "camera_viewer.py"],
    },
    "3": {
        "label": "仅看相机",
        "scripts": ["camera_viewer.py"],
    },
}


def print_menu() -> None:
    print()
    print("=" * 40)
    print("   Farm Robot — 远程端启动器")
    print("=" * 40)
    for key, item in MENU.items():
        print(f"  {key}. {item['label']}")
    print()
    print("  q. 退出")
    print("=" * 40)


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
            for p in procs:
                ret = p.poll()
                if ret is not None:
                    script_name = scripts[procs.index(p)]
                    logger.info(f"{script_name} exited (code {ret}), terminating others...")
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

    logger.info("All processes stopped")


# ── Entry point ────────────────────────────────────────────

def main() -> None:
    print_menu()

    while True:
        try:
            choice = input("请选择 [1-3 / q]: ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            logger.info("Launcher exiting")
            sys.exit(0)

        if choice == "q":
            logger.info("Launcher exiting")
            sys.exit(0)

        if choice not in MENU:
            print(f"  无效选项 '{choice}'，请输入 1-3 或 q")
            continue

        item = MENU[choice]
        logger.info(f"Selected: [{choice}] {item['label']}")
        print(f"\n>>> 启动：{item['label']}\n")

        try:
            run_scripts(item["scripts"])
        except Exception as e:
            logger.error(f"Launch error: {e}")

        # Back to menu after processes exit
        print_menu()


if __name__ == "__main__":
    main()
