from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="SO101 hand/arm landmark preview only. No leader, no radio, no robot command output."
    )
    parser.add_argument("--camera-index", type=int, default=0, help="camera index, usually 0 or 1")
    parser.add_argument(
        "--hand-model",
        default=None,
        help="optional hand_landmarker.task path; defaults to the model beside ground_so101_sender.py",
    )
    parser.add_argument(
        "--gesture-speed-scale",
        type=float,
        default=0.90,
        help="kept for compatibility with GestureController; does not send robot commands",
    )
    return parser.parse_args()


def main() -> int:
    ground_dir = Path(__file__).resolve().parent
    if str(ground_dir) not in sys.path:
        sys.path.insert(0, str(ground_dir))

    from ground_so101_sender import GestureController

    args = parse_args()
    gesture = None
    try:
        gesture = GestureController(
            args.camera_index,
            args.gesture_speed_scale,
            args.hand_model,
        )
        print("SO101 手势纯预览：不连接主臂，不连接数传，不发送任何机械臂控制指令。")
        print("窗口内按 Q 退出，按 C 重新定中。")
        while True:
            _command, key, _lost = gesture.update()
            if key:
                lower = key.lower()
                if lower == "q":
                    break
                if lower == "c":
                    gesture.recenter()
            time.sleep(0.001)
    except KeyboardInterrupt:
        pass
    finally:
        if gesture is not None:
            gesture.close()
        print("手势纯预览已关闭。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
