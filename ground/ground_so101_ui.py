#!/usr/bin/env python3
"""Tkinter control center for the unified SO-101 ground controller."""

from __future__ import annotations

import json
import math
import queue
import threading
import time
import builtins
import tkinter as tk
from tkinter import messagebox, ttk

from ground_so101_sender import (
    EEG_DOWN_POSE,
    GestureController,
    MOTOR_NAMES,
    encode_frame,
)


COLORS = {
    "bg": "#0f172a",
    "panel": "#172033",
    "panel_2": "#1e293b",
    "text": "#e5eefb",
    "muted": "#94a3b8",
    "border": "#334155",
    "green": "#22c55e",
    "blue": "#3b82f6",
    "purple": "#a855f7",
    "amber": "#f59e0b",
    "red": "#ef4444",
    "cyan": "#06b6d4",
}


MODE_COLORS = {
    "DISCONNECTED": COLORS["muted"],
    "CONNECTING": COLORS["amber"],
    "HOLD": COLORS["amber"],
    "TELEOP": COLORS["blue"],
    "GESTURE": COLORS["purple"],
    "GESTURE_WAIT": COLORS["purple"],
    "AUTO": COLORS["green"],
    "EEG": COLORS["cyan"],
    "ERROR": COLORS["red"],
}


class GroundControlWorker:
    """Hardware and radio loop running outside Tk's main thread."""

    def __init__(self, event_queue: queue.Queue):
        self.events = event_queue
        self.commands: queue.Queue = queue.Queue()
        self.stop_event = threading.Event()
        self.thread = None
        self.connected = False
        self.mode = "DISCONNECTED"
        self.sequence = 0
        self.values = [0.0] * len(MOTOR_NAMES)
        self.config = {}
        self.leader = None
        self.leader_is_real = False
        self.leader_connecting = False
        self.leader_error = None
        self.leader_lock = threading.Lock()
        self.leader_thread = None
        self.radio = None
        self.gesture = None
        self.gesture_preloading = False
        self.gesture_preload_error = None
        self.gesture_thread = None
        self.eeg_gripper_value = EEG_DOWN_POSE[5]

    def start(self, config: dict) -> None:
        if self.thread is not None and self.thread.is_alive():
            return
        self.config = dict(config)
        self.stop_event.clear()
        self.thread = threading.Thread(
            target=self._run,
            name="so101-ground-control",
            daemon=True,
        )
        self.thread.start()

    def request(self, command: str) -> None:
        self.commands.put(command.upper())

    def stop(self) -> None:
        self.stop_event.set()
        self.commands.put("STOP")

    def get_gesture_preview(self) -> dict:
        if self.gesture is None:
            return {
                "jpeg": None,
                "status": "手势模式未启动",
                "timestamp": 0.0,
                "frame_id": 0,
                "build_id": "",
            }
        getter = getattr(self.gesture, "get_preview_snapshot", None)
        if getter is None:
            return {
                "jpeg": None,
                "status": "当前手势控制器不支持网页预览",
                "timestamp": 0.0,
                "frame_id": 0,
                "build_id": "",
            }
        try:
            return getter()
        except Exception:
            return {
                "jpeg": None,
                "status": "手势预览暂不可用",
                "timestamp": 0.0,
                "frame_id": 0,
                "build_id": "",
            }

    def _event(self, kind: str, value=None) -> None:
        self.events.put((kind, value))

    def _log(self, text: str) -> None:
        self._event("log", text)

    def _publish_state(self) -> None:
        self._event(
            "state",
            {
                "connected": self.connected,
                "mode": self.mode,
                "sequence": self.sequence,
                "values": list(self.values),
            },
        )

    def _write_frame(self, body: str) -> None:
        self.radio.write(encode_frame(body))

    def _close_gesture(self) -> None:
        current = self.gesture
        self.gesture = None
        if current is None:
            return

        def _worker() -> None:
            try:
                current.close()
            except Exception:
                pass

        threading.Thread(
            target=_worker,
            name="so101-gesture-close",
            daemon=True,
        ).start()

    def _make_no_leader(self):
        class _NoLeader:
            def connect(self):
                pass

            def disconnect(self):
                pass

            def get_action(self):
                return {f"{name}.pos": 0.0 for name in MOTOR_NAMES}

        return _NoLeader()

    def _start_leader_connect(self) -> None:
        leader_id = self.config["leader_id"].strip()
        if leader_id.lower() in {"none", "null", ""}:
            leader_id = None
        leader_port = self.config["leader_port"].strip()
        if leader_port.lower() in {"", "none", "null"}:
            with self.leader_lock:
                self.leader = self._make_no_leader()
                self.leader_is_real = False
                self.leader_connecting = False
                self.leader_error = None
            self._log("主臂未连接；非 TELEOP 模式已就绪")
            return

        def connect_leader() -> None:
            with self.leader_lock:
                self.leader_connecting = True
                self.leader_error = None
                self.leader_is_real = False
            try:
                from lerobot.teleoperators.so101_leader import (
                    SO101Leader,
                    SO101LeaderConfig,
                )

                leader = SO101Leader(
                    SO101LeaderConfig(
                        port=leader_port,
                        id=leader_id,
                        use_degrees=True,
                    )
                )
                self._log("正在连接主臂……")
                original_input = builtins.input
                builtins.input = lambda *_args, **_kwargs: ""
                try:
                    leader.connect()
                finally:
                    builtins.input = original_input
                with self.leader_lock:
                    old_leader = self.leader
                    self.leader = leader
                    self.leader_is_real = True
                    self.leader_connecting = False
                    self.leader_error = None
                if old_leader is not None:
                    try:
                        old_leader.disconnect()
                    except Exception:
                        pass
                self._log("主臂已连接，TELEOP 可用")
            except Exception as exc:
                with self.leader_lock:
                    self.leader = self._make_no_leader()
                    self.leader_is_real = False
                    self.leader_connecting = False
                    self.leader_error = str(exc)
                self._log(f"主臂连接失败，其他模式继续可用: {exc}")

        self.leader_thread = threading.Thread(
            target=connect_leader,
            name="so101-leader-connect",
            daemon=True,
        )
        self.leader_thread.start()

    def _leader_action(self) -> dict:
        with self.leader_lock:
            leader = self.leader
        if leader is None:
            return {f"{name}.pos": 0.0 for name in MOTOR_NAMES}
        return leader.get_action()

    def _build_gesture_controller(self):
        return GestureController(
            self.config["camera_index"],
            self.config["gesture_speed"],
            self.config.get("hand_model") or None,
            self.config.get("pose_model") or None,
            show_window=bool(self.config.get("show_gesture_window", True)),
            jpeg_quality=int(self.config.get("gesture_preview_quality", 82)),
        )

    def _ensure_gesture_ready(self) -> bool:
        if self.gesture is not None:
            return True
        if self.gesture_preloading:
            self._hold("手势摄像头正在预热")
            self.mode = "GESTURE_WAIT"
            return False
        try:
            self._log("正在启动手势摄像头……")
            self.gesture = self._build_gesture_controller()
            self.gesture_preload_error = None
            return True
        except Exception as exc:
            self.gesture = None
            self.gesture_preload_error = str(exc)
            self._hold(f"手势启动失败: {exc}")
            return False

    def _start_gesture_preload(self) -> None:
        if self.gesture is not None or self.gesture_preloading:
            return

        def preload() -> None:
            self.gesture_preloading = True
            self.gesture_preload_error = None
            try:
                self.gesture = self._build_gesture_controller()
                self._log("手势摄像头已预热")
            except Exception as exc:
                self.gesture_preload_error = str(exc)
                self._log(f"手势预热失败: {exc}")
            finally:
                self.gesture_preloading = False

        self.gesture_thread = threading.Thread(
            target=preload,
            name="so101-gesture-preload",
            daemon=True,
        )
        self.gesture_thread.start()

    def _hold(self, reason="") -> None:
        self.mode = "HOLD"
        self._write_frame("MODE,HOLD")
        self._write_frame(f"H,{self.sequence}")
        self.radio.flush()
        self._log("HOLD" + (f" — {reason}" if reason else ""))

    def _send_eeg_command(self, command: str) -> None:
        self._write_frame("MODE,EEG")
        self.radio.flush()
        time.sleep(0.05)
        self._write_frame(f"EEG,{command}")
        self.radio.flush()

    def _stream_eeg_down(self) -> None:
        self.mode = "EEG"
        pose = self.config.get("eeg_down_pose", EEG_DOWN_POSE)
        move_sec = float(self.config.get("eeg_move_sec", 3.0))
        period = 1.0 / float(self.config.get("hz", 30.0))
        self._close_gesture()
        self._write_frame("MODE,EEG")
        self.radio.flush()
        end_time = time.monotonic() + move_sec
        while time.monotonic() < end_time and not self.stop_event.is_set():
            self.values = list(pose[:5]) + [self.eeg_gripper_value]
            body = "J," + str(self.sequence) + "," + ",".join(
                f"{value:.2f}" for value in self.values
            )
            self._write_frame(body)
            self._read_feedback()
            self.sequence = (self.sequence + 1) & 0xFFFF
            self.stop_event.wait(period)
        self._write_frame("MODE,EEG")
        self.radio.flush()
        self._log("EEG 脑电指地动作完成")

    def _handle_command(self, command: str) -> None:
        if command == "STOP":
            return
        if command == "HOLD":
            self._hold("用户指令")
            self._close_gesture()
        elif command == "STOW":
            self.eeg_gripper_value = float(self.config.get("eeg_open_gripper", 0.0))
            self._send_eeg_command("OPEN")
            self.mode = "STOW"
            self._write_frame("MODE,STOW")
            self.radio.flush()
            self._close_gesture()
            self._log("STOW / 回零位")
        elif command == "EEG":
            self.mode = "EEG"
            self._write_frame("MODE,EEG")
            self.radio.flush()
            self._close_gesture()
            self._log("已进入 EEG 脑电控制")
        elif command == "EEG_OPEN":
            self.eeg_gripper_value = float(self.config.get("eeg_open_gripper", 0.0))
            self._send_eeg_command("OPEN")
            self.mode = "EEG"
            self._log("EEG 脑电夹爪打开")
        elif command == "EEG_CLOSE":
            self.eeg_gripper_value = float(self.config.get("eeg_close_gripper", 100.0))
            self._send_eeg_command("CLOSE")
            self.mode = "EEG"
            self._log("EEG 脑电夹爪闭合")
        elif command == "EEG_DOWN":
            self._stream_eeg_down()
        elif command == "TELEOP":
            if self.leader_connecting:
                self._hold("主臂仍在连接，稍后再进入 TELEOP")
                return
            if not self.leader_is_real:
                reason = self.leader_error or "未连接主臂，不能进入 TELEOP"
                self._hold(reason)
                return
            self.mode = "TELEOP"
            self._write_frame("MODE,TELEOP")
            self.radio.flush()
            self._close_gesture()
            self._log("已进入 TELEOP 主从跟随")
        elif command == "AUTO":
            self.mode = "AUTO"
            self._write_frame("MODE,AUTO")
            self._write_frame("start")
            self.radio.flush()
            self._close_gesture()
            self._log("已启动 AUTO 自主抓取")
        elif command == "GESTURE":
            if self.gesture is None:
                if not self._ensure_gesture_ready():
                    return
                self._log("摄像头已打开，把手放入画面")
            else:
                self.gesture.recenter()
                if self.mode == "GESTURE":
                    self._write_frame("GESTURE_RESET")
                    self.radio.flush()
                    self._log("手势中心已重置")
                    return
            self._hold("等待检测手势")
            self.mode = "GESTURE_WAIT"
            self._log("手势中心已重置")
        elif command == "RECENTER" and self.gesture is not None:
            self.gesture.recenter()
            if self.mode == "GESTURE":
                self._write_frame("GESTURE_RESET")
                self.radio.flush()
            self._log("手势中心已重置")

    def _handle_camera_key(self, key: str | None) -> None:
        if not key:
            return
        key = key.lower()
        mapping = {
            "t": "TELEOP",
            "g": "GESTURE",
            "a": "AUTO",
            "e": "EEG",
            "o": "EEG_OPEN",
            "p": "EEG_CLOSE",
            "d": "EEG_DOWN",
            "s": "STOW",
            "h": "HOLD",
            "c": "RECENTER",
            "q": "HOLD",
        }
        if key in mapping:
            self._handle_command(mapping[key])

    def _read_feedback(self) -> None:
        while self.radio.in_waiting:
            raw = self.radio.readline().decode(
                "ascii", errors="replace"
            ).strip()
            if raw:
                self._event("feedback", raw)

    def _run(self) -> None:
        self.mode = "CONNECTING"
        self._publish_state()
        try:
            import serial

            self._log("正在连接数传……")
            self.radio = serial.Serial(
                port=self.config["radio_port"],
                baudrate=self.config["baudrate"],
                timeout=0,
                write_timeout=0.2,
            )
            time.sleep(0.1)
            self.radio.reset_input_buffer()
            self.connected = True
            self._hold("启动安全状态")
            self._event("connected", None)
            self._start_leader_connect()
            self._start_gesture_preload()

            period = 1.0 / self.config["hz"]
            next_tick = time.monotonic()
            last_state_time = 0.0
            while not self.stop_event.is_set():
                while True:
                    try:
                        command = self.commands.get_nowait()
                    except queue.Empty:
                        break
                    if command == "STOP":
                        self.stop_event.set()
                        break
                    self._handle_command(command)

                if self.stop_event.is_set():
                    break

                gesture_command = None
                gesture_active = self.gesture is not None and self.mode in {
                    "GESTURE",
                    "GESTURE_WAIT",
                }
                if gesture_active:
                    gesture_command, camera_key, hand_lost = self.gesture.update()
                    self._handle_camera_key(camera_key)
                    if hand_lost and self.mode == "GESTURE":
                        self._hold("手势丢失")
                        self.mode = "GESTURE_WAIT"

                if gesture_command is not None and self.mode in {
                    "GESTURE",
                    "GESTURE_WAIT",
                }:
                    if self.mode == "GESTURE_WAIT":
                        self.mode = "GESTURE"
                        self._write_frame("MODE,GESTURE")
                        self._log("已进入 GESTURE 手势控制")
                    payload = json.dumps(
                        gesture_command,
                        separators=(",", ":"),
                    )
                    self.radio.write((payload + "\n").encode("ascii"))

                if self.mode == "TELEOP":
                    action = self._leader_action()
                    new_values = [
                        float(action[f"{name}.pos"]) for name in MOTOR_NAMES
                    ]
                    if not all(math.isfinite(value) for value in new_values):
                        self._hold("主臂数据无效")
                        self._close_gesture()
                    else:
                        self.values = new_values
                    body = "J," + str(self.sequence) + "," + ",".join(
                        f"{value:.2f}" for value in self.values
                    )
                    self._write_frame(body)

                self._read_feedback()
                self.sequence = (self.sequence + 1) & 0xFFFF
                if time.monotonic() - last_state_time >= 0.1:
                    last_state_time = time.monotonic()
                    self._publish_state()

                next_tick += period
                delay = next_tick - time.monotonic()
                if delay > 0:
                    self.stop_event.wait(delay)
                else:
                    next_tick = time.monotonic()
        except Exception as exc:
            self.mode = "ERROR"
            self._event("error", str(exc))
        finally:
            self._close_gesture()
            if self.radio is not None and self.radio.is_open:
                try:
                    self._write_frame("MODE,HOLD")
                    self._write_frame(f"H,{self.sequence}")
                    self.radio.flush()
                except Exception:
                    pass
                self.radio.close()
            if self.leader is not None:
                try:
                    self.leader.disconnect()
                except Exception:
                    pass
            self.connected = False
            if self.mode != "ERROR":
                self.mode = "DISCONNECTED"
            self._publish_state()
            self._event("disconnected", None)


class SO101ControlUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("SO-101 Unified Control Center")
        self.root.geometry("1040x720")
        self.root.minsize(920, 650)
        self.root.configure(bg=COLORS["bg"])
        self.events: queue.Queue = queue.Queue()
        self.worker = GroundControlWorker(self.events)
        self.mode_buttons = []
        self.joint_value_labels = []
        self._build_style()
        self._build_ui()
        self._bind_keys()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(80, self._poll_events)

    def _build_style(self) -> None:
        style = ttk.Style()
        style.theme_use("clam")
        style.configure(
            "Dark.TEntry",
            fieldbackground=COLORS["panel_2"],
            foreground=COLORS["text"],
            bordercolor=COLORS["border"],
            insertcolor=COLORS["text"],
            padding=7,
        )
        style.configure(
            "Dark.TCombobox",
            fieldbackground=COLORS["panel_2"],
            foreground=COLORS["text"],
            bordercolor=COLORS["border"],
            padding=6,
        )

    def _panel(self, parent, **kwargs):
        return tk.Frame(
            parent,
            bg=COLORS["panel"],
            highlightthickness=1,
            highlightbackground=COLORS["border"],
            **kwargs,
        )

    def _label(self, parent, text, **kwargs):
        return tk.Label(
            parent,
            text=text,
            bg=kwargs.pop("bg", COLORS["panel"]),
            fg=kwargs.pop("fg", COLORS["text"]),
            **kwargs,
        )

    def _build_ui(self) -> None:
        header = tk.Frame(self.root, bg=COLORS["bg"])
        header.pack(fill="x", padx=24, pady=(20, 12))
        self._label(
            header,
            "SO-101  UNIFIED CONTROL CENTER",
            bg=COLORS["bg"],
            font=("Segoe UI Semibold", 22),
        ).pack(side="left")
        self.connection_dot = self._label(
            header,
            "●  DISCONNECTED",
            bg=COLORS["bg"],
            fg=COLORS["muted"],
            font=("Segoe UI Semibold", 11),
        )
        self.connection_dot.pack(side="right", pady=8)

        connection = self._panel(self.root)
        connection.pack(fill="x", padx=24, pady=(0, 12))
        inner = tk.Frame(connection, bg=COLORS["panel"])
        inner.pack(fill="x", padx=16, pady=14)

        defaults = (
            ("主臂串口", "COM5"),
            ("数传串口", "COM7"),
            ("校准 ID", "ground_leader"),
            ("摄像头", "0"),
        )
        self.entries = {}
        for column, (label, default) in enumerate(defaults):
            frame = tk.Frame(inner, bg=COLORS["panel"])
            frame.grid(row=0, column=column, sticky="ew", padx=(0, 10))
            self._label(
                frame,
                label,
                fg=COLORS["muted"],
                font=("Segoe UI", 9),
            ).pack(anchor="w", pady=(0, 5))
            entry = ttk.Entry(frame, style="Dark.TEntry", width=15)
            entry.insert(0, default)
            entry.pack(fill="x")
            self.entries[label] = entry
            inner.grid_columnconfigure(column, weight=1)

        self.connect_button = tk.Button(
            inner,
            text="CONNECT",
            command=self._toggle_connection,
            bg=COLORS["cyan"],
            fg="#07111e",
            activebackground="#22d3ee",
            activeforeground="#07111e",
            relief="flat",
            font=("Segoe UI Semibold", 11),
            cursor="hand2",
            padx=22,
            pady=9,
        )
        self.connect_button.grid(row=0, column=4, sticky="sew", pady=(20, 0))

        body = tk.Frame(self.root, bg=COLORS["bg"])
        body.pack(fill="both", expand=True, padx=24, pady=(0, 20))
        body.grid_columnconfigure(0, weight=5)
        body.grid_columnconfigure(1, weight=4)
        body.grid_rowconfigure(0, weight=1)

        left = self._panel(body)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        right = self._panel(body)
        right.grid(row=0, column=1, sticky="nsew", padx=(6, 0))

        self._build_control_panel(left)
        self._build_telemetry_panel(right)

    def _build_control_panel(self, parent) -> None:
        self._label(
            parent,
            "CONTROL MODE",
            fg=COLORS["muted"],
            font=("Segoe UI Semibold", 10),
        ).pack(anchor="w", padx=18, pady=(16, 5))
        self.mode_label = self._label(
            parent,
            "DISCONNECTED",
            fg=COLORS["muted"],
            font=("Segoe UI Semibold", 30),
        )
        self.mode_label.pack(anchor="w", padx=18, pady=(0, 14))

        grid = tk.Frame(parent, bg=COLORS["panel"])
        grid.pack(fill="x", padx=16)
        buttons = (
            ("TELEOP", "主从跟随  [T]", COLORS["blue"]),
            ("GESTURE", "手势控制  [G]", COLORS["purple"]),
            ("AUTO", "自主抓取  [A]", COLORS["green"]),
            ("EEG", "脑电控制  [E]", COLORS["cyan"]),
            ("STOW", "回零位  [S]", COLORS["amber"]),
            ("RECENTER", "手势定中  [C]", COLORS["cyan"]),
        )
        for index, (command, text, color) in enumerate(buttons):
            button = tk.Button(
                grid,
                text=text,
                command=lambda value=command: self._request(value),
                bg=color,
                fg="white",
                activebackground=color,
                activeforeground="white",
                disabledforeground="#64748b",
                relief="flat",
                font=("Segoe UI Semibold", 12),
                cursor="hand2",
                pady=14,
                state="disabled",
            )
            button.grid(
                row=index // 2,
                column=index % 2,
                sticky="nsew",
                padx=5,
                pady=5,
            )
            self.mode_buttons.append(button)
        grid.grid_columnconfigure(0, weight=1)
        grid.grid_columnconfigure(1, weight=1)

        hold = tk.Button(
            parent,
            text="■   HOLD / EMERGENCY STOP   [H]",
            command=lambda: self._request("HOLD"),
            bg=COLORS["red"],
            fg="white",
            activebackground="#dc2626",
            activeforeground="white",
            disabledforeground="#7f1d1d",
            relief="flat",
            font=("Segoe UI Semibold", 13),
            cursor="hand2",
            pady=16,
            state="disabled",
        )
        hold.pack(fill="x", padx=21, pady=(12, 15))
        self.mode_buttons.append(hold)

        self._label(
            parent,
            "SYSTEM LOG",
            fg=COLORS["muted"],
            font=("Segoe UI Semibold", 10),
        ).pack(anchor="w", padx=18, pady=(4, 5))
        self.log = tk.Text(
            parent,
            height=9,
            bg="#0b1220",
            fg="#cbd5e1",
            insertbackground="white",
            relief="flat",
            font=("Cascadia Mono", 9),
            padx=10,
            pady=8,
            state="disabled",
        )
        self.log.pack(fill="both", expand=True, padx=18, pady=(0, 18))

    def _build_telemetry_panel(self, parent) -> None:
        self._label(
            parent,
            "LIVE JOINT TELEMETRY",
            fg=COLORS["muted"],
            font=("Segoe UI Semibold", 10),
        ).pack(anchor="w", padx=18, pady=(16, 10))
        for name in MOTOR_NAMES:
            row = tk.Frame(parent, bg=COLORS["panel_2"])
            row.pack(fill="x", padx=18, pady=4)
            self._label(
                row,
                name,
                bg=COLORS["panel_2"],
                fg=COLORS["muted"],
                font=("Cascadia Mono", 10),
            ).pack(side="left", padx=10, pady=9)
            value = self._label(
                row,
                "--.-",
                bg=COLORS["panel_2"],
                fg=COLORS["text"],
                font=("Cascadia Mono", 12, "bold"),
            )
            value.pack(side="right", padx=10)
            self.joint_value_labels.append(value)

        self._label(
            parent,
            "SHORTCUTS",
            fg=COLORS["muted"],
            font=("Segoe UI Semibold", 10),
        ).pack(anchor="w", padx=18, pady=(20, 8))
        shortcut_text = (
            "T / Space   TELEOP\n"
            "G           GESTURE\n"
            "A           AUTO\n"
            "E           EEG\n"
            "O / P       EEG gripper open / close\n"
            "D           EEG point down\n"
            "H           HOLD\n"
            "S           STOW / HOME\n"
            "C           RECENTER\n"
            "Q           SAFE EXIT"
        )
        self._label(
            parent,
            shortcut_text,
            bg=COLORS["panel_2"],
            fg=COLORS["text"],
            justify="left",
            font=("Cascadia Mono", 10),
            padx=14,
            pady=12,
        ).pack(fill="x", padx=18, pady=(0, 18))

    def _bind_keys(self) -> None:
        self.root.bind("<Key-t>", lambda _: self._request("TELEOP"))
        self.root.bind("<space>", lambda _: self._request("TELEOP"))
        self.root.bind("<Key-g>", lambda _: self._request("GESTURE"))
        self.root.bind("<Key-a>", lambda _: self._request("AUTO"))
        self.root.bind("<Key-e>", lambda _: self._request("EEG"))
        self.root.bind("<Key-o>", lambda _: self._request("EEG_OPEN"))
        self.root.bind("<Key-p>", lambda _: self._request("EEG_CLOSE"))
        self.root.bind("<Key-d>", lambda _: self._request("EEG_DOWN"))
        self.root.bind("<Key-h>", lambda _: self._request("HOLD"))
        self.root.bind("<Key-s>", lambda _: self._request("STOW"))
        self.root.bind("<Key-c>", lambda _: self._request("RECENTER"))
        self.root.bind("<Key-q>", lambda _: self._on_close())

    def _config(self) -> dict:
        try:
            camera_index = int(self.entries["摄像头"].get().strip())
        except ValueError as exc:
            raise ValueError("摄像头必须是数字，通常为 0") from exc
        return {
            "leader_port": self.entries["主臂串口"].get().strip(),
            "radio_port": self.entries["数传串口"].get().strip(),
            "leader_id": self.entries["校准 ID"].get().strip(),
            "camera_index": camera_index,
            "baudrate": 57600,
            "hz": 30.0,
            "gesture_speed": 0.90,
            "hand_model": r"C:\Users\Lenovo\Desktop\SO101_3_modes\ground\hand_landmarker.task",
            "pose_model": r"C:\Users\Lenovo\Desktop\SO101_3_modes\ground\pose_landmarker_full.task",
            "eeg_down_pose": EEG_DOWN_POSE,
            "eeg_move_sec": 3.0,
            "eeg_open_gripper": 0.0,
            "eeg_close_gripper": 45.0,
        }

    def _toggle_connection(self) -> None:
        if self.worker.thread is not None and self.worker.thread.is_alive():
            self.worker.stop()
            self.connect_button.configure(text="DISCONNECTING…", state="disabled")
            return
        try:
            config = self._config()
            if not config["radio_port"]:
                raise ValueError("请填写数传串口")
        except ValueError as exc:
            messagebox.showerror("参数错误", str(exc))
            return
        self.connect_button.configure(text="CONNECTING…", state="disabled")
        self.worker.start(config)

    def _request(self, command: str) -> None:
        if not self.worker.connected:
            return
        if command == "AUTO" and not messagebox.askyesno(
            "确认自主抓取",
            "确认机械臂周围安全，并进入 AUTO 模式？",
        ):
            return
        self.worker.request(command)

    def _append_log(self, text: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        self.log.configure(state="normal")
        self.log.insert("end", f"[{timestamp}] {text}\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def _set_connected_controls(self, connected: bool) -> None:
        state = "normal" if connected else "disabled"
        for button in self.mode_buttons:
            button.configure(state=state)
        self.connect_button.configure(
            text="DISCONNECT" if connected else "CONNECT",
            state="normal",
            bg=COLORS["red"] if connected else COLORS["cyan"],
        )

    def _update_state(self, state: dict) -> None:
        mode = state["mode"]
        color = MODE_COLORS.get(mode, COLORS["muted"])
        self.mode_label.configure(text=mode, fg=color)
        connection_text = "●  CONNECTED" if state["connected"] else "●  DISCONNECTED"
        self.connection_dot.configure(
            text=connection_text,
            fg=COLORS["green"] if state["connected"] else COLORS["muted"],
        )
        for label, value in zip(self.joint_value_labels, state["values"]):
            label.configure(text=f"{value:8.2f}")

    def _poll_events(self) -> None:
        try:
            while True:
                kind, value = self.events.get_nowait()
                if kind == "state":
                    self._update_state(value)
                elif kind == "log":
                    self._append_log(value)
                elif kind == "feedback":
                    self._append_log(f"机载: {value}")
                elif kind == "connected":
                    self._set_connected_controls(True)
                elif kind == "disconnected":
                    self._set_connected_controls(False)
                elif kind == "error":
                    self._append_log(f"ERROR: {value}")
                    messagebox.showerror("连接/控制错误", value)
        except queue.Empty:
            pass
        self.root.after(80, self._poll_events)

    def _on_close(self) -> None:
        if self.worker.connected:
            self.worker.request("HOLD")
        self.worker.stop()
        self.root.after(250, self.root.destroy)


def main() -> None:
    root = tk.Tk()
    SO101ControlUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()

