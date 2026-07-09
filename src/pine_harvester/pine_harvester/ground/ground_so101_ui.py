#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Tkinter control center for the unified SO-101 ground controller."""

from __future__ import annotations

import json
import math
import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

from ground_so101_sender import (
    BUILD_ID,
    GestureController,
    LeaderSampler,
    MOTOR_NAMES,
    TeleopSender,
    RadioProtocol,
    PROTOCOL_BUILD_ID,
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
    "ERROR": COLORS["red"],
}


class GroundControlWorker:
    """Web/Tk hardware worker using the v9 gesture and TELEOP pipeline."""

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
        self.radio = None
        self.gesture = None
        self.radio_lock = threading.Lock()
        self.protocol = None
        self.leader_sampler = None
        self.teleop_sender = None

    def start(self, config: dict) -> None:
        if self.thread is not None and self.thread.is_alive():
            return
        self.config = dict(config)
        self.stop_event.clear()
        self.thread = threading.Thread(
            target=self._run,
            name="so101-ground-control-v9",
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
                "build_id": BUILD_ID,
            }
        try:
            return self.gesture.get_preview_snapshot()
        except Exception:
            return {
                "jpeg": None,
                "status": "手势预览暂不可用",
                "timestamp": 0.0,
                "frame_id": 0,
                "build_id": BUILD_ID,
            }

    def _event(self, kind: str, value=None) -> None:
        self.events.put((kind, value))

    def _log(self, text: str) -> None:
        self._event("log", text)

    def _publish_state(self) -> None:
        self._event("state", {
            "connected": self.connected,
            "mode": self.mode,
            "sequence": self.sequence,
            "values": list(self.values),
            "gesture_build_id": BUILD_ID,
        })

    def _write_bytes(self, data: bytes, flush: bool = False) -> None:
        if self.protocol is not None:
            self.protocol.write(data, flush=flush)
            return
        if self.radio is None or not self.radio.is_open:
            return
        with self.radio_lock:
            self.radio.write(data)
            if flush:
                self.radio.flush()

    def _write_frame(self, body: str, flush: bool = False) -> None:
        self._write_bytes(encode_frame(body), flush=flush)

    def _close_gesture(self) -> None:
        if self.gesture is not None:
            self.gesture.close()
            self.gesture = None

    def _deactivate_teleop(self) -> None:
        if self.teleop_sender is not None:
            self.teleop_sender.deactivate()

    def _hold(self, reason: str = "", close_gesture: bool = False) -> None:
        self._deactivate_teleop()
        if close_gesture:
            self._close_gesture()
        self.mode = "HOLD"
        if self.protocol is not None:
            self.protocol.enter_hold(self.sequence)
        else:
            self._write_bytes(
                encode_frame("MODE,HOLD")
                + encode_frame(f"H,{self.sequence}"),
                flush=True,
            )
        self._log("HOLD" + (f" — {reason}" if reason else ""))

    def _enter_teleop(self) -> None:
        self._close_gesture()
        if self.leader_sampler is None or self.teleop_sender is None:
            self._hold("主臂采样器未启动")
            return
        latest, age, error = self.leader_sampler.snapshot()
        if latest is None or age > 0.50:
            self._hold("主臂数据未就绪: " + (error or f"age={age:.3f}s"))
            return
        self.mode = "TELEOP"
        if self.protocol is None:
            self._hold("无线电协议核心未初始化")
            return
        self.sequence = self.protocol.enter_teleop(
            latest,
            self.sequence,
            self.teleop_sender,
        )
        self._log(
            f"已进入 TELEOP 主从跟随（独立 {self.teleop_sender.hz:.1f}Hz 数据流）"
        )

    def _open_gesture(self) -> None:
        self._deactivate_teleop()
        if self.gesture is None:
            self.gesture = GestureController(
                self.config["camera_index"],
                self.config["gesture_speed"],
                self.config.get("hand_model") or None,
                self.config.get("pose_model") or None,
                hand_detection_confidence=float(
                    self.config.get("hand_detection_confidence", 0.42)
                ),
                hand_tracking_confidence=float(
                    self.config.get("hand_tracking_confidence", 0.38)
                ),
                pose_detection_confidence=float(
                    self.config.get("pose_detection_confidence", 0.42)
                ),
                pose_tracking_confidence=float(
                    self.config.get("pose_tracking_confidence", 0.38)
                ),
                show_window=bool(self.config.get("show_gesture_window", True)),
                jpeg_quality=int(self.config.get("gesture_preview_quality", 82)),
            )
            self._write_frame("MODE,HOLD", flush=True)
            self.mode = "GESTURE_WAIT"
            self._log(
                f"手势控制器已打开 | {BUILD_ID} | 等待肩、肘、腕定中"
            )
        else:
            self.gesture.recenter()
            if self.mode == "GESTURE":
                self._write_frame("GESTURE_RESET", flush=True)
            self._log("手势中心已重置")

    def _handle_command(self, command: str) -> None:
        if command == "STOP":
            return
        if command == "HOLD":
            self._hold("用户指令", close_gesture=True)
        elif command == "TELEOP":
            self._enter_teleop()
        elif command == "AUTO":
            self._close_gesture()
            self._deactivate_teleop()
            self.mode = "AUTO"
            self._write_bytes(
                encode_frame("MODE,AUTO") + encode_frame("start"),
                flush=True,
            )
            self._log("已启动 AUTO 自主抓取")
        elif command == "GESTURE":
            try:
                self._open_gesture()
            except Exception as exc:
                self._close_gesture()
                self._hold(f"手势启动失败: {exc}")
        elif command == "RECENTER" and self.gesture is not None:
            self.gesture.recenter()
            if self.mode == "GESTURE":
                self._write_frame("GESTURE_RESET", flush=True)
            self._log("手势中心已重置")

    def _handle_camera_key(self, key: str | None) -> None:
        if not key:
            return
        mapping = {
            "t": "TELEOP", "g": "GESTURE", "a": "AUTO",
            "h": "HOLD", "c": "RECENTER", "q": "HOLD",
        }
        command = mapping.get(key.lower())
        if command:
            self._handle_command(command)

    def _read_feedback(self) -> None:
        while self.radio is not None and self.radio.in_waiting:
            raw = self.radio.readline().decode("ascii", errors="replace").strip()
            if raw:
                self._event("feedback", raw)

    def _run(self) -> None:
        self.mode = "CONNECTING"
        self._publish_state()
        try:
            import serial
            try:
                from so101_leader import SO101Leader, SO101LeaderConfig
                leader_source = "local so101_leader.py"
            except ImportError:
                from lerobot.teleoperators.so101_leader.so101_leader import (
                    SO101Leader, SO101LeaderConfig,
                )
                leader_source = "LeRobot"
            self._log(f"控制构建: {BUILD_ID}")
            self._log(f"主臂驱动: {leader_source}")

            leader_id = str(self.config.get("leader_id", "blue")).strip()
            if leader_id.lower() in {"none", "null", ""}:
                leader_id = None
            self.leader = SO101Leader(SO101LeaderConfig(
                port=self.config["leader_port"],
                id=leader_id,
                use_degrees=True,
            ))
            self._log("正在连接主臂……")
            self.leader.connect()

            self._log("正在连接数传……")
            self.radio = serial.Serial(
                port=self.config["radio_port"],
                baudrate=self.config["baudrate"],
                timeout=0,
                write_timeout=0.2,
            )
            time.sleep(0.7)
            self.protocol = RadioProtocol(self.radio, self.radio_lock)
            self.protocol.sync_startup()

            self.leader_sampler = LeaderSampler(self.leader)
            self.leader_sampler.start()
            if self.leader_sampler.wait_ready(2.0):
                self._log("领导机数据采样已就绪")
            else:
                _, _, error = self.leader_sampler.snapshot()
                self._log("领导机采样尚未就绪: " + (error or "等待数据"))

            self.teleop_sender = TeleopSender(
                self.radio,
                self.radio_lock,
                self.leader_sampler,
                hz=float(self.config.get("teleop_tx_hz", 15.0)),
            )
            self.teleop_sender.start()

            self.connected = True
            self.mode = "HOLD"
            self._log("HOLD — 启动安全状态（共享协议核心）")
            self._log(f"无线电协议构建: {PROTOCOL_BUILD_ID}")
            self._event("connected", None)

            hz = max(1.0, min(50.0, float(self.config.get("hz", 30.0))))
            period = 1.0 / hz
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
                tracking_lost = False
                if self.gesture is not None:
                    gesture_command, camera_key, tracking_lost = self.gesture.update()
                    self._handle_camera_key(camera_key)

                if tracking_lost and self.mode == "GESTURE":
                    self._write_frame("MODE,HOLD", flush=True)
                    self.mode = "GESTURE_WAIT"
                    self._log("肩、肘或腕跟踪丢失，已停止并等待重新识别")

                if gesture_command is not None and self.mode in {
                    "GESTURE", "GESTURE_WAIT"
                }:
                    ready = (
                        self.gesture is not None
                        and self.gesture.baseline_feature is not None
                    )
                    if self.mode == "GESTURE_WAIT" and ready:
                        self.mode = "GESTURE"
                        self._write_frame("MODE,GESTURE", flush=True)
                        time.sleep(0.05)
                        self._write_frame("GESTURE_RESET", flush=True)
                        self._log(
                            "已进入 GESTURE；当前机械臂实测姿态设为自动零位"
                        )
                    if self.mode == "GESTURE":
                        payload = json.dumps(
                            gesture_command,
                            separators=(",", ":"),
                        ).encode("ascii") + b"\n"
                        self._write_bytes(payload)

                if self.leader_sampler is not None:
                    latest, age, _ = self.leader_sampler.snapshot()
                    if latest is not None:
                        self.values = latest

                if self.mode == "TELEOP" and self.teleop_sender is not None:
                    seq, tx_age, tx_count, tx_error = self.teleop_sender.stats()
                    self.sequence = seq
                    if tx_count > 0 and tx_age > 0.30:
                        self._hold(
                            "TELEOP 无线电发送中断: "
                            + (tx_error or f"age={tx_age:.3f}s")
                        )

                self._read_feedback()
                if time.monotonic() - last_state_time >= 0.10:
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
            if self.teleop_sender is not None:
                self.teleop_sender.stop()
            if self.leader_sampler is not None:
                self.leader_sampler.stop()
            if self.radio is not None and self.radio.is_open:
                try:
                    if self.protocol is not None:
                        self.protocol.enter_hold(self.sequence)
                    else:
                        self._write_bytes(
                            encode_frame("MODE,HOLD")
                            + encode_frame(f"H,{self.sequence}"),
                            flush=True,
                        )
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
            "H           HOLD\n"
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
        self.root.bind("<Key-h>", lambda _: self._request("HOLD"))
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
        }

    def _toggle_connection(self) -> None:
        if self.worker.thread is not None and self.worker.thread.is_alive():
            self.worker.stop()
            self.connect_button.configure(text="DISCONNECTING…", state="disabled")
            return
        try:
            config = self._config()
            if not config["leader_port"] or not config["radio_port"]:
                raise ValueError("请填写主臂和数传串口")
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
