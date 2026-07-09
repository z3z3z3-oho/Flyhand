#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SO-101 local FastAPI backend and optional production frontend host."""

from __future__ import annotations

import argparse
import queue
import threading
import time
import webbrowser
from collections import deque
from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field
from ground_so101_sender import BUILD_ID, PROTOCOL_BUILD_ID


# Support running directly from the ground directory and package execution.
try:
    from ground_so101_ui import GroundControlWorker
except ImportError:
    try:
        from pine_harvester.ground_so101_ui import GroundControlWorker
    except ImportError:
        from pine_harvester.pine_harvester.ui.ground_so101_ui import (
            GroundControlWorker,
        )


ROOT = Path(__file__).resolve().parent
DEFAULT_HAND_MODEL = ROOT / "hand_landmarker.task"

# Support both the new React project and the older web/dist layout.
DIST_CANDIDATES = (
    ROOT / "so101-control-ui" / "dist",
    ROOT / "so101-control-ui-clean" / "dist",
    ROOT / "web" / "dist",
)
DIST = next((path for path in DIST_CANDIDATES if path.exists()), DIST_CANDIDATES[0])


class ConnectConfig(BaseModel):
    leader_port: str = Field(min_length=1, max_length=32)
    radio_port: str = Field(min_length=1, max_length=32)
    leader_id: str = Field(default="blue", max_length=64)
    camera_index: int = Field(default=0, ge=0, le=16)
    baudrate: int = Field(default=57600, ge=1200, le=1_000_000)
    hz: float = Field(default=30.0, ge=1.0, le=50.0)
    gesture_speed: float = Field(default=0.35, ge=0.1, le=1.0)
    hand_model: str = Field(
        default=str(DEFAULT_HAND_MODEL),
        max_length=512,
    )


class ControlRuntime:
    def __init__(self) -> None:
        self.events: queue.Queue = queue.Queue()
        self.worker = GroundControlWorker(self.events)
        self.lock = threading.Lock()
        self.logs = deque(maxlen=120)
        self.state: dict[str, Any] = {
            "connected": False,
            "connecting": False,
            "mode": "DISCONNECTED",
            "sequence": 0,
            "values": [0.0] * 6,
            "error": None,
        }
        self.stop_event = threading.Event()
        self.monitor = threading.Thread(
            target=self._monitor_events,
            name="so101-web-event-monitor",
            daemon=True,
        )
        self.monitor.start()

    def _append_log(self, level: str, message: str) -> None:
        self.logs.append(
            {
                "id": str(time.time_ns()),
                "time": time.strftime("%H:%M:%S"),
                "level": level,
                "message": message,
            }
        )

    def _monitor_events(self) -> None:
        while not self.stop_event.is_set():
            try:
                kind, value = self.events.get(timeout=0.2)
            except queue.Empty:
                continue

            with self.lock:
                if kind == "state":
                    self.state.update(value)
                    self.state["connecting"] = value.get("mode") == "CONNECTING"
                elif kind == "connected":
                    self.state["connected"] = True
                    self.state["connecting"] = False
                    self.state["error"] = None
                    self._append_log("info", "主臂与数传已连接")
                elif kind == "disconnected":
                    self.state["connected"] = False
                    self.state["connecting"] = False
                    self._append_log("info", "连接已关闭")
                elif kind == "log":
                    self._append_log("info", str(value))
                elif kind == "feedback":
                    self._append_log("feedback", str(value))
                elif kind == "error":
                    self.state["error"] = str(value)
                    self.state["connecting"] = False
                    self._append_log("error", str(value))

    def snapshot(self) -> dict[str, Any]:
        preview = self.worker.get_gesture_preview()
        with self.lock:
            return {
                **self.state,
                "logs": list(self.logs),
                "gesture_preview": {
                    "ready": bool(preview.get("jpeg")),
                    "status": preview.get("status", ""),
                    "timestamp": preview.get("timestamp", 0.0),
                    "frame_id": preview.get("frame_id", 0),
                    "build_id": preview.get("build_id", ""),
                },
            }

    def gesture_preview(self) -> dict[str, Any]:
        return self.worker.get_gesture_preview()

    def connect(self, config: dict[str, Any]) -> None:
        if self.worker.thread is not None and self.worker.thread.is_alive():
            raise RuntimeError("控制器已在运行")

        hand_model = Path(str(config.get("hand_model", ""))).expanduser()
        if hand_model and not hand_model.is_file():
            raise RuntimeError(f"手势模型不存在: {hand_model}")

        with self.lock:
            self.state.update(
                {
                    "connecting": True,
                    "mode": "CONNECTING",
                    "error": None,
                }
            )
            self._append_log("info", "正在连接主臂与数传")

        self.worker.start(config)

    def disconnect(self) -> None:
        self.worker.stop()

    def command(self, command: str) -> None:
        if not self.worker.connected:
            raise RuntimeError("设备尚未连接")
        self.worker.request(command)

    def close(self) -> None:
        self.worker.stop()
        self.stop_event.set()


runtime = ControlRuntime()

app = FastAPI(
    title="SO-101 Local Control API",
    docs_url="/api/docs",
    redoc_url=None,
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://127.0.0.1:5173",
        "http://localhost:5173",
        "http://127.0.0.1:8765",
        "http://localhost:8765",
    ],
    allow_credentials=False,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/api/health")
def health() -> dict[str, Any]:
    return {
        "ok": True,
        "frontend_dist": str(DIST),
        "frontend_built": DIST.exists(),
        "gesture_build": BUILD_ID,
        "protocol_build": PROTOCOL_BUILD_ID,
    }


@app.get("/api/state")
def state() -> dict[str, Any]:
    return runtime.snapshot()


@app.get("/api/gesture/frame.jpg")
def gesture_frame() -> Response:
    preview = runtime.gesture_preview()
    jpeg = preview.get("jpeg")
    if not jpeg:
        return Response(status_code=204)
    return Response(
        content=jpeg,
        media_type="image/jpeg",
        headers={"Cache-Control": "no-store", "Pragma": "no-cache"},
    )


@app.get("/api/gesture/stream")
def gesture_stream() -> StreamingResponse:
    def frames():
        last_frame_id = -1
        try:
            while True:
                preview = runtime.gesture_preview()
                jpeg = preview.get("jpeg")
                frame_id = int(preview.get("frame_id", 0))
                if jpeg and frame_id != last_frame_id:
                    last_frame_id = frame_id
                    yield (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        b"Cache-Control: no-store\r\n\r\n"
                        + jpeg + b"\r\n"
                    )
                else:
                    time.sleep(0.025)
        except (GeneratorExit, BrokenPipeError, ConnectionResetError):
            return
    return StreamingResponse(
        frames(),
        media_type="multipart/x-mixed-replace; boundary=frame",
        headers={
            "Cache-Control": "no-store, no-cache, must-revalidate",
            "Pragma": "no-cache",
            "X-Accel-Buffering": "no",
        },
    )


@app.post("/api/connect")
def connect(config: ConnectConfig) -> dict[str, bool]:
    try:
        # Pydantic v2 and v1 compatibility.
        values = (
            config.model_dump()
            if hasattr(config, "model_dump")
            else config.dict()
        )
        values["show_gesture_window"] = False
        values["gesture_preview_quality"] = 82
        values.setdefault("teleop_tx_hz", 15.0)
        runtime.connect(values)
    except RuntimeError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    return {"ok": True}


@app.post("/api/disconnect")
def disconnect() -> dict[str, bool]:
    runtime.disconnect()
    return {"ok": True}


@app.post("/api/mode/{command}")
def set_mode(command: str) -> dict[str, bool]:
    command = command.upper()
    valid_commands = {"TELEOP", "GESTURE", "AUTO", "HOLD", "RECENTER"}
    if command not in valid_commands:
        raise HTTPException(status_code=400, detail="无效控制指令")

    try:
        runtime.command(command)
    except RuntimeError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    return {"ok": True}


@app.on_event("shutdown")
def shutdown_event() -> None:
    runtime.close()


if DIST.exists():
    assets = DIST / "assets"
    if assets.exists():
        app.mount("/assets", StaticFiles(directory=assets), name="assets")


@app.get("/{path:path}")
def frontend(path: str):
    if not DIST.exists():
        raise HTTPException(
            status_code=503,
            detail=(
                "前端尚未构建。开发模式请访问 http://127.0.0.1:5173；"
                "生产模式请在 so101-control-ui 目录执行 "
                "corepack pnpm run build。"
            ),
        )

    requested = (DIST / path).resolve()
    dist_resolved = DIST.resolve()

    if (
        path
        and requested.is_file()
        and (
            requested == dist_resolved
            or dist_resolved in requested.parents
        )
    ):
        return FileResponse(requested)

    return FileResponse(DIST / "index.html")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="SO-101 Web backend")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument(
        "--no-browser",
        action="store_true",
        help="启动时不自动打开浏览器",
    )
    return parser.parse_args()


def main() -> None:
    import uvicorn

    args = parse_args()
    url = f"http://{args.host}:{args.port}"

    print("SO-101 Web 后端正在启动")
    print(f"API: {url}/api/health")
    print(f"前端开发地址: http://127.0.0.1:5173")
    print(f"生产构建目录: {DIST}")

    if not args.no_browser:
        threading.Timer(
            1.0,
            lambda: webbrowser.open("http://127.0.0.1:5173"),
        ).start()

    uvicorn.run(
        app,
        host=args.host,
        port=args.port,
        log_level="info",
    )


if __name__ == "__main__":
    main()
