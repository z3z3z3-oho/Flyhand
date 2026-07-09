# SO-101 Control UI

面向现有 FastAPI 控制后端的 React + Tailwind 控制中心。保留以下 API：

- `GET /api/state`
- `POST /api/connect`
- `POST /api/disconnect`
- `POST /api/mode/{command}`

## 本地开发

要求 Node.js 20.19+ 或 22.12+。

```bash
npm install
npm run dev
```

Vite 会把 `/api` 代理到 `http://127.0.0.1:8765`。

## 构建并交给现有 FastAPI 托管

```bash
npm run build
```

将本项目放在 `web/` 目录时，输出文件会位于 `web/dist/`，与现有 `web_server.py` 的静态目录约定一致。

```text
ground/
├─ web_server.py
└─ web/
   ├─ package.json
   ├─ src/
   └─ dist/
```

随后启动原有服务：

```bash
python web_server.py
```

浏览器访问 `http://127.0.0.1:8765`。

## 交互

- `T` 或 `Space`：TELEOP
- `G`：GESTURE
- `A`：AUTO
- `C`：RECENTER
- `H`：HOLD

输入框获得焦点时不会触发快捷键。

## 现有后端的一处必要修正

当前 `web_server.py` 的 `/api/connect` 会覆盖前端提交的 `hand_model`，因此页面里的模型路径输入不会生效。将：

```python
values = config.model_dump()
values["hand_model"] = r"C:\\Users\\Lenovo\\Desktop\\SO101_3_modes\\ground\\hand_landmarker.task"
```

改为：

```python
values = config.model_dump()
```

即可让 UI 配置真正传入控制进程。
