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

当前目录名为 `so101-control-ui/`，输出文件位于 `so101-control-ui/dist/`，由现有 `web_server.py` 直接托管。

```text
ground/
├─ web_server.py
└─ so101-control-ui/
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

## 说明

当前后端已经保留前端提交的 `hand_model`，页面中的模型路径输入会生效。
