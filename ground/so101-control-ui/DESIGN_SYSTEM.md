# SO-101 Control UI — Design System

## 页面结构

1. **全局顶栏**：产品名称、用途说明、设备/API 状态、主题切换。
2. **运行模式卡**：当前模式为唯一主信息；数据序列和连接条件为辅信息。提供 TELEOP、GESTURE、AUTO、RECENTER 与 HOLD。
3. **设备连接卡**：常用串口参数默认展开，高级参数按需展开，降低首屏密度。
4. **关节遥测卡**：六个关节独立小卡；每张仅包含主角度值、中文关节名、通道/内部名两项辅信息。
5. **系统日志卡**：最近事件、记录数量与自动跟随状态。
6. **底部快捷栏**：持续可见的键盘操作提示。

桌面端使用 12 栏结构，控制区 7 栏、配置区 5 栏；遥测与日志沿用相同占比。小屏自动单栏。

## Design Tokens

| 分类 | Token | Light | Dark | 用途 |
|---|---|---:|---:|---|
| Brand | `brand-50` | `#EFF6FF` | 同值 | 选中背景 |
| Brand | `brand-600` | `#2563EB` | 同值 | 唯一品牌主色、主按钮、焦点 |
| Brand | `brand-700` | `#1D4ED8` | 同值 | 主按钮 hover |
| Gray | `gray-0` | `#FFFFFF` | — | 浅色卡片 |
| Gray | `gray-50` | `#F7F8FA` | — | 浅色画布 |
| Gray | `gray-100` | `#F0F2F5` | — | 次级表面 |
| Gray | `gray-200` | `#E3E7ED` | — | 默认边框 |
| Gray | `gray-300` | `#CFD5DE` | — | 强边框 |
| Gray | `gray-400` | `#9AA4B2` | — | 非活动状态 |
| Gray | `gray-500` | `#697386` | — | 第三级文字 |
| Gray | `gray-600` | `#4B5565` | — | 第二级文字 |
| Gray | `gray-900` | `#111827` | — | 主文字/强按钮 |
| Surface | `surface-canvas` | `#F7F8FA` | `#0B0F17` | 页面背景 |
| Surface | `surface-card` | `#FFFFFF` | `#111722` | 卡片、输入框 |
| Surface | `surface-subtle` | `#F0F2F5` | `#171E2B` | 次级区块 |
| Surface | `surface-hover` | `#EDF1F6` | `#1D2634` | hover |
| Text | `text-primary` | `#111827` | `#F3F5F7` | 标题、主数据 |
| Text | `text-secondary` | `#4B5565` | `#B5BDC9` | 正文、说明 |
| Text | `text-tertiary` | `#697386` | `#8D98A8` | 标签、时间 |
| Border | `border-default` | `#E3E7ED` | `#283242` | 全局 1px 边框 |
| Border | `border-strong` | `#CFD5DE` | `#3A4658` | hover/提示边框 |
| Spacing | `space-1` | `4px` | `4px` | 紧凑间隔 |
| Spacing | `space-2` | `8px` | `8px` | 图标与文字 |
| Spacing | `space-3` | `12px` | `12px` | 表单内部 |
| Spacing | `space-4` | `16px` | `16px` | 组件间距 |
| Spacing | `space-6` | `24px` | `24px` | 卡片内边距/栅格 |
| Spacing | `space-8` | `32px` | `32px` | 页面大间距 |
| Spacing | `space-10` | `40px` | `40px` | 控件高度/区块 |
| Spacing | `space-12` | `48px` | `48px` | 大区块间隔 |
| Radius | `radius` | `10px` | `10px` | 全局唯一圆角 |
| Shadow | `shadow-default` | `0 1px 2px rgba(15,23,42,.06)` | `0 1px 2px rgba(0,0,0,.20)` | 默认层级 |
| Shadow | `shadow-hover` | `0 4px 12px rgba(15,23,42,.08)` | `0 4px 12px rgba(0,0,0,.28)` | hover 层级 |
| Type | `display` | `32px / 1.4` | 同值 | 当前模式 |
| Type | `h1` | `24px / 1.4` | 同值 | 页面标题、遥测值 |
| Type | `h2` | `20px / 1.4` | 同值 | 卡片标题 |
| Type | `h3` | `16px / 1.4` | 同值 | 控件标题 |
| Type | `body` | `14px / 1.6` | 同值 | 正文、按钮、输入 |
| Type | `caption` | `12px / 1.6` | 同值 | 标签、元数据 |
| Motion | `micro` | `150ms` | `150ms` | 仅 hover/focus |

## 组件拆分

| 组件 | 职责 |
|---|---|
| `App` | 页面编排、快捷键、全局状态 |
| `Header` | 品牌、连接状态、主题切换 |
| `ConnectionPanel` | 参数持久化、连接与断开 |
| `ModeControl` | 模式状态、模式切换、HOLD |
| `TelemetryGrid` | 六关节实时角度展示 |
| `ActivityLog` | 日志、错误、自动跟随 |
| `ShortcutBar` | 键盘操作提示 |
| `Card / CardHeader` | 统一卡片容器与标题结构 |
| `Button / IconButton` | 四种克制按钮层级 |
| `StatusPill` | 在线、离线、连接中状态 |
| `Icon` | 无外部依赖的线性图标集 |
| `useTheme` | light/dark 与本地持久化 |
| `useControlState` | 500ms 状态轮询与 API 可用性 |
| `api.js` | FastAPI 请求、统一错误处理 |
