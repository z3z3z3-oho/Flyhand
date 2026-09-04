import { useEffect, useMemo, useState } from 'react'
import { Card, CardHeader } from './Card.jsx'
import { Button } from './Button.jsx'
import { Icon } from './Icon.jsx'

const ACTIVE_MODES = new Set(['GESTURE', 'GESTURE_WAIT'])

export function GesturePreview({ mode, connected, preview, buildId }) {
  const active = connected && ACTIVE_MODES.has(mode)
  const [streamVersion, setStreamVersion] = useState(0)
  const [loaded, setLoaded] = useState(false)
  const [failed, setFailed] = useState(false)

  useEffect(() => {
    setLoaded(false)
    setFailed(false)
    if (active) setStreamVersion((value) => value + 1)
  }, [active])

  const streamUrl = useMemo(
    () => `/api/gesture/stream?v=${streamVersion}`,
    [streamVersion],
  )
  const status = preview?.status || (active ? '等待第一帧' : '手势模式未启动')
  const version = preview?.build_id || buildId || '—'

  return (
    <Card className="overflow-hidden">
      <CardHeader
        eyebrow="Gesture camera"
        title="手势识别画面"
        meta="肩关节原点、运动速度跟随、当前姿态自动零位。"
        action={
          <Button
            variant="subtle"
            size="sm"
            disabled={!active}
            onClick={() => {
              setLoaded(false)
              setFailed(false)
              setStreamVersion((value) => value + 1)
            }}
          >
            <Icon name="refresh" size={16} />
            重连画面
          </Button>
        }
      />
      <div className="p-6">
        <div className="relative aspect-video overflow-hidden rounded-[10px] border border-[var(--border-default)] bg-[var(--surface-subtle)]">
          {active ? (
            <img
              key={streamVersion}
              src={streamUrl}
              alt="SO-101 手势识别画面"
              className={`h-full w-full object-contain transition-opacity duration-150 ${loaded ? 'opacity-100' : 'opacity-0'}`}
              onLoad={() => { setLoaded(true); setFailed(false) }}
              onError={() => { setLoaded(false); setFailed(true) }}
            />
          ) : null}
          {!active || !loaded ? (
            <div className="absolute inset-0 flex items-center justify-center px-6 text-center">
              <div className="max-w-[420px]">
                <p className="text-[16px] font-semibold leading-[1.4] text-[var(--text-primary)]">
                  {active ? (failed ? '预览连接中断' : '正在启动肩部与手部识别') : '进入 GESTURE 后显示画面'}
                </p>
                <p className="mt-2 text-[14px] leading-[1.6] text-[var(--text-secondary)]">
                  画面必须包含肩膀、手肘、手腕和手掌。红点为肩关节原点。
                </p>
              </div>
            </div>
          ) : null}
          {active && loaded ? (
            <div className="absolute inset-x-0 bottom-0 bg-black/70 px-4 py-3">
              <p className="truncate text-[12px] font-medium leading-[1.6] text-white">{status}</p>
            </div>
          ) : null}
        </div>
        <div className="mt-4 grid grid-cols-1 gap-4 sm:grid-cols-2">
          <div className="rounded-[10px] border border-[var(--border-default)] bg-[var(--surface-subtle)] px-4 py-3">
            <p className="text-[12px] leading-[1.6] text-[var(--text-tertiary)]">控制构建</p>
            <p className="mt-1 truncate text-[14px] font-semibold leading-[1.6] text-[var(--text-primary)]">{version}</p>
          </div>
          <div className="rounded-[10px] border border-[var(--border-default)] bg-[var(--surface-subtle)] px-4 py-3">
            <p className="text-[12px] leading-[1.6] text-[var(--text-tertiary)]">画面序号</p>
            <p className="mt-1 text-[14px] font-semibold leading-[1.6] text-[var(--text-primary)]">{preview?.frame_id || '—'}</p>
          </div>
        </div>
      </div>
    </Card>
  )
}
