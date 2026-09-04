import { useEffect, useMemo, useRef, useState } from 'react'
import { Card, CardHeader } from './Card.jsx'
import { Icon } from './Icon.jsx'

function normalizeLevel(level = '') {
  const value = level.toLowerCase()
  if (value.includes('error') || value.includes('warn')) return '注意'
  if (value.includes('feedback')) return '机载'
  return '系统'
}

export function ActivityLog({ logs = [], error }) {
  const [follow, setFollow] = useState(true)
  const containerRef = useRef(null)
  const normalizedLogs = useMemo(() => logs.slice(-80), [logs])

  useEffect(() => {
    if (!follow || !containerRef.current) return
    containerRef.current.scrollTop = containerRef.current.scrollHeight
  }, [follow, normalizedLogs])

  return (
    <Card className="min-h-[360px]">
      <CardHeader
        eyebrow="Activity"
        title="系统日志"
        meta={`${normalizedLogs.length} 条最近记录`}
        action={
          <button
            type="button"
            onClick={() => setFollow((value) => !value)}
            className="h-8 rounded-[10px] border border-[var(--border-default)] bg-[var(--surface-subtle)] px-3 text-[12px] font-semibold leading-[1.6] text-[var(--text-secondary)] transition-colors duration-150 hover:bg-[var(--surface-hover)] hover:text-[var(--text-primary)]"
          >
            {follow ? '自动跟随' : '已暂停'}
          </button>
        }
      />
      <div ref={containerRef} className="scrollbar-thin max-h-[416px] overflow-y-auto p-4">
        {error ? (
          <div className="mb-3 flex gap-3 rounded-[10px] border border-[var(--border-strong)] bg-[var(--surface-subtle)] p-3">
            <Icon name="alert" size={16} className="mt-1 shrink-0 text-[var(--text-secondary)]" />
            <p className="text-[14px] leading-[1.6] text-[var(--text-primary)]">{error}</p>
          </div>
        ) : null}

        {normalizedLogs.length ? (
          <div className="space-y-1">
            {normalizedLogs.map((log) => (
              <div
                key={log.id || `${log.time}-${log.message}`}
                className="grid grid-cols-[64px_48px_minmax(0,1fr)] gap-3 rounded-[10px] px-3 py-2 text-[12px] leading-[1.6] transition-colors duration-150 hover:bg-[var(--surface-subtle)]"
              >
                <time className="tabular text-[var(--text-tertiary)]">{log.time || '--:--:--'}</time>
                <span className="font-semibold text-[var(--text-secondary)]">{normalizeLevel(log.level)}</span>
                <span className="break-words text-[var(--text-primary)]">{log.message}</span>
              </div>
            ))}
          </div>
        ) : (
          <div className="flex min-h-48 flex-col items-center justify-center gap-3 text-center">
            <Icon name="terminal" size={24} className="text-[var(--text-tertiary)]" />
            <div>
              <p className="text-[14px] font-semibold leading-[1.6] text-[var(--text-primary)]">暂无日志</p>
              <p className="mt-1 text-[12px] leading-[1.6] text-[var(--text-tertiary)]">连接设备后，系统事件会显示在这里。</p>
            </div>
          </div>
        )}
      </div>
    </Card>
  )
}
