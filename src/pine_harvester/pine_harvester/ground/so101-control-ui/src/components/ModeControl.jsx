import { useState } from 'react'
import { Button } from './Button.jsx'
import { Card, CardHeader } from './Card.jsx'
import { Icon } from './Icon.jsx'
import { sendMode } from '../lib/api.js'

const MODES = [
  { command: 'TELEOP', label: '主从跟随', description: '读取领导臂关节并同步从臂', key: 'T', icon: 'radio' },
  { command: 'GESTURE', label: '手势控制', description: '使用摄像头输入控制末端运动', key: 'G', icon: 'hand' },
  { command: 'AUTO', label: '自主抓取', description: '执行机载自主任务流程', key: 'A', icon: 'cpu' },
  { command: 'RECENTER', label: '重新定中', description: '重置手势基准与机械臂零位', key: 'C', icon: 'target' },
]

function ModeButton({ item, active, disabled, onClick }) {
  return (
    <button
      type="button"
      disabled={disabled}
      onClick={onClick}
      className={`group min-h-24 rounded-[10px] border p-4 text-left shadow-[var(--shadow-default)] transition-[transform,box-shadow,background-color,border-color] duration-150 hover:-translate-y-px hover:shadow-[var(--shadow-hover)] disabled:cursor-not-allowed disabled:opacity-40 disabled:hover:translate-y-0 ${
        active
          ? 'border-[var(--color-brand-600)] bg-[var(--color-brand-50)] dark:bg-[color-mix(in_srgb,var(--color-brand-600)_12%,transparent)]'
          : 'border-[var(--border-default)] bg-[var(--surface-card)] hover:border-[var(--border-strong)] hover:bg-[var(--surface-hover)]'
      }`}
    >
      <div className="flex items-center justify-between gap-3">
        <span className={active ? 'text-[var(--color-brand-600)] dark:text-[var(--color-brand-300)]' : 'text-[var(--text-secondary)]'}>
          <Icon name={item.icon} size={18} />
        </span>
        <kbd className="rounded-[10px] border border-[var(--border-default)] bg-[var(--surface-subtle)] px-2 py-1 text-[12px] font-semibold leading-[1.6] text-[var(--text-tertiary)]">
          {item.key}
        </kbd>
      </div>
      <p className="mt-3 text-[16px] font-semibold leading-[1.4] text-[var(--text-primary)]">{item.label}</p>
      <p className="mt-1 text-[12px] leading-[1.6] text-[var(--text-secondary)]">{item.description}</p>
    </button>
  )
}

export function ModeControl({ state, onChanged }) {
  const [pending, setPending] = useState('')
  const [error, setError] = useState('')
  const connected = state.connected
  const mode = state.mode || 'DISCONNECTED'

  async function request(command) {
    if (!connected || pending) return
    if (command === 'AUTO' && !window.confirm('确认机械臂周围安全，并进入 AUTO 模式？')) return
    setPending(command)
    setError('')
    try {
      await sendMode(command)
      await onChanged()
    } catch (requestError) {
      setError(requestError.message)
    } finally {
      setPending('')
    }
  }

  return (
    <Card className="h-full">
      <CardHeader eyebrow="Control" title="运行模式" meta="模式切换前请确认机械臂工作空间安全" />
      <div className="p-6">
        <div className="rounded-[10px] border border-[var(--border-default)] bg-[var(--surface-subtle)] p-6">
          <p className="text-[12px] font-semibold uppercase leading-[1.6] tracking-[0.12em] text-[var(--text-tertiary)]">
            当前模式
          </p>
          <div className="mt-2 flex flex-wrap items-end justify-between gap-4">
            <p className="text-[32px] font-semibold leading-[1.4] tracking-[-0.03em] text-[var(--text-primary)]">{mode}</p>
            <div className="text-right">
              <p className="text-[12px] leading-[1.6] text-[var(--text-tertiary)]">数据序列</p>
              <p className="tabular text-[16px] font-semibold leading-[1.4] text-[var(--text-primary)]">{state.sequence ?? 0}</p>
            </div>
          </div>
        </div>

        <div className="mt-6 grid grid-cols-1 gap-4 sm:grid-cols-2">
          {MODES.map((item) => (
            <ModeButton
              key={item.command}
              item={item}
              active={mode === item.command || (item.command === 'GESTURE' && mode === 'GESTURE_WAIT')}
              disabled={!connected || Boolean(pending)}
              onClick={() => request(item.command)}
            />
          ))}
        </div>

        <Button
          className="mt-6 w-full"
          icon="stop"
          variant="strong"
          disabled={!connected || Boolean(pending)}
          onClick={() => request('HOLD')}
        >
          HOLD / 安全停止 <span className="text-[12px] font-medium opacity-70">H</span>
        </Button>

        {error ? (
          <p role="alert" className="mt-4 text-[14px] leading-[1.6] text-[var(--text-secondary)]">{error}</p>
        ) : null}
      </div>
    </Card>
  )
}
