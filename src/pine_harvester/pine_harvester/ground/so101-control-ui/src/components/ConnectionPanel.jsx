import { useEffect, useState } from 'react'
import { Button } from './Button.jsx'
import { Card, CardHeader } from './Card.jsx'
import { connectController, disconnectController } from '../lib/api.js'

const DEFAULTS = {
  leader_port: 'COM7',
  radio_port: 'COM10',
  leader_id: 'blue',
  camera_index: 0,
  baudrate: 57600,
  hz: 30,
  gesture_speed: 0.7,
  hand_model: 'D:\\path\\to\\hand_landmarker.task',
}

function Field({ label, hint, ...props }) {
  return (
    <label className="block min-w-0">
      <span className="mb-2 block text-[12px] font-semibold leading-[1.6] text-[var(--text-secondary)]">
        {label}
      </span>
      <input
        className="h-10 w-full rounded-[10px] border border-[var(--border-default)] bg-[var(--surface-card)] px-3 text-[14px] leading-[1.6] text-[var(--text-primary)] shadow-[var(--shadow-default)] transition-[border-color,box-shadow,background-color] duration-150 placeholder:text-[var(--text-tertiary)] hover:border-[var(--border-strong)] focus:border-[var(--color-brand-600)]"
        {...props}
      />
      {hint ? (
        <span className="mt-1 block text-[12px] leading-[1.6] text-[var(--text-tertiary)]">{hint}</span>
      ) : null}
    </label>
  )
}

export function ConnectionPanel({ connected, connecting, onChanged }) {
  const [config, setConfig] = useState(() => {
    const saved = localStorage.getItem('so101-connect-config')
    return saved ? { ...DEFAULTS, ...JSON.parse(saved) } : DEFAULTS
  })
  const [expanded, setExpanded] = useState(false)
  const [pending, setPending] = useState(false)
  const [error, setError] = useState('')

  useEffect(() => {
    localStorage.setItem('so101-connect-config', JSON.stringify(config))
  }, [config])

  const update = (key) => (event) => {
    const value = event.target.type === 'number' ? Number(event.target.value) : event.target.value
    setConfig((current) => ({ ...current, [key]: value }))
  }

  async function handleConnection() {
    setError('')
    setPending(true)
    try {
      if (connected || connecting) {
        await disconnectController()
      } else {
        if (!config.leader_port.trim() || !config.radio_port.trim()) {
          throw new Error('请填写主臂串口和数传串口。')
        }
        await connectController(config)
      }
      await onChanged()
    } catch (requestError) {
      setError(requestError.message)
    } finally {
      setPending(false)
    }
  }

  return (
    <Card>
      <CardHeader
        eyebrow="Connection"
        title="设备连接"
        meta={connected ? `${config.leader_port} · ${config.radio_port}` : '配置本地主臂、数传与识别设备'}
      />
      <div className="p-6">
        <div className="grid grid-cols-1 gap-4 sm:grid-cols-2">
          <Field label="主臂串口" value={config.leader_port} onChange={update('leader_port')} />
          <Field label="数传串口" value={config.radio_port} onChange={update('radio_port')} />
          <Field label="校准 ID" value={config.leader_id} onChange={update('leader_id')} />
          <Field
            label="摄像头索引"
            min="0"
            max="16"
            type="number"
            value={config.camera_index}
            onChange={update('camera_index')}
          />
        </div>

        <button
          type="button"
          className="mt-4 flex w-full items-center justify-between rounded-[10px] border border-[var(--border-default)] bg-[var(--surface-subtle)] px-4 py-3 text-left text-[14px] font-semibold leading-[1.6] text-[var(--text-secondary)] transition-[background-color,color,border-color] duration-150 hover:border-[var(--border-strong)] hover:bg-[var(--surface-hover)] hover:text-[var(--text-primary)]"
          onClick={() => setExpanded((value) => !value)}
          aria-expanded={expanded}
        >
          高级参数
          <span className="text-[12px] font-medium">{expanded ? '收起' : '展开'}</span>
        </button>

        {expanded ? (
          <div className="mt-4 grid grid-cols-1 gap-4 sm:grid-cols-2">
            <Field label="数传波特率" type="number" value={config.baudrate} onChange={update('baudrate')} />
            <Field label="控制频率" type="number" min="1" max="50" value={config.hz} onChange={update('hz')} />
            <Field
              label="手势速度"
              hint="建议首次测试使用 0.30–0.50"
              type="number"
              min="0.1"
              max="1"
              step="0.05"
              value={config.gesture_speed}
              onChange={update('gesture_speed')}
            />
            <Field label="手部模型路径" value={config.hand_model} onChange={update('hand_model')} />
          </div>
        ) : null}

        {error ? (
          <p role="alert" className="mt-4 rounded-[10px] border border-[var(--border-strong)] bg-[var(--surface-subtle)] px-4 py-3 text-[14px] leading-[1.6] text-[var(--text-primary)]">
            {error}
          </p>
        ) : null}

        <Button
          className="mt-6 w-full"
          icon={connected ? 'unlink' : 'link'}
          variant={connected ? 'secondary' : 'primary'}
          disabled={pending}
          onClick={handleConnection}
        >
          {pending || connecting ? '处理中…' : connected ? '断开连接' : '连接设备'}
        </Button>
      </div>
    </Card>
  )
}
