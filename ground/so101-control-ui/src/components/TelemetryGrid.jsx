import { Card, CardHeader } from './Card.jsx'
import { Icon } from './Icon.jsx'

const JOINTS = [
  ['shoulder_pan', '底座旋转'],
  ['shoulder_lift', '肩部抬升'],
  ['elbow_flex', '肘部弯曲'],
  ['wrist_flex', '腕部俯仰'],
  ['wrist_roll', '腕部旋转'],
  ['gripper', '夹爪'],
]

function JointCard({ index, name, label, value }) {
  const safeValue = Number.isFinite(Number(value)) ? Number(value) : 0
  return (
    <article className="rounded-[10px] border border-[var(--border-default)] bg-[var(--surface-card)] p-4 shadow-[var(--shadow-default)] transition-[transform,box-shadow,border-color] duration-150 hover:-translate-y-px hover:border-[var(--border-strong)] hover:shadow-[var(--shadow-hover)]">
      <div className="flex items-center justify-between gap-3">
        <span className="text-[12px] font-semibold leading-[1.6] text-[var(--text-secondary)]">{label}</span>
        <span className="text-[12px] leading-[1.6] text-[var(--text-tertiary)]">J{index + 1}</span>
      </div>
      <p className="tabular mt-3 text-[24px] font-semibold leading-[1.4] tracking-[-0.02em] text-[var(--text-primary)]">
        {safeValue.toFixed(2)}°
      </p>
      <p className="mt-1 truncate text-[12px] leading-[1.6] text-[var(--text-tertiary)]">{name}</p>
    </article>
  )
}

export function TelemetryGrid({ values = [], lastUpdated }) {
  const time = lastUpdated
    ? lastUpdated.toLocaleTimeString('zh-CN', { hour12: false })
    : '--:--:--'

  return (
    <Card>
      <CardHeader
        eyebrow="Telemetry"
        title="关节遥测"
        meta="实时读取领导臂关节角度"
        action={<span className="inline-flex items-center gap-2 text-[12px] leading-[1.6] text-[var(--text-tertiary)]"><Icon name="activity" size={16} />{time}</span>}
      />
      <div className="grid grid-cols-1 gap-4 p-6 sm:grid-cols-2 xl:grid-cols-3">
        {JOINTS.map(([name, label], index) => (
          <JointCard key={name} index={index} name={name} label={label} value={values[index]} />
        ))}
      </div>
    </Card>
  )
}
