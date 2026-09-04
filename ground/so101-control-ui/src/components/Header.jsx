import { IconButton } from './Button.jsx'
import { StatusPill } from './StatusPill.jsx'

export function Header({ theme, toggleTheme, connected, connecting, apiOnline }) {
  const statusLabel = !apiOnline
    ? 'API 离线'
    : connecting
      ? '连接中'
      : connected
        ? '设备在线'
        : '未连接'

  return (
    <header className="border-b border-[var(--border-default)] bg-[color-mix(in_srgb,var(--surface-canvas)_92%,transparent)] backdrop-blur">
      <div className="mx-auto flex max-w-[1440px] items-center justify-between gap-4 px-6 py-4 lg:px-8">
        <div className="min-w-0">
          <div className="flex items-center gap-3">
            <div className="flex h-10 w-10 shrink-0 items-center justify-center rounded-[10px] border border-[var(--color-brand-600)] bg-[var(--color-brand-600)] text-[14px] font-bold leading-[1.6] text-white shadow-[var(--shadow-default)]">
              S1
            </div>
            <div className="min-w-0">
              <h1 className="truncate text-[24px] font-semibold leading-[1.4] tracking-[-0.02em] text-[var(--text-primary)]">
                SO-101 控制中心
              </h1>
              <p className="text-[12px] leading-[1.6] text-[var(--text-tertiary)]">
                本地机械臂控制与遥测
              </p>
            </div>
          </div>
        </div>
        <div className="flex shrink-0 items-center gap-3">
          <StatusPill label={statusLabel} active={connected && apiOnline} pulse={connecting} />
          <IconButton
            label={theme === 'dark' ? '切换到浅色主题' : '切换到深色主题'}
            icon={theme === 'dark' ? 'sun' : 'moon'}
            onClick={toggleTheme}
          />
        </div>
      </div>
    </header>
  )
}
