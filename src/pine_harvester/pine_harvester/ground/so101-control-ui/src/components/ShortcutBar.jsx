const SHORTCUTS = [
  ['T / Space', '主从跟随'],
  ['G', '手势控制'],
  ['A', '自主抓取'],
  ['C', '重新定中'],
  ['H', '安全停止'],
]

export function ShortcutBar() {
  return (
    <footer className="border-t border-[var(--border-default)] bg-[var(--surface-card)]">
      <div className="mx-auto flex max-w-[1440px] flex-wrap items-center justify-between gap-4 px-6 py-4 lg:px-8">
        <p className="text-[12px] leading-[1.6] text-[var(--text-tertiary)]">键盘快捷操作</p>
        <div className="flex flex-wrap items-center gap-4">
          {SHORTCUTS.map(([key, label]) => (
            <span key={key} className="inline-flex items-center gap-2 text-[12px] leading-[1.6] text-[var(--text-secondary)]">
              <kbd className="rounded-[10px] border border-[var(--border-default)] bg-[var(--surface-subtle)] px-2 py-1 font-semibold text-[var(--text-primary)]">{key}</kbd>
              {label}
            </span>
          ))}
        </div>
      </div>
    </footer>
  )
}
