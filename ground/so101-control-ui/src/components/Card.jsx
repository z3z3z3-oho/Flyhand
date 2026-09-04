export function Card({ children, className = '', as: Element = 'section' }) {
  return (
    <Element
      className={`rounded-[10px] border border-[var(--border-default)] bg-[var(--surface-card)] shadow-[var(--shadow-default)] ${className}`}
    >
      {children}
    </Element>
  )
}

export function CardHeader({ eyebrow, title, meta, action }) {
  return (
    <div className="flex items-start justify-between gap-4 border-b border-[var(--border-default)] px-6 py-4">
      <div className="min-w-0">
        {eyebrow ? (
          <p className="text-[12px] font-semibold uppercase leading-[1.6] tracking-[0.12em] text-[var(--text-tertiary)]">
            {eyebrow}
          </p>
        ) : null}
        <h2 className="mt-1 text-[20px] font-semibold leading-[1.4] text-[var(--text-primary)]">
          {title}
        </h2>
        {meta ? (
          <p className="mt-1 text-[14px] leading-[1.6] text-[var(--text-secondary)]">{meta}</p>
        ) : null}
      </div>
      {action}
    </div>
  )
}
