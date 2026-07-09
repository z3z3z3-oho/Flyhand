import { Icon } from './Icon.jsx'

const VARIANTS = {
  primary:
    'border-[var(--color-brand-600)] bg-[var(--color-brand-600)] text-white hover:border-[var(--color-brand-700)] hover:bg-[var(--color-brand-700)]',
  secondary:
    'border-[var(--border-default)] bg-[var(--surface-card)] text-[var(--text-primary)] hover:border-[var(--border-strong)] hover:bg-[var(--surface-hover)]',
  subtle:
    'border-transparent bg-transparent text-[var(--text-secondary)] hover:bg-[var(--surface-hover)] hover:text-[var(--text-primary)]',
  strong:
    'border-[var(--text-primary)] bg-[var(--text-primary)] text-[var(--surface-card)] hover:opacity-90',
}

export function Button({
  children,
  variant = 'secondary',
  icon,
  className = '',
  type = 'button',
  ...props
}) {
  return (
    <button
      type={type}
      className={`inline-flex h-10 items-center justify-center gap-2 rounded-[10px] border px-4 text-[14px] font-semibold leading-[1.6] shadow-[var(--shadow-default)] transition-[transform,box-shadow,background-color,border-color,color] duration-150 hover:-translate-y-px hover:shadow-[var(--shadow-hover)] disabled:cursor-not-allowed disabled:opacity-40 disabled:hover:translate-y-0 disabled:hover:shadow-[var(--shadow-default)] ${VARIANTS[variant]} ${className}`}
      {...props}
    >
      {icon ? <Icon name={icon} size={16} /> : null}
      {children}
    </button>
  )
}

export function IconButton({ label, icon, ...props }) {
  return (
    <button
      aria-label={label}
      title={label}
      className="inline-flex h-10 w-10 items-center justify-center rounded-[10px] border border-[var(--border-default)] bg-[var(--surface-card)] text-[var(--text-secondary)] shadow-[var(--shadow-default)] transition-[transform,box-shadow,background-color,color] duration-150 hover:-translate-y-px hover:bg-[var(--surface-hover)] hover:text-[var(--text-primary)] hover:shadow-[var(--shadow-hover)]"
      {...props}
    >
      <Icon name={icon} size={18} />
    </button>
  )
}
