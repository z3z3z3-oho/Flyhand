export function StatusPill({ label, active = false, pulse = false }) {
  return (
    <span
      className={`inline-flex h-8 items-center gap-2 rounded-[10px] border px-3 text-[12px] font-semibold leading-[1.6] ${
        active
          ? 'border-[color-mix(in_srgb,var(--color-brand-600)_30%,var(--border-default))] bg-[var(--color-brand-50)] text-[var(--color-brand-700)] dark:bg-[color-mix(in_srgb,var(--color-brand-600)_16%,transparent)] dark:text-[var(--color-brand-300)]'
          : 'border-[var(--border-default)] bg-[var(--surface-subtle)] text-[var(--text-secondary)]'
      }`}
    >
      <span
        className={`h-2 w-2 rounded-full ${
          active ? 'bg-[var(--color-brand-600)]' : 'bg-[var(--color-gray-400)]'
        } ${pulse ? 'animate-pulse' : ''}`}
      />
      {label}
    </span>
  )
}
