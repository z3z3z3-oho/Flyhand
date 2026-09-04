const PATHS = {
  sun: <><circle cx="12" cy="12" r="4"/><path d="M12 2v2M12 20v2M4.93 4.93l1.42 1.42M17.66 17.66l1.41 1.41M2 12h2M20 12h2M4.93 19.07l1.42-1.42M17.66 6.34l1.41-1.41"/></>,
  moon: <path d="M20.3 15.4A8.5 8.5 0 0 1 8.6 3.7 8.5 8.5 0 1 0 20.3 15.4Z"/>,
  link: <><path d="M10 13a5 5 0 0 0 7.1.1l2-2a5 5 0 0 0-7.1-7.1l-1.1 1.1"/><path d="M14 11a5 5 0 0 0-7.1-.1l-2 2A5 5 0 0 0 12 20l1.1-1.1"/></>,
  unlink: <><path d="m2 2 20 20"/><path d="M9.9 4.2A5 5 0 0 1 17.1 4l2 2a5 5 0 0 1 .7 6.2M14.1 19.8A5 5 0 0 1 6.9 20l-2-2a5 5 0 0 1-.7-6.2"/></>,
  sliders: <><path d="M4 21v-7M4 10V3M12 21v-9M12 8V3M20 21v-5M20 12V3"/><path d="M1 14h6M9 8h6M17 16h6"/></>,
  radio: <><path d="M4.9 19.1a10 10 0 0 1 0-14.2M8.5 15.5a5 5 0 0 1 0-7M19.1 4.9a10 10 0 0 1 0 14.2M15.5 8.5a5 5 0 0 1 0 7"/><circle cx="12" cy="12" r="2"/></>,
  hand: <path d="M18 11V6a2 2 0 0 0-4 0v4-6a2 2 0 0 0-4 0v6-4a2 2 0 0 0-4 0v6l-1.2-1.2a2 2 0 0 0-2.8 2.8l5.5 5.5A6 6 0 0 0 11.7 21H14a6 6 0 0 0 6-6v-4a2 2 0 0 0-2-2Z"/>,
  cpu: <><rect x="4" y="4" width="16" height="16" rx="2"/><rect x="9" y="9" width="6" height="6"/><path d="M9 1v3M15 1v3M9 20v3M15 20v3M20 9h3M20 14h3M1 9h3M1 14h3"/></>,
  target: <><circle cx="12" cy="12" r="8"/><circle cx="12" cy="12" r="3"/><path d="M12 2v3M12 19v3M2 12h3M19 12h3"/></>,
  stop: <rect x="6" y="6" width="12" height="12" rx="1"/>,
  activity: <path d="M3 12h4l2-8 4 16 2-8h6"/>,
  terminal: <><path d="m4 17 6-6-6-6"/><path d="M12 19h8"/></>,
  refresh: <><path d="M20 11a8.1 8.1 0 0 0-15.5-2M4 4v5h5"/><path d="M4 13a8.1 8.1 0 0 0 15.5 2M20 20v-5h-5"/></>,
  chevron: <path d="m9 18 6-6-6-6"/>,
  check: <path d="m5 12 4 4L19 6"/>,
  alert: <><path d="M12 3 2.8 19h18.4L12 3Z"/><path d="M12 9v4M12 17h.01"/></>,
  command: <><path d="M18 9a3 3 0 1 0-3-3v12a3 3 0 1 0 3-3H6a3 3 0 1 0 3 3V6a3 3 0 1 0-3 3h12Z"/></>,
}

export function Icon({ name, size = 18, className = '' }) {
  return (
    <svg
      aria-hidden="true"
      className={className}
      fill="none"
      height={size}
      stroke="currentColor"
      strokeLinecap="round"
      strokeLinejoin="round"
      strokeWidth="1.75"
      viewBox="0 0 24 24"
      width={size}
    >
      {PATHS[name]}
    </svg>
  )
}
