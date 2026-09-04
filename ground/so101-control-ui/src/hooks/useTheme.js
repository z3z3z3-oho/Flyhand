import { useEffect, useState } from 'react'

function resolveInitialTheme() {
  const saved = localStorage.getItem('so101-theme')
  if (saved === 'light' || saved === 'dark') return saved
  return window.matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light'
}

export function useTheme() {
  const [theme, setTheme] = useState(resolveInitialTheme)

  useEffect(() => {
    document.documentElement.classList.toggle('dark', theme === 'dark')
    document.documentElement.style.colorScheme = theme
    localStorage.setItem('so101-theme', theme)
    const meta = document.querySelector('meta[name="theme-color"]')
    meta?.setAttribute('content', theme === 'dark' ? '#0b0f17' : '#f7f8fa')
  }, [theme])

  return {
    theme,
    toggleTheme: () => setTheme((value) => (value === 'dark' ? 'light' : 'dark')),
  }
}
