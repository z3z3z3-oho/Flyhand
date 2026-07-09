import { useCallback, useEffect, useState } from 'react'
import { Header } from './components/Header.jsx'
import { ConnectionPanel } from './components/ConnectionPanel.jsx'
import { ModeControl } from './components/ModeControl.jsx'
import { TelemetryGrid } from './components/TelemetryGrid.jsx'
import { ActivityLog } from './components/ActivityLog.jsx'
import { GesturePreview } from './components/GesturePreview.jsx'
import { ShortcutBar } from './components/ShortcutBar.jsx'
import { useTheme } from './hooks/useTheme.js'
import { useControlState } from './hooks/useControlState.js'
import { sendMode } from './lib/api.js'

export default function App() {
  const { theme, toggleTheme } = useTheme()
  const { state, apiOnline, lastUpdated, refresh } = useControlState()
  const [shortcutError, setShortcutError] = useState('')

  const runShortcut = useCallback(
    async (command) => {
      if (!state.connected) return
      if (command === 'AUTO' && !window.confirm('确认机械臂周围安全，并进入 AUTO 模式？')) return
      try {
        setShortcutError('')
        await sendMode(command)
        await refresh()
      } catch (error) {
        setShortcutError(error.message)
      }
    },
    [refresh, state.connected],
  )

  useEffect(() => {
    const handler = (event) => {
      const tag = event.target?.tagName
      if (tag === 'INPUT' || tag === 'TEXTAREA' || event.ctrlKey || event.metaKey || event.altKey) return
      const key = event.key.toLowerCase()
      const mapping = {
        t: 'TELEOP',
        ' ': 'TELEOP',
        g: 'GESTURE',
        a: 'AUTO',
        h: 'HOLD',
        c: 'RECENTER',
      }
      if (!mapping[key]) return
      event.preventDefault()
      runShortcut(mapping[key])
    }
    window.addEventListener('keydown', handler)
    return () => window.removeEventListener('keydown', handler)
  }, [runShortcut])

  return (
    <div className="min-h-screen bg-[var(--surface-canvas)] text-[var(--text-primary)]">
      <Header
        theme={theme}
        toggleTheme={toggleTheme}
        connected={state.connected}
        connecting={state.connecting}
        apiOnline={apiOnline}
      />

      <main className="mx-auto max-w-[1440px] px-6 py-6 lg:px-8 lg:py-8">
        {!apiOnline ? (
          <div className="mb-6 rounded-[10px] border border-[var(--border-strong)] bg-[var(--surface-card)] px-4 py-3 shadow-[var(--shadow-default)]">
            <p className="text-[14px] font-semibold leading-[1.6] text-[var(--text-primary)]">无法访问本地控制 API</p>
            <p className="mt-1 text-[12px] leading-[1.6] text-[var(--text-secondary)]">请确认 FastAPI 服务运行在 127.0.0.1:8765。界面会自动重试。</p>
          </div>
        ) : null}

        <div className="grid grid-cols-1 gap-6 xl:grid-cols-12">
          <div className="xl:col-span-7">
            <ModeControl state={state} onChanged={refresh} />
          </div>
          <div className="xl:col-span-5">
            <ConnectionPanel connected={state.connected} connecting={state.connecting} onChanged={refresh} />
          </div>
          <div className="xl:col-span-7">
            <GesturePreview
              mode={state.mode}
              connected={state.connected}
              preview={state.gesture_preview}
              buildId={state.gesture_build_id}
            />
          </div>
          <div className="xl:col-span-5">
            <ActivityLog logs={state.logs} error={shortcutError || state.error} />
          </div>
          <div className="xl:col-span-12">
            <TelemetryGrid values={state.values} lastUpdated={lastUpdated} />
          </div>
        </div>
      </main>

      <ShortcutBar />
    </div>
  )
}
