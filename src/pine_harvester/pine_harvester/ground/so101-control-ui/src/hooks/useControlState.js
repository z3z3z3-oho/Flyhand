import { useCallback, useEffect, useRef, useState } from 'react'
import { getControlState } from '../lib/api.js'

const INITIAL_STATE = {
  connected: false,
  connecting: false,
  mode: 'DISCONNECTED',
  sequence: 0,
  values: [0, 0, 0, 0, 0, 0],
  logs: [],
  error: null,
}

export function useControlState(intervalMs = 500) {
  const [state, setState] = useState(INITIAL_STATE)
  const [apiOnline, setApiOnline] = useState(true)
  const [lastUpdated, setLastUpdated] = useState(null)
  const mounted = useRef(true)

  const refresh = useCallback(async () => {
    const controller = new AbortController()
    const timeout = window.setTimeout(() => controller.abort(), 1800)
    try {
      const nextState = await getControlState(controller.signal)
      if (!mounted.current) return
      setState((current) => ({ ...current, ...nextState }))
      setApiOnline(true)
      setLastUpdated(new Date())
    } catch (error) {
      if (!mounted.current || error.name === 'AbortError') return
      setApiOnline(false)
    } finally {
      window.clearTimeout(timeout)
    }
  }, [])

  useEffect(() => {
    mounted.current = true
    refresh()
    const timer = window.setInterval(refresh, intervalMs)
    return () => {
      mounted.current = false
      window.clearInterval(timer)
    }
  }, [intervalMs, refresh])

  return { state, apiOnline, lastUpdated, refresh }
}
