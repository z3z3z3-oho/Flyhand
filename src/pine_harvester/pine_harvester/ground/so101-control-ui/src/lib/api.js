const JSON_HEADERS = { 'Content-Type': 'application/json' }

async function parseResponse(response) {
  if (response.ok) {
    const type = response.headers.get('content-type') || ''
    return type.includes('application/json') ? response.json() : null
  }

  let message = `请求失败（${response.status}）`
  try {
    const payload = await response.json()
    message = payload.detail || payload.message || message
  } catch {
    // Keep the status-based fallback.
  }
  throw new Error(message)
}

export async function getControlState(signal) {
  const response = await fetch('/api/state', { signal, cache: 'no-store' })
  return parseResponse(response)
}

export async function connectController(config) {
  const response = await fetch('/api/connect', {
    method: 'POST',
    headers: JSON_HEADERS,
    body: JSON.stringify(config),
  })
  return parseResponse(response)
}

export async function disconnectController() {
  const response = await fetch('/api/disconnect', { method: 'POST' })
  return parseResponse(response)
}

export async function sendMode(command) {
  const response = await fetch(`/api/mode/${command}`, { method: 'POST' })
  return parseResponse(response)
}
