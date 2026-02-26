import type {
  ResidualAxisName,
  ResidualConstraintType,
  ResidualFrameType,
  Transform3Like,
  Vec3,
} from "./constraintResiduals"

const STORAGE_KEY = "constraint_logger_enabled"
const MAX_LOGS = 2000

let enabledOverride: boolean | null = null
let logBuffer: ConstraintLogEntry[] = []
let lastTickTsMs: number | null = null

export interface ConstraintLogEntry {
  ts: string
  ts_ms: number
  frameType: ResidualFrameType
  constraintType: ResidualConstraintType
  constraintAxis: ResidualAxisName
  axisVecOrNormalBase: Vec3
  x_cmd:
    | { mode: "pose"; pos: Vec3; quat: [number, number, number, number] }
    | { mode: "twist"; twist_v: Vec3; twist_w: Vec3 }
  x_meas: {
    pos: Vec3
    quat: [number, number, number, number]
    source: "estimated_fk" | "last_known"
    note?: string
  }
  residual: {
    active: number
    plane_n_dot_dp: number
    line_orthogonal_error: number
    point_position_error: number
  }
  T_base_tool?: Transform3Like
  T_base_user?: Transform3Like
  latency_ms: number
  loop_rate_hz: number | null
  warnings?: string[]
  hard_projection?: {
    v_raw: Vec3
    v_hard: Vec3
    residual_norm: number
  }
  point_debug?: {
    p0: Vec3
    p_meas: Vec3
    e_p_norm: number
    omega_des: Vec3
    wr_scale: number
    lambda: number
    cond_j: number
  }
}

interface ConstraintLogInput {
  frameType: ResidualFrameType
  constraintType: ResidualConstraintType
  constraintAxis: ResidualAxisName
  axisVecOrNormalBase: Vec3
  x_cmd: ConstraintLogEntry["x_cmd"]
  x_meas: ConstraintLogEntry["x_meas"]
  residual: ConstraintLogEntry["residual"]
  T_base_tool?: Transform3Like
  T_base_user?: Transform3Like
  latency_ms: number
  warnings?: string[]
  hard_projection?: ConstraintLogEntry["hard_projection"]
  point_debug?: ConstraintLogEntry["point_debug"]
}

function nowMs() {
  return Date.now()
}

function readEnabledFromRuntime(): boolean {
  const envEnabled = process.env.NEXT_PUBLIC_CONSTRAINT_LOGGER === "1"
  if (typeof window === "undefined") {
    return envEnabled
  }

  const params = new URLSearchParams(window.location.search)
  const qp = params.get("constraintLog")
  if (qp === "1" || qp === "true") return true
  if (qp === "0" || qp === "false") return false

  const stored = window.localStorage.getItem(STORAGE_KEY)
  if (stored === "1") return true
  if (stored === "0") return false
  return envEnabled
}

export function isConstraintLoggerEnabled() {
  if (enabledOverride !== null) return enabledOverride
  return readEnabledFromRuntime()
}

export function setConstraintLoggerEnabled(enabled: boolean) {
  enabledOverride = enabled
  if (typeof window !== "undefined") {
    window.localStorage.setItem(STORAGE_KEY, enabled ? "1" : "0")
  }
}

export function clearConstraintLoggerOverride() {
  enabledOverride = null
}

export function addConstraintLog(input: ConstraintLogInput): ConstraintLogEntry | null {
  if (!isConstraintLoggerEnabled()) return null
  const tsMs = nowMs()
  const loopRateHz = lastTickTsMs ? (tsMs > lastTickTsMs ? 1000 / (tsMs - lastTickTsMs) : null) : null
  lastTickTsMs = tsMs

  const entry: ConstraintLogEntry = {
    ts: new Date(tsMs).toISOString(),
    ts_ms: tsMs,
    frameType: input.frameType,
    constraintType: input.constraintType,
    constraintAxis: input.constraintAxis,
    axisVecOrNormalBase: input.axisVecOrNormalBase,
    x_cmd: input.x_cmd,
    x_meas: input.x_meas,
    residual: input.residual,
    T_base_tool: input.T_base_tool,
    T_base_user: input.T_base_user,
    latency_ms: input.latency_ms,
    loop_rate_hz: loopRateHz,
    warnings: input.warnings,
    hard_projection: input.hard_projection,
    point_debug: input.point_debug,
  }

  logBuffer.push(entry)
  if (logBuffer.length > MAX_LOGS) {
    logBuffer = logBuffer.slice(logBuffer.length - MAX_LOGS)
  }

  // JSONL style single-line log for tailing/debug.
  console.log(JSON.stringify(entry))
  return entry
}

export function getConstraintLogs(): ConstraintLogEntry[] {
  return [...logBuffer]
}

export function clearConstraintLogs() {
  logBuffer = []
  lastTickTsMs = null
}

export function exportConstraintLogsAsJson() {
  return JSON.stringify(logBuffer, null, 2)
}

export function downloadConstraintLogsJson(filename = "constraint-logs.json") {
  if (typeof window === "undefined") return
  const payload = exportConstraintLogsAsJson()
  const blob = new Blob([payload], { type: "application/json" })
  const url = window.URL.createObjectURL(blob)
  const a = document.createElement("a")
  a.href = url
  a.download = filename
  document.body.appendChild(a)
  a.click()
  a.remove()
  window.URL.revokeObjectURL(url)
}

// test hook
export function __setConstraintLoggerEnabledForTests(enabled: boolean | null) {
  enabledOverride = enabled
}
