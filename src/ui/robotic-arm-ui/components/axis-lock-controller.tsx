"use client"

import { useCallback, useEffect, useMemo, useState } from "react"
import {
  ArrowLeftRight,
  ArrowUpDown,
  Hand,
  Lock,
  Move,
  RotateCw,
  Unlock,
} from "lucide-react"
import { ServoSlider } from "@/components/servo-slider"
import { ConnectionStatus } from "@/components/connection-status"
import { ControlModeNav } from "@/components/control-mode-nav"
import { Button } from "@/components/ui/button"
import { Switch } from "@/components/ui/switch"
import {
  type AxisName,
  type ConstraintControllerState,
  type ConstraintType,
  type FrameType,
  estimateTcpPoseFromServoValues,
  rpyDegToRotationMatrix,
  stepConstraintMotion,
} from "@/lib/frame-constraint-ik"
import {
  clearConstraintLogs,
  downloadConstraintLogsJson,
  getConstraintLogs,
  isConstraintLoggerEnabled,
  setConstraintLoggerEnabled,
} from "@/lib/constraintLogger"

interface ServoConfig {
  id: number
  label: string
  min: number
  max: number
  defaultValue: number
  unit: string
  icon: React.ReactNode
  model: string
}

const SERVO_CONFIGS: ServoConfig[] = [
  { id: 1, label: "Base Rotation", min: 0, max: 240, defaultValue: 120, unit: "\u00B0", icon: <RotateCw className="h-4 w-4" />, model: "HTD35H" },
  { id: 2, label: "Shoulder", min: 0, max: 240, defaultValue: 120, unit: "\u00B0", icon: <ArrowUpDown className="h-4 w-4" />, model: "HTD35H" },
  { id: 3, label: "Elbow", min: 0, max: 240, defaultValue: 120, unit: "\u00B0", icon: <ArrowLeftRight className="h-4 w-4" />, model: "HTD35H" },
  { id: 4, label: "Wrist Pitch", min: 0, max: 240, defaultValue: 120, unit: "\u00B0", icon: <Move className="h-4 w-4" />, model: "HTS20H" },
  { id: 5, label: "Wrist Roll", min: 0, max: 240, defaultValue: 120, unit: "\u00B0", icon: <RotateCw className="h-4 w-4" />, model: "HTS20H" },
  { id: 6, label: "Gripper", min: 0, max: 240, defaultValue: 120, unit: "\u00B0", icon: <Hand className="h-4 w-4" />, model: "HX12H" },
]

const AXES: AxisName[] = ["x", "y", "z"]

interface UserFramePoseInput {
  xMm: number
  yMm: number
  zMm: number
  rollDeg: number
  pitchDeg: number
  yawDeg: number
}

function buildDefaultPose() {
  const pose: Record<number, number> = {}
  for (const cfg of SERVO_CONFIGS) {
    pose[cfg.id] = cfg.defaultValue
  }
  return pose
}

function buildDefaultLockState() {
  const locks: Record<number, boolean> = {}
  for (const cfg of SERVO_CONFIGS) {
    locks[cfg.id] = false
  }
  return locks
}

function buildDefaultUserFramePose(): UserFramePoseInput {
  return { xMm: 0, yMm: 0, zMm: 0, rollDeg: 0, pitchDeg: 0, yawDeg: 0 }
}

export function AxisLockController() {
  const [servoValues, setServoValues] = useState<Record<number, number>>(buildDefaultPose)
  const [lockedAxes, setLockedAxes] = useState<Record<number, boolean>>(buildDefaultLockState)
  const [jogStepDeg, setJogStepDeg] = useState(5)
  const [frameType, setFrameType] = useState<FrameType>("base")
  const [constraintType, setConstraintType] = useState<ConstraintType>("line")
  const [constraintAxis, setConstraintAxis] = useState<AxisName>("z")
  const [cartesianStepMm, setCartesianStepMm] = useState(10)
  const [rotationStepDeg, setRotationStepDeg] = useState(5)
  const [pointAnchorBase, setPointAnchorBase] = useState<[number, number, number] | null>(null)
  const [pointAllow3AxisRotation, setPointAllow3AxisRotation] = useState(false)
  const [pointRotationAxis, setPointRotationAxis] = useState<AxisName>("z")
  const [userFramePose, setUserFramePose] = useState<UserFramePoseInput>(buildDefaultUserFramePose)
  const [motionStatus, setMotionStatus] = useState("ready")
  const [connectionStatus, setConnectionStatus] = useState<"connected" | "disconnected" | "connecting">("connecting")
  const [logEnabled, setLogEnabled] = useState(false)
  const [logCount, setLogCount] = useState(0)
  const [constraintControllerState, setConstraintControllerState] = useState<ConstraintControllerState | undefined>(undefined)

  useEffect(() => {
    setLogEnabled(isConstraintLoggerEnabled())
    setLogCount(getConstraintLogs().length)
  }, [])

  const refreshLogCount = useCallback(() => {
    setLogCount(getConstraintLogs().length)
    setLogEnabled(isConstraintLoggerEnabled())
  }, [])

  const handleToggleLogs = useCallback(() => {
    const next = !isConstraintLoggerEnabled()
    setConstraintLoggerEnabled(next)
    setLogEnabled(next)
  }, [])

  const handleExportLogs = useCallback(() => {
    downloadConstraintLogsJson()
    refreshLogCount()
  }, [refreshLogCount])

  const handleClearLogs = useCallback(() => {
    clearConstraintLogs()
    refreshLogCount()
  }, [refreshLogCount])

  const sendServoCommand = useCallback(async (id: number, value: number, durationMs = 250) => {
    try {
      const response = await fetch("/api/servo-command", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ servoId: id, value, durationMs }),
      })

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}`)
      }
      setConnectionStatus("connected")
    } catch {
      setConnectionStatus("disconnected")
    }
  }, [])

  const setLockPreset = useCallback((idsToLock: number[]) => {
    const next = buildDefaultLockState()
    for (const id of idsToLock) {
      next[id] = true
    }
    setLockedAxes(next)
  }, [])

  const unlockedCount = useMemo(
    () => SERVO_CONFIGS.filter((cfg) => !lockedAxes[cfg.id]).length,
    [lockedAxes]
  )

  const handleServoChange = useCallback((id: number, value: number) => {
    if (lockedAxes[id]) {
      return
    }
    setServoValues((prev) => ({ ...prev, [id]: value }))
    void sendServoCommand(id, value)
  }, [lockedAxes, sendServoCommand])

  const handleLockChange = useCallback((id: number, checked: boolean) => {
    setLockedAxes((prev) => ({ ...prev, [id]: checked }))
  }, [])

  const applyUnlockedAxes = useCallback(() => {
    for (const cfg of SERVO_CONFIGS) {
      if (!lockedAxes[cfg.id]) {
        void sendServoCommand(cfg.id, servoValues[cfg.id], 420)
      }
    }
  }, [lockedAxes, sendServoCommand, servoValues])

  const setUnlockedToDefault = useCallback(() => {
    setServoValues((prev) => {
      const next = { ...prev }
      for (const cfg of SERVO_CONFIGS) {
        if (!lockedAxes[cfg.id]) {
          next[cfg.id] = cfg.defaultValue
          void sendServoCommand(cfg.id, cfg.defaultValue, 320)
        }
      }
      return next
    })
  }, [lockedAxes, sendServoCommand])

  const nudgeAxis = useCallback((id: number, direction: -1 | 1) => {
    if (lockedAxes[id]) {
      return
    }
    const cfg = SERVO_CONFIGS.find((item) => item.id === id)
    if (!cfg) {
      return
    }

    setServoValues((prev) => {
      const current = prev[id]
      const nextValue = Math.min(cfg.max, Math.max(cfg.min, current + direction * jogStepDeg))
      void sendServoCommand(id, nextValue)
      return { ...prev, [id]: nextValue }
    })
  }, [jogStepDeg, lockedAxes, sendServoCommand])

  const userFrameTransform = useMemo(() => ({
    position: [userFramePose.xMm / 1000, userFramePose.yMm / 1000, userFramePose.zMm / 1000] as [number, number, number],
    rotation: rpyDegToRotationMatrix(userFramePose.rollDeg, userFramePose.pitchDeg, userFramePose.yawDeg),
  }), [userFramePose.pitchDeg, userFramePose.rollDeg, userFramePose.xMm, userFramePose.yMm, userFramePose.yawDeg, userFramePose.zMm])

  const setUserFrameField = useCallback((field: keyof UserFramePoseInput, raw: string) => {
    const parsed = Number(raw)
    setUserFramePose((prev) => ({
      ...prev,
      [field]: Number.isFinite(parsed) ? parsed : 0,
    }))
  }, [])

  const capturePointAnchor = useCallback(() => {
    const tcp = estimateTcpPoseFromServoValues(servoValues)
    setPointAnchorBase(tcp.position)
    setMotionStatus("point_anchor_captured")
  }, [servoValues])

  const runConstraintJog = useCallback((axis: AxisName, sign: -1 | 1) => {
    const result = stepConstraintMotion({
      servoValues,
      lockedAxes,
      frame: frameType,
      constraint: constraintType,
      constraintAxis,
      jogAxis: axis,
      sign,
      stepMm: cartesianStepMm,
      userFrame: frameType === "user" ? userFrameTransform : undefined,
      pointAnchorBase: constraintType === "point" ? pointAnchorBase ?? undefined : undefined,
      rotationStepDeg,
      pointAllow3AxisRotation,
      pointRotationAxis,
      nowMs: Date.now(),
      controllerState: constraintControllerState,
    })

    if (!result.ok || !result.nextServoValues) {
      setMotionStatus(result.error ?? "constraint_step_failed")
      return
    }

    const nextValues = result.nextServoValues
    setServoValues(nextValues)
    for (let id = 1; id <= 5; id += 1) {
      if (!lockedAxes[id] && nextValues[id] !== servoValues[id]) {
        void sendServoCommand(id, nextValues[id], 260)
      }
    }
    const baseStatus = `${frameType}_${constraintType}_${axis}_${sign > 0 ? "plus" : "minus"}`
    setMotionStatus(result.warning ? `${baseStatus} (${result.warning})` : baseStatus)
    setConstraintControllerState(result.nextControllerState)
    setLogCount(getConstraintLogs().length)
  }, [cartesianStepMm, constraintAxis, constraintControllerState, constraintType, frameType, lockedAxes, pointAllow3AxisRotation, pointAnchorBase, pointRotationAxis, rotationStepDeg, sendServoCommand, servoValues, userFrameTransform])

  return (
    <div className="min-h-screen bg-background">
      <header className="sticky top-0 z-10 border-b border-border bg-background/80 backdrop-blur-md">
        <div className="mx-auto flex max-w-7xl items-center justify-between px-4 py-3 lg:px-8">
          <div className="flex items-center gap-3">
            <div className="flex h-8 w-8 items-center justify-center rounded-lg bg-primary text-primary-foreground">
              <Lock className="h-4 w-4" />
            </div>
            <div>
              <h1 className="text-base font-semibold text-foreground tracking-tight">
                Robot Arm Controller
              </h1>
              <p className="text-xs text-muted-foreground font-mono">
                Frame Constraint Motion Mode
              </p>
            </div>
          </div>
          <div className="flex items-center gap-3">
            <ControlModeNav currentMode="axis-lock" />
            <ConnectionStatus status={connectionStatus} />
          </div>
        </div>
      </header>

      <main className="mx-auto max-w-7xl px-4 py-6 lg:px-8">
        <div className="grid gap-6 lg:grid-cols-[1fr_380px]">
          <section>
            <div className="mb-4 flex items-center justify-between">
              <h2 className="text-sm font-semibold text-foreground uppercase tracking-wider">
                Joint Panel (Lock Aware)
              </h2>
              <span className="text-xs font-mono text-muted-foreground">
                unlocked {unlockedCount}/{SERVO_CONFIGS.length}
              </span>
            </div>
            <div className="grid gap-3 sm:grid-cols-2">
              {SERVO_CONFIGS.map((cfg) => {
                const isLocked = lockedAxes[cfg.id]
                return (
                  <div
                    key={cfg.id}
                    className="rounded-xl border border-border bg-card p-3"
                  >
                    <div className="mb-2 flex items-center justify-between">
                      <p className="text-xs font-mono text-muted-foreground">
                        {cfg.model}
                      </p>
                      <div className="flex items-center gap-2">
                        <span className="text-[11px] font-mono text-muted-foreground">
                          {isLocked ? "LOCKED" : "FREE"}
                        </span>
                        <Switch
                          checked={isLocked}
                          onCheckedChange={(checked) => handleLockChange(cfg.id, checked)}
                          aria-label={`${cfg.label} axis lock`}
                        />
                      </div>
                    </div>

                    <div className={isLocked ? "opacity-45 pointer-events-none" : ""}>
                      <ServoSlider
                        id={cfg.id}
                        label={cfg.label}
                        value={servoValues[cfg.id]}
                        min={cfg.min}
                        max={cfg.max}
                        unit={cfg.unit}
                        icon={cfg.icon}
                        onChange={(value) => handleServoChange(cfg.id, value)}
                      />
                    </div>

                    <div className="mt-2 grid grid-cols-2 gap-2">
                      <Button
                        variant="outline"
                        size="sm"
                        className="font-mono text-xs"
                        onClick={() => nudgeAxis(cfg.id, -1)}
                        disabled={isLocked}
                      >
                        -{jogStepDeg}
                      </Button>
                      <Button
                        variant="outline"
                        size="sm"
                        className="font-mono text-xs"
                        onClick={() => nudgeAxis(cfg.id, 1)}
                        disabled={isLocked}
                      >
                        +{jogStepDeg}
                      </Button>
                    </div>
                  </div>
                )
              })}
            </div>
          </section>

          <aside className="flex flex-col gap-4">
            <div className="rounded-xl border border-border bg-card p-4">
              <h3 className="mb-3 text-xs font-semibold text-foreground uppercase tracking-wider">
                Frame Constraint Jog
              </h3>

              <div className="mb-3 grid grid-cols-2 gap-2">
                <Button
                  size="sm"
                  variant={frameType === "base" ? "default" : "outline"}
                  className="font-mono text-xs"
                  onClick={() => setFrameType("base")}
                >
                  Base Frame
                </Button>
                <Button
                  size="sm"
                  variant={frameType === "tool" ? "default" : "outline"}
                  className="font-mono text-xs"
                  onClick={() => setFrameType("tool")}
                >
                  Tool Frame
                </Button>
                <Button
                  size="sm"
                  variant={frameType === "user" ? "default" : "outline"}
                  className="font-mono text-xs col-span-2"
                  onClick={() => setFrameType("user")}
                >
                  User Frame
                </Button>
              </div>

              <div className="mb-3 rounded-md border border-border/70 bg-muted/20 p-2">
                <p className="mb-2 text-[11px] font-mono text-muted-foreground">User Frame (Base 기준)</p>
                <div className="grid grid-cols-3 gap-2">
                  <input
                    type="number"
                    value={userFramePose.xMm}
                    onChange={(e) => setUserFrameField("xMm", e.target.value)}
                    className="h-8 rounded-md border border-input bg-background px-2 text-right font-mono text-xs"
                    aria-label="user frame x mm"
                  />
                  <input
                    type="number"
                    value={userFramePose.yMm}
                    onChange={(e) => setUserFrameField("yMm", e.target.value)}
                    className="h-8 rounded-md border border-input bg-background px-2 text-right font-mono text-xs"
                    aria-label="user frame y mm"
                  />
                  <input
                    type="number"
                    value={userFramePose.zMm}
                    onChange={(e) => setUserFrameField("zMm", e.target.value)}
                    className="h-8 rounded-md border border-input bg-background px-2 text-right font-mono text-xs"
                    aria-label="user frame z mm"
                  />
                  <input
                    type="number"
                    value={userFramePose.rollDeg}
                    onChange={(e) => setUserFrameField("rollDeg", e.target.value)}
                    className="h-8 rounded-md border border-input bg-background px-2 text-right font-mono text-xs"
                    aria-label="user frame roll deg"
                  />
                  <input
                    type="number"
                    value={userFramePose.pitchDeg}
                    onChange={(e) => setUserFrameField("pitchDeg", e.target.value)}
                    className="h-8 rounded-md border border-input bg-background px-2 text-right font-mono text-xs"
                    aria-label="user frame pitch deg"
                  />
                  <input
                    type="number"
                    value={userFramePose.yawDeg}
                    onChange={(e) => setUserFrameField("yawDeg", e.target.value)}
                    className="h-8 rounded-md border border-input bg-background px-2 text-right font-mono text-xs"
                    aria-label="user frame yaw deg"
                  />
                </div>
                <p className="mt-2 text-[10px] font-mono text-muted-foreground">
                  pos(mm): x y z / rot(deg): roll pitch yaw
                </p>
              </div>

              <div className="mb-3 grid grid-cols-3 gap-2">
                <Button
                  size="sm"
                  variant={constraintType === "line" ? "default" : "outline"}
                  className="font-mono text-xs"
                  onClick={() => setConstraintType("line")}
                >
                  Line
                </Button>
                <Button
                  size="sm"
                  variant={constraintType === "plane" ? "default" : "outline"}
                  className="font-mono text-xs"
                  onClick={() => setConstraintType("plane")}
                >
                  Plane
                </Button>
                <Button
                  size="sm"
                  variant={constraintType === "point" ? "default" : "outline"}
                  className="font-mono text-xs"
                  onClick={() => setConstraintType("point")}
                >
                  Point
                </Button>
              </div>

              <div className="mb-3 grid grid-cols-3 gap-2">
                {AXES.map((axis) => (
                  <Button
                    key={axis}
                    size="sm"
                    variant={constraintAxis === axis ? "secondary" : "outline"}
                    className="font-mono text-xs uppercase"
                    onClick={() => setConstraintAxis(axis)}
                  >
                    {axis}
                  </Button>
                ))}
              </div>

              {constraintType !== "point" ? (
                <div className="mb-3 flex items-center gap-2">
                  <span className="text-xs font-mono text-muted-foreground">Step</span>
                  <input
                    type="number"
                    value={cartesianStepMm}
                    min={1}
                    max={40}
                    onChange={(e) => setCartesianStepMm(Math.max(1, Math.min(40, Math.trunc(Number(e.target.value) || 1))))}
                    className="h-8 w-20 rounded-md border border-input bg-background px-2 text-right font-mono text-xs"
                    aria-label="constraint jog step mm"
                  />
                  <span className="text-xs font-mono text-muted-foreground">mm</span>
                </div>
              ) : (
                <div className="mb-3 space-y-2">
                  <div className="flex items-center gap-2">
                    <span className="text-xs font-mono text-muted-foreground">Rotate</span>
                    <input
                      type="number"
                      value={rotationStepDeg}
                      min={1}
                      max={30}
                      onChange={(e) => setRotationStepDeg(Math.max(1, Math.min(30, Math.trunc(Number(e.target.value) || 1))))}
                      className="h-8 w-20 rounded-md border border-input bg-background px-2 text-right font-mono text-xs"
                      aria-label="point rotate step degree"
                    />
                    <span className="text-xs font-mono text-muted-foreground">deg</span>
                  </div>
                  <Button
                    size="sm"
                    variant="secondary"
                    className="w-full font-mono text-xs"
                    onClick={capturePointAnchor}
                  >
                    Capture Point Anchor (p0)
                  </Button>
                  <p className="text-[10px] font-mono text-muted-foreground">
                    {pointAnchorBase
                      ? `p0(base,m)=(${pointAnchorBase[0].toFixed(3)}, ${pointAnchorBase[1].toFixed(3)}, ${pointAnchorBase[2].toFixed(3)})`
                      : "p0 not set: first point jog will use current TCP as fallback"}
                  </p>
                  <div className="grid grid-cols-2 gap-2">
                    <Button
                      size="sm"
                      variant={!pointAllow3AxisRotation ? "default" : "outline"}
                      className="font-mono text-xs"
                      onClick={() => setPointAllow3AxisRotation(false)}
                    >
                      Rotation 1-axis
                    </Button>
                    <Button
                      size="sm"
                      variant={pointAllow3AxisRotation ? "default" : "outline"}
                      className="font-mono text-xs"
                      onClick={() => setPointAllow3AxisRotation(true)}
                    >
                      Rotation 3-axis
                    </Button>
                  </div>
                  <div className="grid grid-cols-3 gap-2">
                    {AXES.map((axis) => (
                      <Button
                        key={`point-axis-${axis}`}
                        size="sm"
                        variant={pointRotationAxis === axis ? "secondary" : "outline"}
                        className="font-mono text-xs uppercase"
                        onClick={() => setPointRotationAxis(axis)}
                      >
                        Tool {axis.toUpperCase()}
                      </Button>
                    ))}
                  </div>
                  {pointAllow3AxisRotation && (
                    <p className="text-[10px] font-mono text-amber-600">
                      5DOF arm: 3-axis rotation mode may increase point drift near singularity.
                    </p>
                  )}
                </div>
              )}

              <div className="grid grid-cols-3 gap-2">
                {AXES.map((axis) => {
                  return (
                    <div key={axis} className="space-y-2">
                      <Button
                        size="sm"
                        variant="outline"
                        className="w-full font-mono text-xs"
                        onClick={() => runConstraintJog(axis, 1)}
                      >
                        +{constraintType === "point" ? (axis === "x" ? "Roll" : axis === "y" ? "Pitch" : "Yaw") : axis.toUpperCase()}
                      </Button>
                      <Button
                        size="sm"
                        variant="outline"
                        className="w-full font-mono text-xs"
                        onClick={() => runConstraintJog(axis, -1)}
                      >
                        -{constraintType === "point" ? (axis === "x" ? "Roll" : axis === "y" ? "Pitch" : "Yaw") : axis.toUpperCase()}
                      </Button>
                    </div>
                  )
                })}
              </div>

              <p className="mt-3 text-[11px] font-mono text-muted-foreground">
                status: {motionStatus}
              </p>
              <p className="mt-2 text-[11px] font-mono text-muted-foreground">
                line=하드 선투영, plane=하드 평면투영, point=소프트 위치고정 + 회전(기본 1축)
              </p>
            </div>

            <div className="rounded-xl border border-border bg-card p-4">
              <h3 className="mb-3 text-xs font-semibold text-foreground uppercase tracking-wider">
                Lock Preset
              </h3>
              <div className="grid grid-cols-2 gap-2">
                <Button variant="secondary" size="sm" className="font-mono text-xs" onClick={() => setLockPreset([])}>
                  <Unlock className="h-3.5 w-3.5" />
                  Unlock All
                </Button>
                <Button variant="secondary" size="sm" className="font-mono text-xs" onClick={() => setLockPreset([1])}>
                  Base Lock
                </Button>
                <Button variant="secondary" size="sm" className="font-mono text-xs" onClick={() => setLockPreset([4, 5, 6])}>
                  Wrist+Grip Lock
                </Button>
                <Button variant="secondary" size="sm" className="font-mono text-xs" onClick={() => setLockPreset([1, 4, 5])}>
                  XY Guide Lock
                </Button>
              </div>
            </div>

            <div className="rounded-xl border border-border bg-card p-4">
              <h3 className="mb-3 text-xs font-semibold text-foreground uppercase tracking-wider">
                Motion Actions
              </h3>
              <div className="space-y-2">
                <div className="flex items-center gap-2">
                  <span className="text-xs font-mono text-muted-foreground">Joint Jog</span>
                  <input
                    type="number"
                    value={jogStepDeg}
                    min={1}
                    max={30}
                    onChange={(e) => setJogStepDeg(Math.max(1, Math.min(30, Math.trunc(Number(e.target.value) || 1))))}
                    className="h-8 w-20 rounded-md border border-input bg-background px-2 text-right font-mono text-xs"
                    aria-label="joint jog step degree"
                  />
                  <span className="text-xs font-mono text-muted-foreground">\u00B0</span>
                </div>
                <Button size="sm" className="w-full font-mono text-xs" onClick={applyUnlockedAxes} disabled={unlockedCount === 0}>
                  Apply Unlocked Axes
                </Button>
                <Button variant="outline" size="sm" className="w-full font-mono text-xs" onClick={setUnlockedToDefault} disabled={unlockedCount === 0}>
                  Reset Unlocked To Default
                </Button>
              </div>
            </div>

            <div className="rounded-xl border border-border bg-card p-4">
              <h3 className="mb-3 text-xs font-semibold text-foreground uppercase tracking-wider">
                Developer Observability
              </h3>
              <div className="space-y-2">
                <p className="text-[11px] font-mono text-muted-foreground">
                  logger: {logEnabled ? "ON" : "OFF"} / entries: {logCount}
                </p>
                <div className="grid grid-cols-2 gap-2">
                  <Button size="sm" variant={logEnabled ? "secondary" : "outline"} className="font-mono text-xs" onClick={handleToggleLogs}>
                    {logEnabled ? "Disable Logs" : "Enable Logs"}
                  </Button>
                  <Button size="sm" variant="outline" className="font-mono text-xs" onClick={handleExportLogs}>
                    Export JSON
                  </Button>
                  <Button size="sm" variant="outline" className="font-mono text-xs" onClick={refreshLogCount}>
                    Refresh Count
                  </Button>
                  <Button size="sm" variant="outline" className="font-mono text-xs" onClick={handleClearLogs}>
                    Clear Logs
                  </Button>
                </div>
                <p className="text-[10px] font-mono text-muted-foreground">
                  x_meas는 현재 센서 실측이 아닌 FK 추정치(estimated_fk)로 기록됩니다.
                </p>
              </div>
            </div>

            <div className="rounded-xl border border-border bg-card p-4">
              <h3 className="mb-3 text-xs font-semibold text-foreground uppercase tracking-wider">
                Servo Summary (11.1V)
              </h3>
              <div className="space-y-2 text-xs font-mono text-muted-foreground">
                <p>HTD35H x3: Body axes (S1,S2,S3), 35kg-cm, 0-240deg</p>
                <p>HTS20H x2: Mid axes (S4,S5), 20kg-cm, 0-240deg</p>
                <p>HX12H x1: End axis (S6), 12kg-cm, 0-240deg</p>
              </div>
            </div>
          </aside>
        </div>
      </main>
    </div>
  )
}
