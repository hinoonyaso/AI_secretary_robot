"use client"

import { useState, useCallback } from "react"
import {
  RotateCw,
  ArrowUpDown,
  ArrowLeftRight,
  Move,
  Hand,
  Undo2,
} from "lucide-react"
import { ServoSlider } from "@/components/servo-slider"
import { ConnectionStatus } from "@/components/connection-status"
import { ControlModeNav } from "@/components/control-mode-nav"
import { Button } from "@/components/ui/button"

interface ServoConfig {
  id: number
  label: string
  min: number
  max: number
  defaultValue: number
  unit: string
  icon: React.ReactNode
}

const SERVO_CONFIGS: ServoConfig[] = [
  {
    id: 1,
    label: "Base Rotation",
    min: 0,
    max: 240,
    defaultValue: 120,
    unit: "\u00B0",
    icon: <RotateCw className="h-4 w-4" />,
  },
  {
    id: 2,
    label: "Shoulder",
    min: 0,
    max: 240,
    defaultValue: 120,
    unit: "\u00B0",
    icon: <ArrowUpDown className="h-4 w-4" />,
  },
  {
    id: 3,
    label: "Elbow",
    min: 0,
    max: 240,
    defaultValue: 120,
    unit: "\u00B0",
    icon: <ArrowLeftRight className="h-4 w-4" />,
  },
  {
    id: 4,
    label: "Wrist Pitch",
    min: 0,
    max: 240,
    defaultValue: 120,
    unit: "\u00B0",
    icon: <Move className="h-4 w-4" />,
  },
  {
    id: 5,
    label: "Wrist Roll",
    min: 0,
    max: 240,
    defaultValue: 120,
    unit: "\u00B0",
    icon: <RotateCw className="h-4 w-4" />,
  },
  {
    id: 6,
    label: "Gripper",
    min: 0,
    max: 240,
    defaultValue: 120,
    unit: "\u00B0",
    icon: <Hand className="h-4 w-4" />,
  },
]

export function RobotController() {
  const [servoValues, setServoValues] = useState<Record<number, number>>(
    () => {
      const initial: Record<number, number> = {}
      for (const config of SERVO_CONFIGS) {
        initial[config.id] = config.defaultValue
      }
      return initial
    }
  )

  const [connectionStatus, setConnectionStatus] = useState<"connected" | "disconnected" | "connecting">("connecting")

  const sendServoCommand = useCallback(async (id: number, value: number, durationMs = 250) => {
    try {
      const response = await fetch("/api/servo-command", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          servoId: id,
          value,
          durationMs,
        }),
      })

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}`)
      }

      setConnectionStatus("connected")
    } catch {
      setConnectionStatus("disconnected")
    }
  }, [])

  const handleServoChange = useCallback((id: number, value: number) => {
    setServoValues((prev) => ({ ...prev, [id]: value }))
    void sendServoCommand(id, value)
  }, [sendServoCommand])

  const applyPose = useCallback((pose: Record<number, number>) => {
    setServoValues(pose)
    for (const [id, value] of Object.entries(pose)) {
      void sendServoCommand(Number(id), value, 400)
    }
  }, [sendServoCommand])

  const handleResetAll = useCallback(() => {
    const defaults: Record<number, number> = {}
    for (const config of SERVO_CONFIGS) {
      defaults[config.id] = config.defaultValue
    }
    applyPose(defaults)
  }, [applyPose])

  return (
    <div className="min-h-screen bg-background">
      {/* Header */}
      <header className="sticky top-0 z-10 border-b border-border bg-background/80 backdrop-blur-md">
        <div className="mx-auto flex max-w-7xl items-center justify-between px-4 py-3 lg:px-8">
          <div className="flex items-center gap-3">
            <div className="flex h-8 w-8 items-center justify-center rounded-lg bg-primary text-primary-foreground">
              <RotateCw className="h-4 w-4" />
            </div>
            <div>
              <h1 className="text-base font-semibold text-foreground tracking-tight">
                Robot Arm Controller
              </h1>
              <p className="text-xs text-muted-foreground font-mono">
                6-Axis Servo Control Panel
              </p>
            </div>
          </div>
          <div className="flex items-center gap-4">
            <ControlModeNav currentMode="servo" />
            <ConnectionStatus status={connectionStatus} />
            <Button
              variant="outline"
              size="sm"
              onClick={handleResetAll}
              className="gap-1.5 text-xs font-mono"
            >
              <Undo2 className="h-3.5 w-3.5" />
              Reset All
            </Button>
          </div>
        </div>
      </header>

      {/* Main Content */}
      <main className="mx-auto max-w-7xl px-4 py-6 lg:px-8">
        <div className="grid gap-6 lg:grid-cols-[1fr_400px]">
          {/* Servo Controls */}
          <div className="order-2 lg:order-1">
            <div className="mb-4 flex items-center justify-between">
              <h2 className="text-sm font-semibold text-foreground uppercase tracking-wider">
                Servo Controls
              </h2>
              <span className="text-xs font-mono text-muted-foreground">
                {SERVO_CONFIGS.length} servos
              </span>
            </div>
            <div className="grid gap-3 sm:grid-cols-2">
              {SERVO_CONFIGS.map((config) => (
                <ServoSlider
                  key={config.id}
                  id={config.id}
                  label={config.label}
                  value={servoValues[config.id]}
                  min={config.min}
                  max={config.max}
                  unit={config.unit}
                  icon={config.icon}
                  onChange={(value) => handleServoChange(config.id, value)}
                />
              ))}
            </div>
          </div>

          {/* Robot Preview & Status */}
          <div className="order-1 lg:order-2 flex flex-col gap-4">
            {/* Robot Image */}
            <div className="rounded-xl border border-border bg-card p-4">
              <div className="relative flex items-center justify-center rounded-lg bg-secondary/30 p-6">
                {/* eslint-disable-next-line @next/next/no-img-element */}
                <img
                  src="/api/robot-image"
                  alt="6-axis robotic arm illustration"
                  width={300}
                  height={350}
                  className="object-contain drop-shadow-[0_0_30px_rgba(74,168,100,0.15)]"
                />
              </div>
            </div>

            {/* Live Values */}
            <div className="rounded-xl border border-border bg-card p-4">
              <h3 className="mb-3 text-xs font-semibold text-foreground uppercase tracking-wider">
                Live Values
              </h3>
              <div className="grid grid-cols-3 gap-2">
                {SERVO_CONFIGS.map((config) => (
                  <div
                    key={config.id}
                    className="rounded-lg bg-secondary p-2.5 text-center"
                  >
                    <p className="text-[10px] font-mono text-muted-foreground uppercase tracking-wider mb-1">
                      {"S"}{config.id}
                    </p>
                    <p className="text-sm font-mono font-semibold text-foreground">
                      {servoValues[config.id]}
                      <span className="text-[10px] text-muted-foreground ml-0.5">
                        {config.unit}
                      </span>
                    </p>
                  </div>
                ))}
              </div>
            </div>

            {/* Quick Actions */}
            <div className="rounded-xl border border-border bg-card p-4">
              <h3 className="mb-3 text-xs font-semibold text-foreground uppercase tracking-wider">
                Quick Actions
              </h3>
              <div className="grid grid-cols-2 gap-2">
                <Button
                  variant="secondary"
                  size="sm"
                  className="font-mono text-xs"
                  onClick={() => {
                    const home: Record<number, number> = {}
                    for (const config of SERVO_CONFIGS) {
                      home[config.id] = config.id === 6 ? 0 : Math.round((config.max - config.min) / 2) + config.min
                    }
                    applyPose(home)
                  }}
                >
                  Home Position
                </Button>
                <Button
                  variant="secondary"
                  size="sm"
                  className="font-mono text-xs"
                  onClick={() => {
                    const zero: Record<number, number> = {}
                    for (const config of SERVO_CONFIGS) {
                      zero[config.id] = config.min
                    }
                    applyPose(zero)
                  }}
                >
                  All Min
                </Button>
                <Button
                  variant="secondary"
                  size="sm"
                  className="font-mono text-xs"
                  onClick={() => {
                    const maxVals: Record<number, number> = {}
                    for (const config of SERVO_CONFIGS) {
                      maxVals[config.id] = config.max
                    }
                    applyPose(maxVals)
                  }}
                >
                  All Max
                </Button>
                <Button
                  variant="secondary"
                  size="sm"
                  className="font-mono text-xs gap-1.5"
                  onClick={handleResetAll}
                >
                  <Undo2 className="h-3 w-3" />
                  Default
                </Button>
              </div>
            </div>
          </div>
        </div>
      </main>
    </div>
  )
}
