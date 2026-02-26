import { describe, expect, it } from "vitest"
import {
  __setConstraintLoggerEnabledForTests,
  clearConstraintLogs,
  getConstraintLogs,
} from "./constraintLogger"
import { stepConstraintMotion } from "./frame-constraint-ik"

function baseServo() {
  return { 1: 120, 2: 120, 3: 120, 4: 120, 5: 120, 6: 120 }
}

function unlocked() {
  return { 1: false, 2: false, 3: false, 4: false, 5: false, 6: false }
}

describe("constraint observability integration", () => {
  it("fills log fields with correct frame/constraint metadata", () => {
    __setConstraintLoggerEnabledForTests(true)
    clearConstraintLogs()

    const first = stepConstraintMotion({
      servoValues: baseServo(),
      lockedAxes: unlocked(),
      frame: "base",
      constraint: "line",
      constraintAxis: "z",
      jogAxis: "z",
      sign: 1,
      stepMm: 20,
    })
    expect(first.ok).toBe(true)

    const second = stepConstraintMotion({
      servoValues: first.nextServoValues ?? baseServo(),
      lockedAxes: unlocked(),
      frame: "user",
      constraint: "plane",
      constraintAxis: "z",
      jogAxis: "x",
      sign: 1,
      stepMm: 20,
      userFrame: {
        position: [0.05, 0.01, 0],
        rotation: [
          [0, -1, 0],
          [1, 0, 0],
          [0, 0, 1],
        ],
      },
    })
    expect(second.ok).toBe(true)

    const logs = getConstraintLogs()
    expect(logs.length).toBeGreaterThanOrEqual(2)
    const l0 = logs[0]
    const l1 = logs[1]

    expect(l0.frameType).toBe("base")
    expect(l0.constraintType).toBe("line")
    expect(l0.latency_ms).toBeGreaterThanOrEqual(0)
    expect(l0.x_cmd).toBeDefined()
    expect(l0.x_meas).toBeDefined()
    expect(l0.residual).toBeDefined()

    expect(l1.frameType).toBe("user")
    expect(l1.constraintType).toBe("plane")
    expect(l1.T_base_user).toBeDefined()
    // second sample should usually have loop rate measured.
    expect(l1.loop_rate_hz === null || l1.loop_rate_hz > 0).toBe(true)

    __setConstraintLoggerEnabledForTests(null)
    clearConstraintLogs()
  })
})

