import { describe, expect, it } from "vitest"
import {
  applyResidualCorrection,
  autoScaleStepForDirectionDifference,
  computeTransitionWeight,
  computeBaseVsToolDirectionDifference,
  DEFAULT_CONSTRAINT_TUNING,
  estimateTcpPoseFromServoValues,
  projectAngularToSingleAxis,
  projectToLineDirection,
  projectToPlaneDirection,
  resolveConstraintDirection,
  rpyDegToRotationMatrix,
  solveWeightedDlsQdot,
  stepConstraintMotion,
} from "./frame-constraint-ik"

const IDENTITY: [[number, number, number], [number, number, number], [number, number, number]] = [
  [1, 0, 0],
  [0, 1, 0],
  [0, 0, 1],
]

function vecNorm(v: [number, number, number]) {
  return Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
}

function matMul3(a: number[][], b: number[][]): number[][] {
  return [
    [
      a[0][0] * b[0][0] + a[0][1] * b[1][0] + a[0][2] * b[2][0],
      a[0][0] * b[0][1] + a[0][1] * b[1][1] + a[0][2] * b[2][1],
      a[0][0] * b[0][2] + a[0][1] * b[1][2] + a[0][2] * b[2][2],
    ],
    [
      a[1][0] * b[0][0] + a[1][1] * b[1][0] + a[1][2] * b[2][0],
      a[1][0] * b[0][1] + a[1][1] * b[1][1] + a[1][2] * b[2][1],
      a[1][0] * b[0][2] + a[1][1] * b[1][2] + a[1][2] * b[2][2],
    ],
    [
      a[2][0] * b[0][0] + a[2][1] * b[1][0] + a[2][2] * b[2][0],
      a[2][0] * b[0][1] + a[2][1] * b[1][1] + a[2][2] * b[2][1],
      a[2][0] * b[0][2] + a[2][1] * b[1][2] + a[2][2] * b[2][2],
    ],
  ]
}

function matTranspose3(a: number[][]): number[][] {
  return [
    [a[0][0], a[1][0], a[2][0]],
    [a[0][1], a[1][1], a[2][1]],
    [a[0][2], a[1][2], a[2][2]],
  ]
}

describe("frame constraint direction", () => {
  it("maps user X line to base Y when user yaw is +90deg", () => {
    const userRotation = rpyDegToRotationMatrix(0, 0, 90)
    const result = resolveConstraintDirection(
      "user",
      "line",
      "x",
      "x",
      1,
      IDENTITY,
      {
        position: [0, 0, 0],
        rotation: userRotation,
      }
    )

    expect(result.direction).not.toBeNull()
    const [x, y, z] = result.direction!
    expect(Math.abs(x)).toBeLessThan(1e-6)
    expect(y).toBeGreaterThan(0.999)
    expect(Math.abs(z)).toBeLessThan(1e-6)
  })

  it("falls back to base frame with warning when user frame is missing", () => {
    const result = resolveConstraintDirection(
      "user",
      "line",
      "x",
      "x",
      1,
      IDENTITY,
      undefined
    )

    expect(result.direction).toEqual([1, 0, 0])
    expect(result.warning).toBe("user_frame_missing_fallback_to_base")
  })

  it("keeps base and tool behavior unchanged for identity tool rotation", () => {
    const base = resolveConstraintDirection(
      "base",
      "line",
      "z",
      "z",
      1,
      IDENTITY
    )
    const tool = resolveConstraintDirection(
      "tool",
      "line",
      "z",
      "z",
      1,
      IDENTITY
    )

    expect(base.direction).toEqual([0, 0, 1])
    expect(tool.direction).toEqual([0, 0, 1])
  })

  it("applies hard line projection even when jog axis differs from constraint axis", () => {
    const result = resolveConstraintDirection(
      "base",
      "line",
      "z",
      "x",
      1,
      IDENTITY
    )
    expect(result.direction).toEqual([0, 0, 0])
  })

  it("reflects user frame redefinition immediately on next computation", () => {
    const yaw0 = resolveConstraintDirection(
      "user",
      "line",
      "x",
      "x",
      1,
      IDENTITY,
      { position: [0, 0, 0], rotation: rpyDegToRotationMatrix(0, 0, 0) }
    )
    const yaw90 = resolveConstraintDirection(
      "user",
      "line",
      "x",
      "x",
      1,
      IDENTITY,
      { position: [0, 0, 0], rotation: rpyDegToRotationMatrix(0, 0, 90) }
    )

    expect(yaw0.direction).toEqual([1, 0, 0])
    expect(yaw90.direction).not.toEqual([1, 0, 0])
  })

  it("proves deterministic base vs tool direction gap at theta=45deg (angle >= 10deg)", () => {
    const toolRot = rpyDegToRotationMatrix(0, 45, 0)
    const diff = computeBaseVsToolDirectionDifference("line", "z", "z", 1, toolRot)
    expect(diff).not.toBeNull()
    expect(diff!.angleDeg).toBeGreaterThanOrEqual(10)
  })

  it("auto-scales comparison step from direction angle so difference remains observable", () => {
    const toolRot = rpyDegToRotationMatrix(0, 45, 0)
    const diff = computeBaseVsToolDirectionDifference("line", "z", "z", 1, toolRot)
    expect(diff).not.toBeNull()
    const step = autoScaleStepForDirectionDifference(diff!.angleDeg, 2, 1, 40)
    expect(step).toBeGreaterThan(1)
    expect(step).toBeLessThanOrEqual(40)
  })

  it("quantization diagnostics expose deadband for tiny line step while test mode can bypass", () => {
    const servoValues = { 1: 120, 2: 165, 3: 120, 4: 120, 5: 120, 6: 120 }
    const lockedAxes = { 1: false, 2: false, 3: false, 4: false, 5: false, 6: false }

    const quantized = stepConstraintMotion({
      servoValues,
      lockedAxes,
      frame: "tool",
      constraint: "line",
      constraintAxis: "z",
      jogAxis: "z",
      sign: 1,
      stepMm: 1,
    })
    const floatMode = stepConstraintMotion({
      servoValues,
      lockedAxes,
      frame: "tool",
      constraint: "line",
      constraintAxis: "z",
      jogAxis: "z",
      sign: 1,
      stepMm: 1,
      disableServoQuantization: true,
    })

    expect(quantized.ok).toBe(true)
    expect(floatMode.ok).toBe(true)
    expect(quantized.diagnostics).toBeDefined()
    expect(floatMode.diagnostics).toBeDefined()
    expect(
      (quantized.diagnostics!.postQuantizationDeadband || quantized.diagnostics!.hadQuantizationLoss) ||
      (!floatMode.diagnostics!.postQuantizationDeadband && !floatMode.diagnostics!.hadQuantizationLoss)
    ).toBe(true)
  })

  it("small-step input can still produce deterministic frame difference via direction utility", () => {
    const toolRot = rpyDegToRotationMatrix(0, 45, 0)
    const diff = computeBaseVsToolDirectionDifference("line", "z", "z", 1, toolRot)
    expect(diff).not.toBeNull()
    const smallStepMm = 1
    const expectedOrthDiffMm = smallStepMm * Math.sin((diff!.angleDeg * Math.PI) / 180)
    expect(diff!.angleDeg).toBeGreaterThanOrEqual(10)
    expect(expectedOrthDiffMm).toBeGreaterThan(0.1)
  })

  it("applies smooth transition weight after mode switch (no jump)", () => {
    const tau = 300
    const w0 = computeTransitionWeight(undefined, "base:plane:z", undefined, 1000, tau)
    const w1 = computeTransitionWeight("base:plane:z", "base:plane:z", 1000, 1100, tau)
    const w2 = computeTransitionWeight("base:plane:z", "base:plane:z", 1000, 1600, tau)
    expect(w0).toBe(0)
    expect(w1).toBeGreaterThan(0)
    expect(w1).toBeLessThan(w2)
    expect(w2).toBeLessThanOrEqual(1)
  })

  it("residual correction converges violation toward zero", () => {
    let residual: [number, number, number] = [0, 0, 0.004]
    const axis: [number, number, number] = [0, 0, 1]
    for (let i = 0; i < 20; i += 1) {
      const corrected = applyResidualCorrection(
        "plane",
        axis,
        [0, 0, 0],
        residual,
        DEFAULT_CONSTRAINT_TUNING.residualCorrectionGain,
        DEFAULT_CONSTRAINT_TUNING.residualCorrectionClampM,
        DEFAULT_CONSTRAINT_TUNING.residualEpsM
      )
      residual = [
        residual[0] + corrected[0],
        residual[1] + corrected[1],
        residual[2] + corrected[2],
      ]
    }
    expect(Math.abs(residual[2])).toBeLessThan(0.0008)
  })

  it("projects vectors for line/plane hard constraints", () => {
    const v: [number, number, number] = [1, 2, 3]
    const axis: [number, number, number] = [0, 0, 1]
    expect(projectToLineDirection(v, axis)).toEqual([0, 0, 3])
    expect(projectToPlaneDirection(v, axis)).toEqual([1, 2, 0])
  })

  it("limits angular command to one axis in point default mode", () => {
    const omega: [number, number, number] = [0.3, 0.2, 0.4]
    const projected = projectAngularToSingleAxis(omega, [0, 0, 1])
    expect(projected[0]).toBeCloseTo(0)
    expect(projected[1]).toBeCloseTo(0)
    expect(projected[2]).toBeCloseTo(0.4)
  })

  it("weighted DLS prioritizes position residual when Wp >> Wr", () => {
    const j = [
      [1, 0],
      [0, 1],
      [0, 0],
      [0, 1],
    ]
    const xdot = [1, 0, 0, 1]
    const highPos = solveWeightedDlsQdot(j, xdot, [10, 10, 10, 0.1], 0.01)
    const lowPos = solveWeightedDlsQdot(j, xdot, [0.1, 0.1, 0.1, 10], 0.01)
    expect(highPos).not.toBeNull()
    expect(lowPos).not.toBeNull()
    const posErrHigh = Math.abs((highPos?.[0] ?? 0) - 1)
    const posErrLow = Math.abs((lowPos?.[0] ?? 0) - 1)
    expect(posErrHigh).toBeLessThan(posErrLow)
  })

  it("stepConstraintMotion exposes transition metrics and continuity over ticks", () => {
    const servoValues = { 1: 120, 2: 150, 3: 120, 4: 120, 5: 120, 6: 120 }
    const lockedAxes = { 1: false, 2: false, 3: false, 4: false, 5: false, 6: false }
    const mode = { frame: "base", constraint: "plane", constraintAxis: "z" } as const

    const s0 = stepConstraintMotion({
      servoValues,
      lockedAxes,
      ...mode,
      jogAxis: "x",
      sign: 1,
      stepMm: 20,
      nowMs: 1000,
      controllerState: undefined,
      disableServoQuantization: true,
    })
    expect(s0.ok).toBe(true)
    expect(s0.diagnostics).toBeDefined()
    expect(s0.diagnostics!.transitionWeight).toBe(0)

    const s1 = stepConstraintMotion({
      servoValues: s0.nextServoValues!,
      lockedAxes,
      ...mode,
      jogAxis: "x",
      sign: 1,
      stepMm: 20,
      nowMs: 1450,
      controllerState: s0.nextControllerState,
      disableServoQuantization: true,
    })
    expect(s1.ok).toBe(true)
    expect(s1.diagnostics!.transitionWeight).toBeGreaterThan(0.7)
  })

  it("keeps TCP position near p0 while point rotation jog is commanded", () => {
    const servoValues = { 1: 120, 2: 120, 3: 120, 4: 120, 5: 120, 6: 120 }
    const anchorPose = estimateTcpPoseFromServoValues(servoValues)

    const result = stepConstraintMotion({
      servoValues,
      lockedAxes: { 1: false, 2: false, 3: false, 4: false, 5: false, 6: false },
      frame: "base",
      constraint: "point",
      constraintAxis: "z",
      jogAxis: "z",
      sign: 1,
      stepMm: 10,
      pointAnchorBase: anchorPose.position,
      rotationStepDeg: 8,
      tuning: { transitionTauMs: 0 },
    })

    expect(result.ok).toBe(true)
    expect(result.nextServoValues).toBeDefined()

    const pose2 = estimateTcpPoseFromServoValues(result.nextServoValues!)
    const dp: [number, number, number] = [
      pose2.position[0] - anchorPose.position[0],
      pose2.position[1] - anchorPose.position[1],
      pose2.position[2] - anchorPose.position[2],
    ]
    expect(vecNorm(dp)).toBeLessThan(0.04)
  })

  it("changes orientation with expected yaw sign for point z+ jog", () => {
    const servoValues = { 1: 120, 2: 120, 3: 120, 4: 120, 5: 120, 6: 120 }
    const pose1 = estimateTcpPoseFromServoValues(servoValues)
    const result = stepConstraintMotion({
      servoValues,
      lockedAxes: { 1: false, 2: false, 3: false, 4: false, 5: false, 6: false },
      frame: "base",
      constraint: "point",
      constraintAxis: "z",
      jogAxis: "z",
      sign: 1,
      stepMm: 10,
      pointAnchorBase: pose1.position,
      rotationStepDeg: 8,
      tuning: { transitionTauMs: 0 },
    })

    expect(result.ok).toBe(true)
    const pose2 = estimateTcpPoseFromServoValues(result.nextServoValues!)
    const rRel = matMul3(matTranspose3(pose1.rotation as unknown as number[][]), pose2.rotation as unknown as number[][])
    const yawApprox = Math.atan2(rRel[1][0], rRel[0][0])
    expect(Math.abs(yawApprox)).toBeGreaterThan(1e-6)
    expect(yawApprox).toBeGreaterThan(0)
  })
})
