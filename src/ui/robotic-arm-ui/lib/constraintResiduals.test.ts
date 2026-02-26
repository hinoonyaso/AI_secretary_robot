import { describe, expect, it } from "vitest"
import { computeConstraintResidual } from "./constraintResiduals"

describe("constraint residuals", () => {
  it("plane residual uses n·Δp", () => {
    const out = computeConstraintResidual({
      frameType: "base",
      constraintType: "plane",
      constraintAxis: "z",
      xCmdPosBase: [0, 0, 0],
      xMeasPosBase: [0, 0, 0.01],
    })
    expect(out.plane_n_dot_dp).toBeCloseTo(0.01, 8)
    expect(out.active_residual).toBeCloseTo(0.01, 8)
  })

  it("line residual equals orthogonal leakage", () => {
    const out = computeConstraintResidual({
      frameType: "base",
      constraintType: "line",
      constraintAxis: "x",
      xCmdPosBase: [0, 0, 0],
      xMeasPosBase: [0.1, 0.03, 0.04],
    })
    expect(out.line_orthogonal_error).toBeCloseTo(Math.sqrt(0.03 * 0.03 + 0.04 * 0.04), 8)
    expect(out.active_residual).toBeCloseTo(out.line_orthogonal_error, 8)
  })

  it("point residual uses anchor distance", () => {
    const out = computeConstraintResidual({
      frameType: "base",
      constraintType: "point",
      constraintAxis: "z",
      xCmdPosBase: [0, 0, 0],
      xMeasPosBase: [0.001, 0.002, 0.002],
      pointAnchorBase: [0, 0, 0],
    })
    expect(out.point_position_error).toBeCloseTo(0.003, 8)
    expect(out.active_residual).toBeCloseTo(0.003, 8)
  })
})

