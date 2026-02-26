export type ResidualFrameType = "base" | "tool" | "user"
export type ResidualConstraintType = "line" | "plane" | "point"
export type ResidualAxisName = "x" | "y" | "z"

export type Vec3 = [number, number, number]
export type Mat3 = [Vec3, Vec3, Vec3]

export interface Transform3Like {
  position: Vec3
  rotation: Mat3
}

export interface PoseLike {
  pos: Vec3
  quat: [number, number, number, number]
}

export interface ConstraintResidualInput {
  frameType: ResidualFrameType
  constraintType: ResidualConstraintType
  constraintAxis: ResidualAxisName
  xCmdPosBase: Vec3
  xMeasPosBase: Vec3
  pointAnchorBase?: Vec3
  tBaseTool?: Transform3Like
  tBaseUser?: Transform3Like
}

export interface ConstraintResidualOutput {
  deltaPosBase: Vec3
  axisOrNormalBase: Vec3
  plane_n_dot_dp: number
  line_orthogonal_error: number
  point_position_error: number
  active_residual: number
  warnings: string[]
}

export function axisToVec3(axis: ResidualAxisName): Vec3 {
  if (axis === "x") return [1, 0, 0]
  if (axis === "y") return [0, 1, 0]
  return [0, 0, 1]
}

export function vecSub(a: Vec3, b: Vec3): Vec3 {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

export function vecNorm(a: Vec3): number {
  return Math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])
}

export function vecDot(a: Vec3, b: Vec3): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

export function vecScale(a: Vec3, s: number): Vec3 {
  return [a[0] * s, a[1] * s, a[2] * s]
}

export function matVecMul(m: Mat3, v: Vec3): Vec3 {
  return [
    m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
    m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
    m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
  ]
}

function resolveAxisInBase(
  frameType: ResidualFrameType,
  axis: ResidualAxisName,
  tBaseTool?: Transform3Like,
  tBaseUser?: Transform3Like
): { axisBase: Vec3; warnings: string[] } {
  const warnings: string[] = []
  const axisVec = axisToVec3(axis)
  if (frameType === "base") {
    return { axisBase: axisVec, warnings }
  }
  if (frameType === "tool") {
    if (!tBaseTool) {
      warnings.push("missing_t_base_tool_fallback_to_base")
      return { axisBase: axisVec, warnings }
    }
    return { axisBase: matVecMul(tBaseTool.rotation, axisVec), warnings }
  }
  if (!tBaseUser) {
    warnings.push("missing_t_base_user_fallback_to_base")
    return { axisBase: axisVec, warnings }
  }
  return { axisBase: matVecMul(tBaseUser.rotation, axisVec), warnings }
}

export function rotationMatrixToQuaternion(m: Mat3): [number, number, number, number] {
  const trace = m[0][0] + m[1][1] + m[2][2]
  let x = 0
  let y = 0
  let z = 0
  let w = 1

  if (trace > 0) {
    const s = Math.sqrt(trace + 1) * 2
    w = 0.25 * s
    x = (m[2][1] - m[1][2]) / s
    y = (m[0][2] - m[2][0]) / s
    z = (m[1][0] - m[0][1]) / s
  } else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
    const s = Math.sqrt(1 + m[0][0] - m[1][1] - m[2][2]) * 2
    w = (m[2][1] - m[1][2]) / s
    x = 0.25 * s
    y = (m[0][1] + m[1][0]) / s
    z = (m[0][2] + m[2][0]) / s
  } else if (m[1][1] > m[2][2]) {
    const s = Math.sqrt(1 + m[1][1] - m[0][0] - m[2][2]) * 2
    w = (m[0][2] - m[2][0]) / s
    x = (m[0][1] + m[1][0]) / s
    y = 0.25 * s
    z = (m[1][2] + m[2][1]) / s
  } else {
    const s = Math.sqrt(1 + m[2][2] - m[0][0] - m[1][1]) * 2
    w = (m[1][0] - m[0][1]) / s
    x = (m[0][2] + m[2][0]) / s
    y = (m[1][2] + m[2][1]) / s
    z = 0.25 * s
  }

  return [x, y, z, w]
}

export function computeConstraintResidual(input: ConstraintResidualInput): ConstraintResidualOutput {
  const deltaPosBase = vecSub(input.xMeasPosBase, input.xCmdPosBase)
  const { axisBase, warnings } = resolveAxisInBase(
    input.frameType,
    input.constraintAxis,
    input.tBaseTool,
    input.tBaseUser
  )

  const planeNdotDp = vecDot(axisBase, deltaPosBase)
  const lineProj = vecScale(axisBase, vecDot(deltaPosBase, axisBase))
  const lineOrth = vecSub(deltaPosBase, lineProj)
  const lineResidual = vecNorm(lineOrth)
  const pointTarget = input.pointAnchorBase ?? input.xCmdPosBase
  const pointResidual = vecNorm(vecSub(input.xMeasPosBase, pointTarget))

  let activeResidual = lineResidual
  if (input.constraintType === "plane") activeResidual = Math.abs(planeNdotDp)
  if (input.constraintType === "point") activeResidual = pointResidual

  return {
    deltaPosBase,
    axisOrNormalBase: axisBase,
    plane_n_dot_dp: planeNdotDp,
    line_orthogonal_error: lineResidual,
    point_position_error: pointResidual,
    active_residual: activeResidual,
    warnings,
  }
}

