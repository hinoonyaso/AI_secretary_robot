import { addConstraintLog } from "./constraintLogger"
import { computeConstraintResidual, rotationMatrixToQuaternion } from "./constraintResiduals"

export type AxisName = "x" | "y" | "z"
export type FrameType = "base" | "tool" | "user"
export type ConstraintType = "line" | "plane" | "point"

type Vec3 = [number, number, number]
type Mat3 = [Vec3, Vec3, Vec3]

export interface Transform3 {
  position: Vec3
  rotation: Mat3
}

interface KinematicConfig {
  linksM: {
    l1: number
    l2: number
    l3: number
    l4: number
    l5: number
  }
  servoCenterDeg: number[]
  servoDirection: number[]
  servoMinDeg: number[]
  servoMaxDeg: number[]
}

export const DEFAULT_KINEMATIC_CONFIG: KinematicConfig = {
  // Approximate lengths; tune for your arm geometry.
  linksM: { l1: 0.09, l2: 0.11, l3: 0.11, l4: 0.07, l5: 0.04 },
  servoCenterDeg: [120, 120, 120, 120, 120, 120],
  servoDirection: [1, 1, 1, 1, 1, 1],
  servoMinDeg: [0, 0, 0, 0, 0, 0],
  servoMaxDeg: [240, 240, 240, 240, 240, 240],
}

interface StepConstraintInput {
  servoValues: Record<number, number>
  lockedAxes: Record<number, boolean>
  frame: FrameType
  constraint: ConstraintType
  constraintAxis: AxisName
  jogAxis: AxisName
  sign: -1 | 1
  stepMm: number
  userFrame?: Transform3
  pointAnchorBase?: Vec3
  rotationStepDeg?: number
  pointAllow3AxisRotation?: boolean
  pointRotationAxis?: AxisName
  disableServoQuantization?: boolean
  nowMs?: number
  controllerState?: ConstraintControllerState
  tuning?: Partial<ConstraintTuning>
  config?: KinematicConfig
}

interface StepConstraintResult {
  ok: boolean
  nextServoValues?: Record<number, number>
  error?: string
  warning?: string
  diagnostics?: {
    requestedStepMm: number
    hasSaturation: boolean
    hadQuantizationLoss: boolean
    postQuantizationDeadband: boolean
    minServoDeltaDeg: number
    maxServoDeltaDeg: number
    transitionWeight: number
    conditionNumber: number
    appliedDampingLambda: number
  }
  nextControllerState?: ConstraintControllerState
}

export interface ConstraintControllerState {
  modeKey?: string
  modeSinceMs?: number
  lastResidualVecBase?: Vec3
}

interface PointIkWeights {
  wp: number
  wr: number
}

export interface ConstraintTuning {
  residualEpsM: number
  residualCorrectionGain: number
  residualCorrectionClampM: number
  maxStepMm: number
  transitionTauMs: number
  baseDampingLambda: number
  maxDampingLambda: number
  conditionThreshold: number
  singularStepScaleMin: number
}

export const DEFAULT_CONSTRAINT_TUNING: ConstraintTuning = {
  residualEpsM: 0.0005,
  residualCorrectionGain: 0.7,
  residualCorrectionClampM: 0.002,
  maxStepMm: 40,
  transitionTauMs: 300,
  baseDampingLambda: 0.03,
  maxDampingLambda: 0.25,
  conditionThreshold: 60,
  singularStepScaleMin: 0.2,
}

function degToRad(v: number) {
  return (v * Math.PI) / 180
}

function radToDeg(v: number) {
  return (v * 180) / Math.PI
}

function clamp(v: number, min: number, max: number) {
  return Math.max(min, Math.min(max, v))
}

function axisToVec3(axis: AxisName): Vec3 {
  if (axis === "x") return [1, 0, 0]
  if (axis === "y") return [0, 1, 0]
  return [0, 0, 1]
}

function vecAdd(a: Vec3, b: Vec3): Vec3 {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

function vecSub(a: Vec3, b: Vec3): Vec3 {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

function vecScale(a: Vec3, s: number): Vec3 {
  return [a[0] * s, a[1] * s, a[2] * s]
}

function vecNorm(a: Vec3): number {
  return Math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])
}

function vecDot(a: Vec3, b: Vec3): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

function normalizeVec(a: Vec3): Vec3 {
  const n = vecNorm(a)
  if (n < 1e-12) return [0, 0, 0]
  return [a[0] / n, a[1] / n, a[2] / n]
}

function vecClampNorm(a: Vec3, maxNorm: number): Vec3 {
  const n = vecNorm(a)
  if (n <= maxNorm || n < 1e-12) return a
  return vecScale(a, maxNorm / n)
}

function cross(a: Vec3, b: Vec3): Vec3 {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ]
}

function matVecMul(m: Mat3, v: Vec3): Vec3 {
  return [
    m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
    m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
    m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
  ]
}

function matMul(a: Mat3, b: Mat3): Mat3 {
  const r0: Vec3 = [
    a[0][0] * b[0][0] + a[0][1] * b[1][0] + a[0][2] * b[2][0],
    a[0][0] * b[0][1] + a[0][1] * b[1][1] + a[0][2] * b[2][1],
    a[0][0] * b[0][2] + a[0][1] * b[1][2] + a[0][2] * b[2][2],
  ]
  const r1: Vec3 = [
    a[1][0] * b[0][0] + a[1][1] * b[1][0] + a[1][2] * b[2][0],
    a[1][0] * b[0][1] + a[1][1] * b[1][1] + a[1][2] * b[2][1],
    a[1][0] * b[0][2] + a[1][1] * b[1][2] + a[1][2] * b[2][2],
  ]
  const r2: Vec3 = [
    a[2][0] * b[0][0] + a[2][1] * b[1][0] + a[2][2] * b[2][0],
    a[2][0] * b[0][1] + a[2][1] * b[1][1] + a[2][2] * b[2][1],
    a[2][0] * b[0][2] + a[2][1] * b[1][2] + a[2][2] * b[2][2],
  ]
  return [r0, r1, r2]
}

function rotX(a: number): Mat3 {
  const c = Math.cos(a)
  const s = Math.sin(a)
  return [
    [1, 0, 0],
    [0, c, -s],
    [0, s, c],
  ]
}

function rotY(a: number): Mat3 {
  const c = Math.cos(a)
  const s = Math.sin(a)
  return [
    [c, 0, s],
    [0, 1, 0],
    [-s, 0, c],
  ]
}

function rotZ(a: number): Mat3 {
  const c = Math.cos(a)
  const s = Math.sin(a)
  return [
    [c, -s, 0],
    [s, c, 0],
    [0, 0, 1],
  ]
}

export function rpyDegToRotationMatrix(rollDeg: number, pitchDeg: number, yawDeg: number): Mat3 {
  const roll = degToRad(rollDeg)
  const pitch = degToRad(pitchDeg)
  const yaw = degToRad(yawDeg)
  return matMul(matMul(rotZ(yaw), rotY(pitch)), rotX(roll))
}

function servoDegToJointRad(servoDeg: number, jointIndex0: number, cfg: KinematicConfig) {
  const center = cfg.servoCenterDeg[jointIndex0]
  const direction = cfg.servoDirection[jointIndex0]
  return degToRad((servoDeg - center) * direction)
}

function jointRadToServoDeg(jointRad: number, jointIndex0: number, cfg: KinematicConfig) {
  const center = cfg.servoCenterDeg[jointIndex0]
  const direction = cfg.servoDirection[jointIndex0]
  const raw = center + radToDeg(jointRad) * direction
  return clamp(raw, cfg.servoMinDeg[jointIndex0], cfg.servoMaxDeg[jointIndex0])
}

function forwardKinematics(jointRad: number[], cfg: KinematicConfig) {
  let r: Mat3 = [
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
  ]
  let p: Vec3 = [0, 0, 0]

  r = matMul(r, rotZ(jointRad[0]))
  p = vecAdd(p, matVecMul(r, [0, 0, cfg.linksM.l1]))

  r = matMul(r, rotY(jointRad[1]))
  p = vecAdd(p, matVecMul(r, [cfg.linksM.l2, 0, 0]))

  r = matMul(r, rotY(jointRad[2]))
  p = vecAdd(p, matVecMul(r, [cfg.linksM.l3, 0, 0]))

  r = matMul(r, rotY(jointRad[3]))
  p = vecAdd(p, matVecMul(r, [cfg.linksM.l4, 0, 0]))

  r = matMul(r, rotX(jointRad[4]))
  p = vecAdd(p, matVecMul(r, [cfg.linksM.l5, 0, 0]))

  return { position: p, rotation: r }
}

function invert3x3(m: Mat3): Mat3 | null {
  const a = m[0][0]
  const b = m[0][1]
  const c = m[0][2]
  const d = m[1][0]
  const e = m[1][1]
  const f = m[1][2]
  const g = m[2][0]
  const h = m[2][1]
  const i = m[2][2]

  const A = e * i - f * h
  const B = -(d * i - f * g)
  const C = d * h - e * g
  const D = -(b * i - c * h)
  const E = a * i - c * g
  const F = -(a * h - b * g)
  const G = b * f - c * e
  const H = -(a * f - c * d)
  const I = a * e - b * d

  const det = a * A + b * B + c * C
  if (Math.abs(det) < 1e-9) {
    return null
  }

  const invDet = 1 / det
  return [
    [A * invDet, D * invDet, G * invDet],
    [B * invDet, E * invDet, H * invDet],
    [C * invDet, F * invDet, I * invDet],
  ]
}

function solveLinearSystem(a: number[][], b: number[]): number[] | null {
  const n = a.length
  const m = a.map((row) => [...row])
  const rhs = [...b]

  for (let col = 0; col < n; col += 1) {
    let pivot = col
    for (let row = col + 1; row < n; row += 1) {
      if (Math.abs(m[row][col]) > Math.abs(m[pivot][col])) {
        pivot = row
      }
    }
    if (Math.abs(m[pivot][col]) < 1e-12) return null
    if (pivot !== col) {
      const tmpRow = m[col]
      m[col] = m[pivot]
      m[pivot] = tmpRow
      const tmpRhs = rhs[col]
      rhs[col] = rhs[pivot]
      rhs[pivot] = tmpRhs
    }

    const diag = m[col][col]
    for (let k = col; k < n; k += 1) {
      m[col][k] /= diag
    }
    rhs[col] /= diag

    for (let row = 0; row < n; row += 1) {
      if (row === col) continue
      const factor = m[row][col]
      if (Math.abs(factor) < 1e-14) continue
      for (let k = col; k < n; k += 1) {
        m[row][k] -= factor * m[col][k]
      }
      rhs[row] -= factor * rhs[col]
    }
  }
  return rhs
}

/**
 * Weighted DLS solve:
 * qdot = argmin ||W (J qdot - xdot)||^2 + lambda^2 ||qdot||^2
 *      = (J^T W^2 J + lambda^2 I)^-1 J^T W^2 xdot
 */
export function solveWeightedDlsQdot(
  j: number[][],
  xdot: number[],
  taskWeights: number[],
  lambda: number
): number[] | null {
  const m = j.length
  const n = j[0]?.length ?? 0
  if (m === 0 || n === 0 || xdot.length !== m || taskWeights.length !== m) return null
  const normal: number[][] = Array.from({ length: n }, () => Array.from({ length: n }, () => 0))
  const rhs: number[] = Array.from({ length: n }, () => 0)

  for (let row = 0; row < m; row += 1) {
    const w2 = taskWeights[row] * taskWeights[row]
    for (let colI = 0; colI < n; colI += 1) {
      const ji = j[row][colI]
      rhs[colI] += ji * w2 * xdot[row]
      for (let colK = 0; colK < n; colK += 1) {
        normal[colI][colK] += ji * w2 * j[row][colK]
      }
    }
  }

  for (let i = 0; i < n; i += 1) {
    normal[i][i] += lambda * lambda
  }

  return solveLinearSystem(normal, rhs)
}

function axisVectorInBase(
  frame: FrameType,
  jogAxis: AxisName,
  toolRotation: Mat3,
  userFrame?: Transform3
): { vector: Vec3; warning?: string } {
  const axisVec = axisToVec3(jogAxis)
  if (frame === "base") {
    return { vector: axisVec }
  }
  if (frame === "tool") {
    return { vector: matVecMul(toolRotation, axisVec) }
  }
  if (!userFrame) {
    return { vector: axisVec, warning: "user_frame_missing_fallback_to_base" }
  }
  // Direction vector transform uses rotation only (R * v), not translation.
  return { vector: matVecMul(userFrame.rotation, axisVec) }
}

function safeNormalizeVector(vec: Vec3, fallback: Vec3): Vec3 {
  const n = vecNorm(vec)
  if (n < 1e-9) return fallback
  return vecScale(vec, 1 / n)
}

/**
 * Hard line constraint projection:
 * v_line = (v·u_hat) * u_hat
 */
export function projectToLineDirection(v: Vec3, axis: Vec3): Vec3 {
  const uHat = safeNormalizeVector(axis, [1, 0, 0])
  return vecScale(uHat, vecDot(v, uHat))
}

/**
 * Hard plane constraint projection:
 * v_plane = v - (v·n_hat) * n_hat
 */
export function projectToPlaneDirection(v: Vec3, normal: Vec3): Vec3 {
  const nHat = safeNormalizeVector(normal, [0, 0, 1])
  return vecSub(v, vecScale(nHat, vecDot(v, nHat)))
}

/**
 * Default point-mode angular policy (1-axis):
 * ω_1axis = (ω·a_hat) * a_hat
 */
export function projectAngularToSingleAxis(omega: Vec3, axis: Vec3): Vec3 {
  const aHat = safeNormalizeVector(axis, [0, 0, 1])
  return vecScale(aHat, vecDot(omega, aHat))
}

function applyHardConstraintMotion(
  constraint: ConstraintType,
  rawMotionBase: Vec3,
  constraintAxisBase: Vec3
): Vec3 {
  if (constraint === "line") {
    return projectToLineDirection(rawMotionBase, constraintAxisBase)
  }
  if (constraint === "plane") {
    return projectToPlaneDirection(rawMotionBase, constraintAxisBase)
  }
  return rawMotionBase
}

export function resolveConstraintDirection(
  frame: FrameType,
  constraint: ConstraintType,
  constraintAxis: AxisName,
  jogAxis: AxisName,
  sign: -1 | 1,
  toolRotation: Mat3,
  userFrame?: Transform3
): { direction: Vec3 | null; warning?: string } {
  const { vector: jogVec, warning } = axisVectorInBase(frame, jogAxis, toolRotation, userFrame)
  const { vector: constraintVec } = axisVectorInBase(frame, constraintAxis, toolRotation, userFrame)
  const raw = vecScale(jogVec, sign)
  const direction = applyHardConstraintMotion(constraint, raw, constraintVec)
  return { direction, warning }
}

export interface DirectionDifferenceResult {
  baseDirection: Vec3
  toolDirection: Vec3
  angleDeg: number
}

export function directionAngleDeg(a: Vec3, b: Vec3): number {
  const na = normalizeVec(a)
  const nb = normalizeVec(b)
  const c = clamp(vecDot(na, nb), -1, 1)
  return radToDeg(Math.acos(c))
}

export function computeBaseVsToolDirectionDifference(
  constraint: ConstraintType,
  constraintAxis: AxisName,
  jogAxis: AxisName,
  sign: -1 | 1,
  toolRotation: Mat3
): DirectionDifferenceResult | null {
  const base = resolveConstraintDirection("base", constraint, constraintAxis, jogAxis, sign, toolRotation)
  const tool = resolveConstraintDirection("tool", constraint, constraintAxis, jogAxis, sign, toolRotation)
  if (!base.direction || !tool.direction) return null
  return {
    baseDirection: base.direction,
    toolDirection: tool.direction,
    angleDeg: directionAngleDeg(base.direction, tool.direction),
  }
}

export function autoScaleStepForDirectionDifference(
  angleDeg: number,
  targetOrthogonalDiffMm = 2,
  minStepMm = 1,
  maxStepMm = 40
) {
  const s = Math.sin(degToRad(Math.max(0, angleDeg)))
  if (s < 1e-6) return maxStepMm
  const needed = targetOrthogonalDiffMm / s
  return clamp(needed, minStepMm, maxStepMm)
}

function modeKey(frame: FrameType, constraint: ConstraintType, constraintAxis: AxisName) {
  return `${frame}:${constraint}:${constraintAxis}`
}

export function computeTransitionWeight(
  prevModeKey: string | undefined,
  nextModeKey: string,
  modeSinceMs: number | undefined,
  nowMs: number,
  tauMs: number
) {
  if (!prevModeKey || prevModeKey !== nextModeKey || modeSinceMs === undefined) {
    return tauMs <= 0 ? 1 : 0
  }
  if (tauMs <= 0) return 1
  const dt = Math.max(0, nowMs - modeSinceMs)
  return clamp(1 - Math.exp(-dt / tauMs), 0, 1)
}

function mat3Vec(m: Mat3, v: Vec3): Vec3 {
  return matVecMul(m, v)
}

function estimateConditionNumberFromJJt(jjt: Mat3): number {
  // Power iteration for max eigenvalue.
  let v: Vec3 = [1, 0, 0]
  for (let i = 0; i < 8; i += 1) {
    v = normalizeVec(mat3Vec(jjt, v))
  }
  const mv = mat3Vec(jjt, v)
  const lambdaMax = Math.max(1e-12, vecDot(v, mv))

  const inv = invert3x3(jjt)
  if (!inv) return Number.POSITIVE_INFINITY

  let w: Vec3 = [0, 1, 0]
  for (let i = 0; i < 8; i += 1) {
    w = normalizeVec(mat3Vec(inv, w))
  }
  const iww = mat3Vec(inv, w)
  const invLambda = Math.max(1e-12, vecDot(w, iww))
  const lambdaMin = 1 / invLambda
  if (lambdaMin <= 1e-12) return Number.POSITIVE_INFINITY
  return Math.sqrt(lambdaMax / lambdaMin)
}

function projectToConstraintResidual(
  constraint: ConstraintType,
  axisBase: Vec3,
  residualVec: Vec3
) {
  if (constraint === "line") {
    const lineProj = vecScale(axisBase, vecDot(residualVec, axisBase))
    return vecSub(residualVec, lineProj)
  }
  if (constraint === "plane") {
    return vecScale(axisBase, vecDot(residualVec, axisBase))
  }
  return residualVec
}

export function applyResidualCorrection(
  constraint: ConstraintType,
  axisBase: Vec3,
  targetDeltaBase: Vec3,
  residualVecBase: Vec3 | undefined,
  gain: number,
  maxCorrectionM: number,
  epsM: number
) {
  if (!residualVecBase) return targetDeltaBase
  const violating = projectToConstraintResidual(constraint, axisBase, residualVecBase)
  const vNorm = vecNorm(violating)
  if (vNorm <= epsM) return targetDeltaBase
  const correction = vecClampNorm(vecScale(violating, -gain), maxCorrectionM)
  return vecAdd(targetDeltaBase, correction)
}

function computeKinematicState(jointRad: number[], cfg: KinematicConfig) {
  let r: Mat3 = [
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
  ]
  let p: Vec3 = [0, 0, 0]

  const jointOrigins: Vec3[] = []
  const jointAxes: Vec3[] = []

  jointOrigins.push(p)
  jointAxes.push(matVecMul(r, [0, 0, 1]))
  r = matMul(r, rotZ(jointRad[0]))
  p = vecAdd(p, matVecMul(r, [0, 0, cfg.linksM.l1]))

  jointOrigins.push(p)
  jointAxes.push(matVecMul(r, [0, 1, 0]))
  r = matMul(r, rotY(jointRad[1]))
  p = vecAdd(p, matVecMul(r, [cfg.linksM.l2, 0, 0]))

  jointOrigins.push(p)
  jointAxes.push(matVecMul(r, [0, 1, 0]))
  r = matMul(r, rotY(jointRad[2]))
  p = vecAdd(p, matVecMul(r, [cfg.linksM.l3, 0, 0]))

  jointOrigins.push(p)
  jointAxes.push(matVecMul(r, [0, 1, 0]))
  r = matMul(r, rotY(jointRad[3]))
  p = vecAdd(p, matVecMul(r, [cfg.linksM.l4, 0, 0]))

  jointOrigins.push(p)
  jointAxes.push(matVecMul(r, [1, 0, 0]))
  r = matMul(r, rotX(jointRad[4]))
  p = vecAdd(p, matVecMul(r, [cfg.linksM.l5, 0, 0]))

  return { position: p, rotation: r, jointOrigins, jointAxes }
}

function buildGeometricJacobians(
  state: ReturnType<typeof computeKinematicState>,
  jointLocked: boolean[]
) {
  const jv: number[][] = [[], [], []]
  const jw: number[][] = [[], [], []]

  for (let col = 0; col < 5; col += 1) {
    if (jointLocked[col]) {
      jv[0][col] = 0
      jv[1][col] = 0
      jv[2][col] = 0
      jw[0][col] = 0
      jw[1][col] = 0
      jw[2][col] = 0
      continue
    }
    const axis = state.jointAxes[col]
    const origin = state.jointOrigins[col]
    const r = vecSub(state.position, origin)
    const jvCol = cross(axis, r)
    jv[0][col] = jvCol[0]
    jv[1][col] = jvCol[1]
    jv[2][col] = jvCol[2]
    jw[0][col] = axis[0]
    jw[1][col] = axis[1]
    jw[2][col] = axis[2]
  }

  const j6: number[][] = [
    [jv[0][0], jv[0][1], jv[0][2], jv[0][3], jv[0][4]],
    [jv[1][0], jv[1][1], jv[1][2], jv[1][3], jv[1][4]],
    [jv[2][0], jv[2][1], jv[2][2], jv[2][3], jv[2][4]],
    [jw[0][0], jw[0][1], jw[0][2], jw[0][3], jw[0][4]],
    [jw[1][0], jw[1][1], jw[1][2], jw[1][3], jw[1][4]],
    [jw[2][0], jw[2][1], jw[2][2], jw[2][3], jw[2][4]],
  ]

  return { jv, jw, j6 }
}

export function estimateTcpPoseFromServoValues(
  servoValues: Record<number, number>,
  config: KinematicConfig = DEFAULT_KINEMATIC_CONFIG
) {
  const q = [
    servoDegToJointRad(servoValues[1], 0, config),
    servoDegToJointRad(servoValues[2], 1, config),
    servoDegToJointRad(servoValues[3], 2, config),
    servoDegToJointRad(servoValues[4], 3, config),
    servoDegToJointRad(servoValues[5], 4, config),
  ]
  const state = computeKinematicState(q, config)
  return { position: state.position, rotation: state.rotation }
}

export function stepConstraintMotion({
  servoValues,
  lockedAxes,
  frame,
  constraint,
  constraintAxis,
  jogAxis,
  sign,
  stepMm,
  userFrame,
  pointAnchorBase,
  rotationStepDeg = 5,
  pointAllow3AxisRotation = false,
  pointRotationAxis = "z",
  disableServoQuantization = false,
  nowMs,
  controllerState,
  tuning,
  config = DEFAULT_KINEMATIC_CONFIG,
}: StepConstraintInput): StepConstraintResult {
  const startedAt = Date.now()
  const now = nowMs ?? startedAt
  const tune: ConstraintTuning = { ...DEFAULT_CONSTRAINT_TUNING, ...(tuning ?? {}) }
  const currentModeKey = modeKey(frame, constraint, constraintAxis)
  const prevState = controllerState ?? {}
  const modeSinceMs =
    !prevState.modeKey || prevState.modeKey !== currentModeKey
      ? now
      : (prevState.modeSinceMs ?? now)
  const transitionWeight = computeTransitionWeight(
    prevState.modeKey,
    currentModeKey,
    modeSinceMs,
    now,
    tune.transitionTauMs
  )
  const requestedStepMm = clamp(stepMm, 0, tune.maxStepMm)
  const q = [
    servoDegToJointRad(servoValues[1], 0, config),
    servoDegToJointRad(servoValues[2], 1, config),
    servoDegToJointRad(servoValues[3], 2, config),
    servoDegToJointRad(servoValues[4], 3, config),
    servoDegToJointRad(servoValues[5], 4, config),
  ]

  const fk = forwardKinematics(q, config)
  if (constraint === "point") {
    const state = computeKinematicState(q, config)
    const jointLocked = [lockedAxes[1], lockedAxes[2], lockedAxes[3], lockedAxes[4], lockedAxes[5]]
    const movableCount = jointLocked.filter((v) => !v).length
    if (movableCount === 0) {
      return { ok: false, error: "all_ik_joints_locked" }
    }

    const directionResult = resolveConstraintDirection(
      frame,
      "point",
      constraintAxis,
      jogAxis,
      sign,
      state.rotation,
      userFrame
    )
    const warningParts: string[] = []
    if (directionResult.warning) {
      warningParts.push(directionResult.warning)
    }

    const anchor = pointAnchorBase ?? state.position
    if (!pointAnchorBase) {
      warningParts.push("point_anchor_missing_fallback_to_current_tcp")
    }
    const ePos = vecSub(anchor, state.position)
    const kp = 2.0
    const maxCorrStepM = 0.003
    const vTarget = vecClampNorm(vecScale(ePos, kp), maxCorrStepM)

    const omegaAxisRaw = safeNormalizeVector(directionResult.direction ?? [0, 0, 1], [0, 0, 1])
    const omegaMag = degToRad(Math.max(0.1, rotationStepDeg)) * transitionWeight
    const omegaUser = vecScale(omegaAxisRaw, omegaMag)
    const { vector: pointAxisVec } = axisVectorInBase(
      frame,
      pointRotationAxis ?? "z",
      state.rotation,
      userFrame
    )
    const omegaTarget = pointAllow3AxisRotation
      ? omegaUser
      : projectAngularToSingleAxis(omegaUser, pointAxisVec)

    const { jv, j6 } = buildGeometricJacobians(state, jointLocked)

    const rawPointJJt: Mat3 = [
      [
        jv[0][0] * jv[0][0] + jv[0][1] * jv[0][1] + jv[0][2] * jv[0][2] + jv[0][3] * jv[0][3] + jv[0][4] * jv[0][4],
        jv[0][0] * jv[1][0] + jv[0][1] * jv[1][1] + jv[0][2] * jv[1][2] + jv[0][3] * jv[1][3] + jv[0][4] * jv[1][4],
        jv[0][0] * jv[2][0] + jv[0][1] * jv[2][1] + jv[0][2] * jv[2][2] + jv[0][3] * jv[2][3] + jv[0][4] * jv[2][4],
      ],
      [
        jv[1][0] * jv[0][0] + jv[1][1] * jv[0][1] + jv[1][2] * jv[0][2] + jv[1][3] * jv[0][3] + jv[1][4] * jv[0][4],
        jv[1][0] * jv[1][0] + jv[1][1] * jv[1][1] + jv[1][2] * jv[1][2] + jv[1][3] * jv[1][3] + jv[1][4] * jv[1][4],
        jv[1][0] * jv[2][0] + jv[1][1] * jv[2][1] + jv[1][2] * jv[2][2] + jv[1][3] * jv[2][3] + jv[1][4] * jv[2][4],
      ],
      [
        jv[2][0] * jv[0][0] + jv[2][1] * jv[0][1] + jv[2][2] * jv[0][2] + jv[2][3] * jv[0][3] + jv[2][4] * jv[0][4],
        jv[2][0] * jv[1][0] + jv[2][1] * jv[1][1] + jv[2][2] * jv[1][2] + jv[2][3] * jv[1][3] + jv[2][4] * jv[1][4],
        jv[2][0] * jv[2][0] + jv[2][1] * jv[2][1] + jv[2][2] * jv[2][2] + jv[2][3] * jv[2][3] + jv[2][4] * jv[2][4],
      ],
    ]
    const pointCondition = estimateConditionNumberFromJJt(rawPointJJt)
    const pointOver = Math.max(0, pointCondition - tune.conditionThreshold)
    const pointRatio =
      tune.conditionThreshold > 0 ? clamp(pointOver / tune.conditionThreshold, 0, 1) : 1
    const minJointMarginDeg = q
      .map((jointRad, i) => {
        const deg = jointRadToServoDeg(jointRad, i, config)
        const lowerMargin = Math.abs(deg - config.servoMinDeg[i])
        const upperMargin = Math.abs(config.servoMaxDeg[i] - deg)
        return Math.min(lowerMargin, upperMargin)
      })
      .reduce((minV, v) => Math.min(minV, v), Number.POSITIVE_INFINITY)
    const jointLimitScale = clamp(minJointMarginDeg / 15, 0.1, 1)
    const wrScale = clamp((1 - pointRatio) * jointLimitScale, 0.1, 1)
    const lambda =
      (tune.baseDampingLambda + (tune.maxDampingLambda - tune.baseDampingLambda) * pointRatio) * 1.35

    const weights: PointIkWeights = {
      wp: 8,
      wr: (pointAllow3AxisRotation ? 1 : 0.5) * wrScale,
    }
    const xdot = [vTarget[0], vTarget[1], vTarget[2], omegaTarget[0], omegaTarget[1], omegaTarget[2]]
    const taskWeights = [weights.wp, weights.wp, weights.wp, weights.wr, weights.wr, weights.wr]
    const dqRaw = solveWeightedDlsQdot(j6, xdot, taskWeights, lambda)
    if (!dqRaw) {
      return { ok: false, error: "point_ik_solve_failed" }
    }
    const dq = dqRaw.map((v) => clamp(v, -0.22, 0.22))
    const hasSaturation = dq.some((v) => Math.abs(v) >= 0.219999)
    const qNext = q.map((v, idx) => v + dq[idx])
    const nextServoValues = { ...servoValues }
    const rawServoNext: number[] = []
    const quantizedServoNext: number[] = []
    for (let i = 0; i < 5; i += 1) {
      // Keep float resolution in point mode to reduce anchor drift from quantization.
      const raw = jointRadToServoDeg(qNext[i], i, config)
      rawServoNext[i] = raw
      const quantized = Number(raw.toFixed(3))
      quantizedServoNext[i] = quantized
      nextServoValues[i + 1] = disableServoQuantization ? raw : quantized
    }
    nextServoValues[6] = servoValues[6]
    const deltas = rawServoNext.map((v, i) => v - servoValues[i + 1])
    const qDeltas = quantizedServoNext.map((v, i) => v - servoValues[i + 1])
    const hadQuantizationLoss = deltas.some((v, i) => Math.abs(v) > 1e-9 && Math.abs(qDeltas[i]) === 0)
    const postQuantizationDeadband = qDeltas.every((v) => Math.abs(v) === 0) && deltas.some((v) => Math.abs(v) > 1e-9)
    const absD = deltas.map((v) => Math.abs(v))
    const minServoDeltaDeg = absD.length > 0 ? Math.min(...absD) : 0
    const maxServoDeltaDeg = absD.length > 0 ? Math.max(...absD) : 0

    const nextPose = estimateTcpPoseFromServoValues(nextServoValues, config)
    const residual = computeConstraintResidual({
      frameType: frame,
      constraintType: constraint,
      constraintAxis,
      xCmdPosBase: anchor,
      xMeasPosBase: nextPose.position,
      pointAnchorBase: anchor,
      tBaseTool: { position: nextPose.position, rotation: nextPose.rotation },
      tBaseUser: userFrame,
    })
    addConstraintLog({
      frameType: frame,
      constraintType: constraint,
      constraintAxis,
      axisVecOrNormalBase: residual.axisOrNormalBase,
      x_cmd: {
        mode: "twist",
        twist_v: vTarget,
        twist_w: omegaTarget,
      },
      x_meas: {
        pos: nextPose.position,
        quat: rotationMatrixToQuaternion(nextPose.rotation),
        source: "estimated_fk",
        note: "x_meas derived from FK of computed next servo values",
      },
      residual: {
        active: residual.active_residual,
        plane_n_dot_dp: residual.plane_n_dot_dp,
        line_orthogonal_error: residual.line_orthogonal_error,
        point_position_error: residual.point_position_error,
      },
      T_base_tool: { position: nextPose.position, rotation: nextPose.rotation },
      T_base_user: userFrame,
      latency_ms: Date.now() - startedAt,
      warnings: warningParts.concat(residual.warnings),
      point_debug: {
        p0: anchor,
        p_meas: state.position,
        e_p_norm: vecNorm(ePos),
        omega_des: omegaTarget,
        wr_scale: wrScale,
        lambda,
        cond_j: pointCondition,
      },
    })

    return {
      ok: true,
      nextServoValues,
      warning: warningParts.length > 0 ? warningParts.join("|") : undefined,
      diagnostics: {
        requestedStepMm,
        hasSaturation,
        hadQuantizationLoss,
        postQuantizationDeadband,
        minServoDeltaDeg,
        maxServoDeltaDeg,
        transitionWeight,
        conditionNumber: pointCondition,
        appliedDampingLambda: lambda,
      },
      nextControllerState: {
        modeKey: currentModeKey,
        modeSinceMs,
        lastResidualVecBase: residual.deltaPosBase,
      },
    }
  }

  const { vector: jogVectorBase, warning: jogWarning } = axisVectorInBase(frame, jogAxis, fk.rotation, userFrame)
  const { vector: constraintAxisBase, warning: axisWarning } = axisVectorInBase(
    frame,
    constraintAxis,
    fk.rotation,
    userFrame
  )
  const stepAfterTransitionMm = requestedStepMm * transitionWeight
  const rawDelta = vecScale(jogVectorBase, sign * stepAfterTransitionMm / 1000)
  const targetDeltaRaw = applyHardConstraintMotion(constraint, rawDelta, constraintAxisBase)
  const targetDelta = applyResidualCorrection(
    constraint,
    safeNormalizeVector(constraintAxisBase, axisToVec3(constraintAxis)),
    targetDeltaRaw,
    prevState.lastResidualVecBase,
    tune.residualCorrectionGain,
    tune.residualCorrectionClampM,
    tune.residualEpsM
  )
  const eps = 1e-4
  const jointLocked = [lockedAxes[1], lockedAxes[2], lockedAxes[3], lockedAxes[4], lockedAxes[5]]
  const movableCount = jointLocked.filter((v) => !v).length
  if (movableCount === 0) {
    return { ok: false, error: "all_ik_joints_locked" }
  }

  const j: number[][] = [[], [], []]
  for (let col = 0; col < 5; col += 1) {
    if (jointLocked[col]) {
      j[0][col] = 0
      j[1][col] = 0
      j[2][col] = 0
      continue
    }
    const q2 = [...q]
    q2[col] += eps
    const fk2 = forwardKinematics(q2, config)
    const dp = vecScale(vecSub(fk2.position, fk.position), 1 / eps)
    j[0][col] = dp[0]
    j[1][col] = dp[1]
    j[2][col] = dp[2]
  }

  const rawJJt: Mat3 = [
    [
      j[0][0] * j[0][0] + j[0][1] * j[0][1] + j[0][2] * j[0][2] + j[0][3] * j[0][3] + j[0][4] * j[0][4],
      j[0][0] * j[1][0] + j[0][1] * j[1][1] + j[0][2] * j[1][2] + j[0][3] * j[1][3] + j[0][4] * j[1][4],
      j[0][0] * j[2][0] + j[0][1] * j[2][1] + j[0][2] * j[2][2] + j[0][3] * j[2][3] + j[0][4] * j[2][4],
    ],
    [
      j[1][0] * j[0][0] + j[1][1] * j[0][1] + j[1][2] * j[0][2] + j[1][3] * j[0][3] + j[1][4] * j[0][4],
      j[1][0] * j[1][0] + j[1][1] * j[1][1] + j[1][2] * j[1][2] + j[1][3] * j[1][3] + j[1][4] * j[1][4],
      j[1][0] * j[2][0] + j[1][1] * j[2][1] + j[1][2] * j[2][2] + j[1][3] * j[2][3] + j[1][4] * j[2][4],
    ],
    [
      j[2][0] * j[0][0] + j[2][1] * j[0][1] + j[2][2] * j[0][2] + j[2][3] * j[0][3] + j[2][4] * j[0][4],
      j[2][0] * j[1][0] + j[2][1] * j[1][1] + j[2][2] * j[1][2] + j[2][3] * j[1][3] + j[2][4] * j[1][4],
      j[2][0] * j[2][0] + j[2][1] * j[2][1] + j[2][2] * j[2][2] + j[2][3] * j[2][3] + j[2][4] * j[2][4],
    ],
  ]
  const conditionNumber = estimateConditionNumberFromJJt(rawJJt)
  const condOver = Math.max(0, conditionNumber - tune.conditionThreshold)
  const condRatio = tune.conditionThreshold > 0 ? clamp(condOver / tune.conditionThreshold, 0, 1) : 1
  const lambda = tune.baseDampingLambda + (tune.maxDampingLambda - tune.baseDampingLambda) * condRatio
  const singularScale = clamp(1 - condRatio, tune.singularStepScaleMin, 1)
  const targetDeltaSafe = vecScale(targetDelta, singularScale)
  const a: Mat3 = [
    [
      rawJJt[0][0] + lambda * lambda,
      j[0][0] * j[1][0] + j[0][1] * j[1][1] + j[0][2] * j[1][2] + j[0][3] * j[1][3] + j[0][4] * j[1][4],
      j[0][0] * j[2][0] + j[0][1] * j[2][1] + j[0][2] * j[2][2] + j[0][3] * j[2][3] + j[0][4] * j[2][4],
    ],
    [
      j[1][0] * j[0][0] + j[1][1] * j[0][1] + j[1][2] * j[0][2] + j[1][3] * j[0][3] + j[1][4] * j[0][4],
      rawJJt[1][1] + lambda * lambda,
      j[1][0] * j[2][0] + j[1][1] * j[2][1] + j[1][2] * j[2][2] + j[1][3] * j[2][3] + j[1][4] * j[2][4],
    ],
    [
      j[2][0] * j[0][0] + j[2][1] * j[0][1] + j[2][2] * j[0][2] + j[2][3] * j[0][3] + j[2][4] * j[0][4],
      j[2][0] * j[1][0] + j[2][1] * j[1][1] + j[2][2] * j[1][2] + j[2][3] * j[1][3] + j[2][4] * j[1][4],
      rawJJt[2][2] + lambda * lambda,
    ],
  ]

  const invA = invert3x3(a)
  if (!invA) {
    return { ok: false, error: "jacobian_singular" }
  }

  const y = matVecMul(invA, targetDeltaSafe)
  const dq = [0, 0, 0, 0, 0]
  for (let col = 0; col < 5; col += 1) {
    if (jointLocked[col]) {
      dq[col] = 0
      continue
    }
    const raw = j[0][col] * y[0] + j[1][col] * y[1] + j[2][col] * y[2]
    dq[col] = clamp(raw, -0.22, 0.22)
  }
  const hasSaturation = dq.some((v) => Math.abs(v) >= 0.219999)

  const qNext = q.map((v, idx) => v + dq[idx])
  const nextServoValues = { ...servoValues }
  const rawServoNext: number[] = []
  const quantizedServoNext: number[] = []
  for (let i = 0; i < 5; i += 1) {
    const raw = jointRadToServoDeg(qNext[i], i, config)
    rawServoNext[i] = raw
    const quantized = Math.round(raw)
    quantizedServoNext[i] = quantized
    nextServoValues[i + 1] = disableServoQuantization ? raw : quantized
  }
  // Keep gripper untouched by translational IK.
  nextServoValues[6] = servoValues[6]

  const cmdPos: Vec3 = vecAdd(fk.position, targetDeltaSafe)
  const nextPose = estimateTcpPoseFromServoValues(nextServoValues, config)
  const residual = computeConstraintResidual({
    frameType: frame,
    constraintType: constraint,
    constraintAxis,
    xCmdPosBase: cmdPos,
    xMeasPosBase: nextPose.position,
    tBaseTool: { position: nextPose.position, rotation: nextPose.rotation },
    tBaseUser: userFrame,
  })
  const logWarnings = [...residual.warnings]
  if (jogWarning) logWarnings.push(jogWarning)
  if (axisWarning) logWarnings.push(axisWarning)

  addConstraintLog({
    frameType: frame,
    constraintType: constraint,
    constraintAxis,
    axisVecOrNormalBase: residual.axisOrNormalBase,
    x_cmd: {
      mode: "pose",
      pos: cmdPos,
      quat: rotationMatrixToQuaternion(fk.rotation),
    },
    x_meas: {
      pos: nextPose.position,
      quat: rotationMatrixToQuaternion(nextPose.rotation),
      source: "estimated_fk",
      note: "x_meas derived from FK of computed next servo values",
    },
    residual: {
      active: residual.active_residual,
      plane_n_dot_dp: residual.plane_n_dot_dp,
      line_orthogonal_error: residual.line_orthogonal_error,
      point_position_error: residual.point_position_error,
    },
    T_base_tool: { position: nextPose.position, rotation: nextPose.rotation },
    T_base_user: userFrame,
    latency_ms: Date.now() - startedAt,
    warnings: logWarnings.length > 0 ? logWarnings : undefined,
    hard_projection: {
      v_raw: rawDelta,
      v_hard: targetDeltaRaw,
      residual_norm: vecNorm(vecSub(rawDelta, targetDeltaRaw)),
    },
  })

  const deltas = rawServoNext.map((v, i) => v - servoValues[i + 1])
  const qDeltas = quantizedServoNext.map((v, i) => v - servoValues[i + 1])
  const hadQuantizationLoss = deltas.some((v, i) => Math.abs(v) > 1e-9 && Math.abs(qDeltas[i]) === 0)
  const postQuantizationDeadband = qDeltas.every((v) => Math.abs(v) === 0) && deltas.some((v) => Math.abs(v) > 1e-9)
  const absD = deltas.map((v) => Math.abs(v))
  const minServoDeltaDeg = absD.length > 0 ? Math.min(...absD) : 0
  const maxServoDeltaDeg = absD.length > 0 ? Math.max(...absD) : 0

  return {
    ok: true,
    nextServoValues,
    warning: jogWarning ?? axisWarning,
    diagnostics: {
      requestedStepMm,
      hasSaturation,
      hadQuantizationLoss,
      postQuantizationDeadband,
      minServoDeltaDeg,
      maxServoDeltaDeg,
      transitionWeight,
      conditionNumber,
      appliedDampingLambda: lambda,
    },
    nextControllerState: {
      modeKey: currentModeKey,
      modeSinceMs,
      lastResidualVecBase: residual.deltaPosBase,
    },
  }
}
