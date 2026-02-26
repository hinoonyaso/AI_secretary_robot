# Constraint Tuning Parameters

`stepConstraintMotion` supports runtime tuning via `tuning`:

- `residualEpsM`:
  - residual dead-zone (m). Below this, correction is not applied.
- `residualCorrectionGain`:
  - gain for residual back-correction term.
- `residualCorrectionClampM`:
  - max correction magnitude per step (m).
- `maxStepMm`:
  - command step upper bound (mm).
- `transitionTauMs`:
  - mode transition time constant for ease-in (`free -> plane -> line`).
- `baseDampingLambda`:
  - nominal LM damping.
- `maxDampingLambda`:
  - max damping in near-singular conditions.
- `conditionThreshold`:
  - condition-number threshold where damping/step scaling starts.
- `singularStepScaleMin`:
  - minimum step scale when condition is high.

## Notes
- Transition uses exponential ease-in:
  - `w = 1 - exp(-dt / transitionTauMs)`
- Residual correction is applied on violating component only:
  - line: orthogonal leakage component
  - plane: normal leakage component
- Near singularity:
  - increase damping (`lambda`)
  - reduce effective step scale

