# Reproducibility and acceptance criteria

## Current executed verification

- CMake GNU C++20 Release build and CTest: `ix_core_tests` green; source-level core test groups cover quaternion mathematics, gyro/accelerometer fusion, CoM geometry, planar stability, gravity observability, robust outliers, invalid inputs, payload changes, isolated impulses and failure conditions.
- Python `unittest` conformance checks: original donor smoke checks only when donor source trees are provided, alongside metadata/provenance and OLS baseline checks.
- Synthetic 240-trial experiment: 40 deterministic seeds × 2 arm tilt configurations × 3 sensor conditions, 80 independent static wrench observations per trial. The same observations are given to robust IX estimator and an independent ordinary-least-squares baseline. All input orientations and truth values are programmatically generated; neither side is tuned on the held-out trial labels.
- Each experiment operates with known gravity vector, known static-condition label, and pre-tared sensor wrench; these are significant oracle assumptions, not demonstrated capabilities in live hardware.

- Synthetic change benchmark: 20 trial seeds × 2 arm configurations; an unannounced payload change at observation 90 out of 180, re-identification after sustained residuals. Results saved under `results/change_benchmark.json`. The tracker sees generated wrench measurements, not real sensor data.
- The MuJoCo physics package was unavailable in the execution environment; attempts to install it could not reach a package index. No MuJoCo simulation or hardware validation has been performed.

## What passing means

Pass: source compiles, unit tests hold under deterministic counterexamples, basic donor interfaces interoperate, and performance figures are reproducible for the synthetic model. None of these proves performance on a physical robot or general superiority over existing adaptive estimators.

## Stronger future targets (NOT YET SATISFIED)

- Accurate unknown payload + 3D CoM estimates on two physically distinct robots, under calibrated and uncalibrated conditions; report per-axis error, confidence, speed, abstention, fault rates.
- Remain stable under unknown friction, slip, dynamic motions, biases, shocks, and uncertain contact state.
- Cross-embodiment transfer without hand-tuned per-robot calibration beyond accessible URDF/geometry and sensor metadata.
- Head-to-head against modern published estimators, not only an OLS reference.
- Runtime and deadline quantiles measured in a native robot stack.

See `results/benchmark.json` for exact numbers; do not substitute rounded marketing figures.
