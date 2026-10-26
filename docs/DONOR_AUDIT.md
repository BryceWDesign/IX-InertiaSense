# Donor audit and integration provenance

Input snapshots (provided by project owner, October 2026):

| Repository | Inspected modules | Used in this release | Not copied/claimed |
|---|---|---|---|
| IX-HapticSight | `src/ohip_interfaces/force_torque.py`, `signal_health.py`, `src/ohip/safety_gate.py` | Optional adapter `from_hapticsight`, tested against original typed `ForceTorqueSample` and health/freshness classes | Perception, consent, and real hardware execution gates |
| SynapDrive-AI | `synapdrive_ai/neuro/drift.py`, `uncertainty.py` | Optional adapter calls the original `FeatureDriftMonitor` when donor installed; tested against original module | EEG decoder, BCI runtime, neuroadaptive policy |
| IX-Autonomy-Assurance-Case-Runtime | `telemetry.py`, `telemetry_adapter.py`, assurance documentation | Small provenance-oriented telemetry contract and independent test evidence; no import dependency | No claim of full assurance-dossier integration or certified evidence chain |

**Integration status:** HapticSight and SynapDrive compatibility tests successfully ran against extracted original source snapshots; those snapshots are not redistributed in the new package. The assurance donor informed the audit structure and contract but its complete runtime is **not** integrated. This is selective integration, not a wholesale combination of three large projects.

The new repository's C++ code and Python adapters were newly authored; no donor source files have been copied. Donors remain independently licensed: HapticSight under MIT, others Apache-2.0. The new package is Apache-2.0. If future changes copy source code, preserve appropriate original copyright/license notices.

## Original input SHA-256 (source ZIP archives, not embedded in release)

- `IX-HapticSight-main(20261021-005543).zip`: `4423e55c5837bca07e225fc14be9e0b785770d86e9ec9cf8a642c18495e2dbea`
- `SynapDrive-AI-main(20261021-005543).zip`: `6d2c0c2cea909df20a410f2b996085f5b43794ba934b59a8377ebc1e8d8b64e6`
- `IX-Autonomy-Assurance-Case-Runtime-main(20261021-005543).zip`: `cf1b3aa6f0486e77f90c0161d0cca95d9b5c7dd422ce1a551b1f1f10475df8c7`
