# IX-InertiaSense

**An Apache-2.0, C++20 adaptive inertial-awareness research core for robotics.**

Author: Bryce Lovell. Independent research release v0.1.0 (October 2026).

## What really works in this release

- Estimates a payload's unknown **mass and three-dimensional center of mass relative to a wrist F/T sensor** from several different, tared, quasi-static gravity-wrench observations using Huber-weighted regression.
- **Abstains** if the gravity directions do not make the parameters observable, rather than producing an invented 3D CoM.
- Rejects flagged non-static, unhealthy, out-of-order and invalid measurements; tracks persistent payload changes and restarts identification.
- Implements a small artificial-vestibular IMU tilt filter, known-link CoM summation, support polygon signed margin and simple LIPM capture point.
- Exposes a small C++ library and CSV command-line interface, plus an independently calculated OLS comparison in Python.
- Tested typed HapticSight and SynapDrive input adapters against actual donor code (donor code NOT redistributed); assurance provenance bridge is schematic, not end-to-end assurance integration.

**Not yet delivered:** autonomous safe balance on real robots, motor-level control, contact-rich dynamics, full IMU navigation, unknown-inertia-tensor learning, broad cross-embodiment generalization, ROS 2 production middleware, or a genuine robotics scientific breakthrough. No physical robot was tested. The published benchmark uses a **synthetic quasi-static wrench model** with known gravity, known stationary state and pre-tared sensor inputs.

## Requirements

CMake 3.16+, C++20 compiler (GCC, Clang, MSVC), Python 3.11+ for tests and benchmarks (standard library only). The native library deliberately uses no C++ third-party dependencies. Additional optional donor integrations can import the original user projects.

## Build and run (Windows 11 / PowerShell)

```powershell
Expand-Archive .\IX-InertiaSense-v0.1.0.zip -DestinationPath .\IX-InertiaSense -Force
cd .\IX-InertiaSense\IX-InertiaSense-v0.1.0
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
python -m unittest discover -s tests -p "test_*.py" -v
python benchmarks/benchmark.py --binary build/Release/inertiasense.exe --repeats 40 --output results/benchmark_windows.json
```

On Linux/macOS:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
python -m unittest discover -s tests -p 'test_*.py' -v
python benchmarks/benchmark.py --binary build/inertiasense --repeats 40 --output results/benchmark_local.json
```

CSV CLI: `inertiasense fit path/to/wrench.csv` for a single static identification episode, or `inertiasense track path/to/wrench.csv` for change detection and re-identification. Columns:
`time,gx,gy,gz,fx,fy,fz,tx,ty,tz,quality,static,healthy`. The estimator operates in the sensor reference frame; static is an explicit flag that must come from validated motion/contact monitoring. Output is JSON. If insufficiently observable, the command exits with code 3 and returns a reason, rather than a guessed result.

## Benchmark outcomes

There are **240 synthetically generated independent payload episodes** in `results/benchmark.json` (2 arm-tilt configurations × 3 disturbance conditions × 40 trial seeds). The comparison is to **ordinary least squares with identical observation data**, not to a state-of-the-art robot estimator. In clean/noisy datasets both perform comparably. Robust estimation is better in injected impulse-outlier datasets; this expected benefit of Huber regression should **not** be described as a novel robotics breakthrough. Exact metrics and limitations: `docs/VALIDATION.md` and `results/benchmark.json`.

A separate 40-trial synthetic changing-payload benchmark in `results/change_benchmark.json` measures model invalidation and re-identification after switching payload mass and CoM. It is not a physical robot benchmark.

## Research and next milestones

Read `docs/MATHEMATICS.md` for equations and observability limits, `docs/RESEARCH_AND_POSITIONING.md` for the C++/Python choice and closest existing systems, and `docs/DONOR_AUDIT.md` for donor compatibility. A next release must add an actual physics environment, articulated robot models, independent strong baselines, dynamic contact and slip, and eventually hardware validation before claiming the general physical-adaptation problem is solved.

## Release integrity

Run `python scripts/verify_release.py` from the repository root to validate `MANIFEST.sha256`. This checks source/release integrity, not scientific truth.

## Responsible use

**NOT functional-safety-rated or suitable for autonomous hardware actuation.** An F/T sensor failure, a badly tared tool, excessive acceleration, changing contacts, a compromised clock or an unexpected geometry can make estimates unreliable. An independent safety-certified controller must govern real robots. No medical, defense, aerospace or factory safety clearance is implied by passing these tests.

## License

Apache License 2.0. Free to use, modify and distribute, including commercially, under its terms. Original donor projects remain separate; see `NOTICE` and donor audit.
