# Toolchain decision and closest published work

## Language choice

**C++20 core, Python 3.11+ experimental tools.** C++ exposes deterministic memory ownership and a native ABI path to `rclcpp`/ROS 2, sensor drivers and robot controller pipelines, without requiring Python in a control path. Python standard library generates deterministic held-out trials and handles human-readable reports. This package makes **no measured hard real-time guarantee** and is not yet a ROS 2 node.

C# is useful for desktop tools but less directly supported in common real-time ROS 2 and robot dynamics tooling. Julia is useful for optimization research, but not the most portable robotics middleware default. Rust may offer memory-safety advantages, but current industry robotics ecosystem and native physics integration strongly favor C++ for core robotics; Rust may be considered for future isolated services.

The v0.1.0 core intentionally avoids requiring Eigen/Pinocchio at compile time, to make a source build on Windows and Linux straightforward. For production work, prefer model-based kinematics/dynamics through established **Pinocchio (C++ and Python bindings)**, and use a recognized physics simulator such as **MuJoCo**, rather than extending hand-coded toy kinematics into a general physics engine.

## Research sources (accessed October 2026)

- Pinocchio official documentation (modern articulated rigid-body dynamics and analytic derivatives): https://gepetto.github.io/doc/pinocchio/doxygen-html/
- Pinocchio GitHub: https://github.com/stack-of-tasks/pinocchio
- Google DeepMind MuJoCo Python API: https://mujoco.readthedocs.io/en/latest/python.html
- ROS 2 rclcpp/rclpy core APIs: https://docs.ros.org/en/rolling/Concepts/Basic/About-Client-Libraries.html
- System Identification of Robots in Contact, ICRA 2025 source and citation: https://github.com/ShahramKhorshidi/system_identification
- ROS 2 control real-time tooling: https://control.ros.org/kilted/doc/realtime_tools/doc/migration.html

## Novelty assessment

Static wrench-based inertial parameter identification, robust regression, excitation/observability testing, sensor-quality gating, and change detection are established robotics concepts. This release should **not be advertised as a new general solution to physical adaptation**. It is a focused, reusable, audited integration, suitable for expanded evaluation and reproducible engineering work. Novelty would require an independently confirmed advantage over strong methods on unseen real-world morphology and dynamics tasks, potentially requiring new active self-identification algorithms and validated embodiment-invariant representations.

## Next validation gates

1. Simulated force-torque and IMU with actual time stepping, including varied contact configurations, tool tare errors and compliant objects (MuJoCo / alternative validated dynamics engine).
2. Robot body models through URDF plus Pinocchio, validated against independent known inertial parameters.
3. Quantitative comparison to state-of-the-art online adaptive estimators, robust least squares, and learned estimators under identical sensors, signals, compute budgets.
4. Full hardware characterization with safe force limits, emergency stop, sensors calibrated to known test masses. No public claims of improved robot safety until then.
5. Real out-of-distribution validation: variable gravity direction, slip/contact impulses, poor excitation, unmodeled accelerations, changing mass and kinematics, heterogeneous physical robot bodies.
