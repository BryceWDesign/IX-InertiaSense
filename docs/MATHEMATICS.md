# Core mathematics, assumptions and limitations

## Payload parameter identification (implemented)

For a calibrated wrist force-torque sensor under **static or quasi-static equilibrium**, with gravity `g` represented in sensor coordinates, an attached payload of unknown mass `m` and sensor-frame center of mass `r` produces

`F = m g` and `tau = (m r) cross g`.

The unknown vector `theta = [m, m*r_x, m*r_y, m*r_z]` enters linearly. Each gravity direction contributes six regression rows. The weighted least-squares estimator uses force rows with weight 1 and torque rows with weight 16 (a force-equivalent length scale of 0.25 m), followed by iterative Huber reweighting to resist outliers. The support across orientations determines a 4x4 information matrix, whose eigenvalue ratio is used to reject unobservable datasets. The optimizer refuses a physically negative mass or CoM outside the configured maximum payload extent.

**Important**: Under a single gravity direction, the CoM component parallel to gravity is not identifiable through the static torque. The library must refuse to guess it; this is a fundamental observability limit. Probe suggestion maximizes the smallest eigenvalue of the information matrix over supplied candidate gravity directions. The caller, not this function, must establish safe allowable probe motions.

Mass and CoM estimates are relative to the sensor only, and require a correctly *tared and tool-compensated* wrench. They do not automatically estimate distributed inertias, dynamic friction, actuator torques, robot posture, or unconstrained contact forces. Known gravity is assumed, and tests generating gravity vectors are illustrative synthetic episodes.

## Change-point detection (implemented)

After an estimate becomes observable, consecutive residuals exceeding a configured threshold trigger an invalidation and re-identification. Pending suspect samples are isolated before a change is confirmed. This is not a formally calibrated statistical change-point test; thresholds must be calibrated to sensor noise, payload class, and task. Invalid or untrusted data should be screened upstream. The tracker now rejects unhealthy, invalid, and out-of-order inputs prior to change detection. Do not equate a step change in contact force with a mass change.

## IMU orientation (implemented)

The unit quaternion integrates body-frame gyro angular velocity (`q_next = q * exp(omega*dt/2)`), and its tilt is nudged using measured specific force only when norm is near 9.81 m/s². Significant linear acceleration is rejected for gravity-based tilt correction. Yaw is not observable from gyro and gravity alone and will drift; this is a complementary tilt filter, **not** a 6-DoF localization system or precision inertial-navigation solution. Rate bias estimation, gravity misalignment, magnetometers and asynchronous timestamps are not implemented.

## Whole-body center of mass (implemented geometry only)

Given each link's mass and **world-frame** CoM position, `CoM = sum(m_i*r_i)/sum(m_i)`. The library does not solve forward kinematics from joint angles or parse URDF; external encoders and a kinematics library must supply actual link poses. A detected payload can be included using its location in world frame after an independently known sensor-to-world transform.

## Stability proxy (implemented)

For a flat horizontal convex CCW support polygon, the signed inward distance to its boundary estimates a quasi-static stability margin. A separate simple linear-inverted-pendulum capture-point estimate `x_cp=x_com + v_com * sqrt(z_com/g)` is provided. These are **not** equivalent to verified physical stability for accelerating, multi-contact, dynamically stepping robots, and must not actuate hardware without an independent real-time safety controller.

## What is deliberately not implemented

Full Featherstone-style dynamics, actuator-level control, contact-rich rigid-body simulation, IMU bias observer/EKF, 3D inertia-tensor learning, friction discovery, unrestricted online changes during impacts, hardware timing guarantees, ROS 2 runtime nodes, functional-safety certification, full robot-to-robot learning transfer, and proof of novel algorithmic superiority. They require real robot models, hardware logs, and independent measurement.
