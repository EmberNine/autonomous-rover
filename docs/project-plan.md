# Project plan

This document is the source of truth for scope and schedule. If the plan
changes, change it here. Each week has the plan as written at the start and
what was actually done, from the commits and the docs. The open work is in the
[Backlog](#backlog) at the end.

## Week 1 — C++ basics, project scaffold, bicycle motion model

**Plan:** the scaffold and a kinematic bicycle model.

**Actual (2026-09-21 to 09-22):** the `rover_core` library (bicycle model, a
strict `vehicle.cfg` parser), GoogleTest suites, CI, and `vehicle.cfg` filled
from caliper and turning-circle measurements (docs/calibration.md). The URDF
model and the first Gazebo drive came the same days (Week 2's).

## Week 2 — Simulation and chassis CAD

**Plan:** the rover modeled and driving in Gazebo; start chassis CAD for the
electronics deck and mounts.

**Actual:** `rover_description` (xacro model, RViz view) and `rover_gazebo`
(spawn and drive in Gazebo Harmonic), 2026-09-22. Chassis CAD isn't in this
repo; the electronics went on a soldered protoboard carrier
(firmware/README.md, "Carrier hardware").

## Week 3 — Kalman filter

**Plan:** the Kalman filter (IMU + encoder fusion), ROS-free C++ with Eigen;
print first parts.

**Actual (2026-09-27):** a 6-state EKF (encoder, gyro, and the steering model
as a pseudo-measurement) with Eigen, and system tests: NEES/NIS consistency, a
realistic drive against dead reckoning, tire scrub (docs/kalman-filter.md).
Printed parts aren't in this repo.

## Week 4 — Microcontroller firmware

**Plan:** motor, steering servo, one drive encoder, IMU, wheel-speed control,
and a framed, checksummed serial protocol.

**Actual (2026-09-29 to 10-16), NUCLEO-L432KC:** the AS5600 encoder at 1 kHz,
the ICM-42688-P IMU, the motor (TB6612) with ramp, latch and command watchdog,
the steering servo, a hardware watchdog (IWDG), the battery monitor with its
cutoff, closed-loop wheel speed (a PI loop every 10 ms) with stall protection,
and the Pi link: a binary protocol with COBS framing and CRC-16, host-tested and
mutation-tested (firmware/README.md).

## Week 5 — Pi integration

**Plan:** the serial bridge, calibration (ticks per meter, steering angle, gyro
bias), teleop, and the live pose in RViz. (Planned on a Raspberry Pi; built on
an Orange Pi.)

**Actual (2026-10-16 to 10-22):** `rover_bridge` in Docker on the Pi, in six
groups: the read side, the board-clock fit and `/diagnostics`, driving with its
stop paths, keyboard teleop with a deadman, the low-battery shutdown, and the EKF
node with the pose in RViz (docs/ros-bridge.md). Calibration: meters per tick
(2026-09-29) and the steering angle (2026-10-22) in docs/calibration.md; the gyro
bias is learned by the EKF from its first 2 s at rest.

## Week 6 — Real-vs-simulated accuracy benchmark

**Plan:** a taped closed-loop course; return-to-start error over several laps.

**Actual (2026-10-25):** two teleop routes on the truck, taped against the EKF:
a 2 m straight and a full-lock half circle; then the same routes scripted in
Gazebo against the physics engine's ground truth (docs/kalman-filter.md,
"Real-drive results" and "Real vs sim"). Closed laps need the truck to drive a
route by itself, which isn't built (Backlog).

## Week 7 — Automated regression suite

**Plan:** straight line, fixed-steering circle, S-curve, U-turn and obstacle
stop, run headless in Gazebo in CI.

**Actual (2026-10-25):** the ROS-dependent tests in CI instead: job `ros-tests`
builds the ROS packages in a `ros:jazzy` container and runs the bridge's node
tests and the EKF node's test (docs/ros-bridge.md, "Tests"). The Gazebo route
suite isn't built (Backlog).

## Week 8 — Telemetry

**Plan:** Postgres + Streamlit dashboard.

**Actual (2026-10-25):** runs recorded as ros2 bags, imported into Postgres,
one Streamlit page per run, and the CI job `telemetry-db` against a Postgres 16
service (docs/telemetry.md).

## Week 9 — Documentation

**Plan:** README, architecture diagram, and a 60–90 s demo video.

**Actual:** the README with the architecture diagram, docs/design-decisions.md,
and these docs brought up to date. No demo video yet.

## Week 10 — Buffer

## Backlog

Open work, roughly in order of value. Each item points to where it's described.

1. **Autonomous route following.** The truck drives only by teleop today; there
   is no planner or path follower. Closed-loop laps (the Week 6 plan) need it.
2. **Heading follows the steering model in long turns** (docs/kalman-filter.md,
   "Backlog", item 1): −0.072 rad over the Week 6 half circle, growing with the
   turn's length.
3. **External ground truth**, e.g. an overhead ArUco camera: today truth is a
   tape measure at the end of a run, and nothing measures x/y during it.
4. **Headless Gazebo route check in CI** (`tools/sim_route.py straight`). The
   cost, estimated 2026-10-25 and not measured: 2–4 h of work; about 6–10 min of
   CI per run, most of it installing ros_gz (~1 GB); flakiness medium to high
   (the sim's real-time factor on a shared runner, start-up order, tolerances
   tuned on the desktop). Deferred for that cost.
5. **Run-to-run steering spread, about 7 %** at full lock (docs/calibration.md,
   "2026-10-25 drive check"): any full-lock tolerance has to allow for it.
6. **Telemetry (Week 8) follow-ups** (docs/telemetry.md):
   - the sim's `/odometry/ground_truth` is recorded in the bag but not imported,
     so the page compares the EKFs, not the sim's truth;
   - the gyro heading comes from `tools/gyro_turn_check.py`, run next to the
     recording, not from the recording itself (`/rover/imu/raw` isn't recorded);
   - a split bag (more than one data file) is refused, not imported.
7. Smaller items: `tools/gyro_turn_check.py`'s traceback on Ctrl-C
   (docs/kalman-filter.md, "Backlog", item 2); the Gazebo wheel size
   (docs/calibration.md, "Open item"); the 10 s-line test that can flake
   (docs/ros-bridge.md, "Open items (tests)").
