# Design decisions

Ten decisions that shaped the rover, each with why it was made and what was measured. The
numbers come from the linked docs; each entry says where.

## 1. The EKF runs on the Pi, not on the laptop

**Decision:** the EKF node runs on the truck's Orange Pi, next to the bridge, as its own Docker
service (same image, host network and IPC). Only `/odom` and TF cross Wi-Fi to the laptop.

**Why:** the filter's inputs come at 100 Hz, and Wi-Fi to the laptop showed gaps up to about
600 ms. A filter fed across that link would have to predict through the gaps or stall. On the
Pi the inputs arrive over shared memory; RViz on the laptop only needs the result.

**Source:** [ros-bridge.md, "EKF and pose"](ros-bridge.md#ekf-and-pose-group-6) ("Where").

## 2. The steering model is a soft measurement, and it's off while the truck is stopped

**Decision:** the bicycle model's yaw rate (k·speed) enters the EKF as a pseudo-measurement whose
noise grows with the turn, not as a hard constraint. Since 2026-10-25 it's skipped while the
encoder reads at rest (|v| < 5 mm/s).

**Why:** the gyro reports rotation plus bias, one number with two unknowns. The steering model
is a second opinion on rotation; when it says "about zero", the gyro's reading is bias. That's
what makes the bias observable. But "stopped means not rotating" is false when the truck is
turned by hand or set down facing elsewhere.

**Measured:** before the fix, a 90° hand turn at rest showed as 0.75 of the turn and then
decayed about 0.015 rad/s, as the bias absorbed the rotation. After it, the same turn read
1.539 rad against the gyro's 1.543 (−0.004 rad) and stayed flat.

**Source:** [kalman-filter.md, "The steering-model pseudo-measurement"](kalman-filter.md#the-steering-model-pseudo-measurement)
and ["Real-drive results"](kalman-filter.md#real-drive-results-week-6-2026-10-25).

## 3. Locked axles: the encoder wheel reads the axle centre

**Decision:** `rear_axle_locked = true` in `config/vehicle.cfg`. Both rear wheels are on one
shaft, so the encoder wheel's speed is the rear-axle centre's speed, not one wheel's in a
differential. The front axle and the centre are locked too (checked by hand 2026-10-25).

**Why:** with an open differential the inner and outer wheels turn at different speeds in a
corner, and one wheel's encoder would need a correction for the turn. Locked, there's nothing to
correct, but in tight turns all four wheels are forced to one speed while the front runs a
larger radius, so the encoder over-reads ground speed by a few percent. The Gazebo model drives
its wheels like an ideal differential, so the sim's EKF runs with the open formula.

**Source:** `config/vehicle.cfg` (rear_axle_locked); [kalman-filter.md, "Real-drive results" and
"Real vs sim"](kalman-filter.md#real-vs-sim-week-6-step-3).

## 4. Sensor stamps come from a fit of the board's clock

**Decision:** each frame is stamped with when the board made it, on the Pi's clock, from a
least-squares line through the earliest arrivals of each second (the last 20 s), not with when
the Pi read it.

**Why:** USB, the ST-LINK bridge and the reader only ever add delay, so the earliest arrival in
each second is the closest to the truth. The board's clock runs a few thousand ppm fast and
drifts with temperature; on the bench it climbed from +2800 to +4469 ppm over about 8 minutes.

**Measured:** in synthetic tests (2000–4000 ppm, jitter, 20–100 ms bursts, resets, a wrap, a
drift change, a 2 s stall) the fit gives drift within ±100 ppm and stamps within 2 ms.

**Source:** [ros-bridge.md, "Board clock fit"](ros-bridge.md#board-clock-fit-clock_fitpy).

## 5. Layered stop paths: each layer covers a failure the others can't see

**Decision:** stopping is not one mechanism. In order of distance from the motor:

| Layer | Trigger | Action | Source |
|---|---|---|---|
| Teleop deadman | no key or keepalive for 0.5 s / 2 s | speed 0 (latched) / ESTOP | [ros-bridge.md, "Teleop"](ros-bridge.md#teleop-group-4) |
| Bridge heartbeat | no `/cmd_vel` for 300 ms / 1 s | DRIVE 0 / ESTOP and disarm | [ros-bridge.md, "Driving"](ros-bridge.md#driving-safetypy) |
| Firmware command watchdog | no valid command for 500 ms | driver off, latched | [firmware/README.md](../firmware/README.md) |
| Steering timeout | no steering command for 500 ms | servo centred | [firmware/README.md](../firmware/README.md) |
| Hardware watchdog (IWDG) | main loop hung for 250 ms | MCU reset | [firmware/README.md](../firmware/README.md) |
| Battery cutoff | below 6.6 V for 3 s | motor locked out until reset | [firmware/README.md](../firmware/README.md) |
| Pi low-battery shutdown | pack flat at rest | the Pi shuts down | [ros-bridge.md, "Low-battery shutdown"](ros-bridge.md#low-battery-shutdown-group-5) |

**Why:** a Wi-Fi drop keeps the bridge sending DRIVEs, so the firmware's watchdog can't see it;
the bridge's heartbeat can. A hung bridge stops the DRIVEs; the firmware's watchdog catches that.
A hung firmware loop stops refreshing the IWDG. The bridge's stop paths are covered by node
tests and by planted bugs the tests must catch (#6); the firmware's by host tests and bench tests
(firmware/README.md).

## 6. A binary, CRC-checked link, and tests that are themselves tested

**Decision:** the Pi and the board talk over a binary protocol (COBS framing, CRC-16/CCITT-FALSE,
explicit arm), with the codec in pure C, host-tested with sanitizers and fuzzed. Mutation
testing plants one bug at a time and requires a test to fail.

**Why:** a test suite that passes proves little until it's shown to fail on the bugs that
matter. A mutant that survives is a missing test.

**Measured:** the link: 88 mutants, 88 killed, 0 survived, on 2026-10-25. The bridge: 97 mutants on 2026-10-22, 96 killed and
the one control (a comment-only change) survived, as it must.

**Source:** [firmware/README.md](../firmware/README.md) ("Mutation testing");
[ros-bridge.md, "Tests"](ros-bridge.md#tests).

## 7. A discovery anchor for same-host DDS tests

**Decision:** the node tests start an idle rclpy node in its own process (an "anchor") before
anything else.

**Why:** in WSL, two DDS participants on one host started seconds apart often never discover each
other unless a third is already running. The cause inside Fast DDS is unknown; the anchor is the
workaround.

**Measured:** without the anchor, a test whose operator started after a bridge subprocess failed
in 10 of 10 isolated runs on 2026-10-21; with it, 0 of 10.

**Source:** [ros-bridge.md, "Tests"](ros-bridge.md#tests).

## 8. Telemetry keys: integer nanoseconds, loud duplicates, idempotent re-import

**Decision:** in Postgres, each sample's time is `t_ns bigint`, integer nanoseconds since the
run's first `/odom` header stamp, and `(run_id, t_ns)` is the primary key. Two messages of one
stream with the same stamp fail the import. A run is identified by the sha256 of its recording,
and importing it again replaces it in one transaction.

**Why:** a double of epoch seconds keeps only about 0.2 µs, so two stamps could round to one key.
A duplicate stamp means the stamping or the recording is wrong, and dropping one silently would
hide it. Replacing on re-import means a fixed importer can refresh an old run, and a failed
import leaves the old one in place.

**Source:** [telemetry.md, "Schema"](telemetry.md#schema-toolstelemetry_schemasql).

## 9. The sim's steering had to be as fast as the real servo

**Decision:** Gazebo's AckermannSteering `steer_p_gain` is 10, not the default 1.0.

**Why:** at 1.0 the model's steering joints lagged by about 1 s, so the model drove straight for
about 5 cm while `/rover/steering` already said full lock. At 10 the lag is about 0.1 s, close to
the real servo.

**Measured:** the first sim half circle missed the end-point check by 4.4 cm (the truth turned
only 3.03 rad). After the change: heading +0.002 rad and end point 0.6 cm against ground truth.

**Source:** [kalman-filter.md, "Real vs sim", "First results"](kalman-filter.md#first-results-2026-10-25).

## 10. Failure controls: each new CI check is shown to fail before it's trusted

**Decision:** before a CI job was committed, a deliberately broken copy of the code was run
against it, to prove the job turns red.

**Why:** a job can pass for the wrong reason: tests skipped, a result ignored, or an exit code
swallowed. `colcon test`, for one, returns 0 on test failures; only `colcon test-result` fails.

**Measured (2026-10-25):** with an assertion in the EKF node's test broken,
`colcon test-result --verbose` exited 1. With telemetry re-import changed to insert without
deleting, the integration test failed and pytest exited 1; with the `UNIQUE` also removed, so
the duplicate run really landed, the same test failed on finding two runs.

**Source:** [ros-bridge.md, "Tests"](ros-bridge.md#tests); [telemetry.md, "Tests"](telemetry.md#tests).
