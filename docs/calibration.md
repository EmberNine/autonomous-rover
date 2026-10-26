# Calibration guide

Every value in [`config/vehicle.cfg`](../config/vehicle.cfg) has been
measured on the truck in its **pre-electronics configuration**. These values
are not final: mass and center of mass will be re-measured once the
electronics are mounted, and `meters_per_encoder_tick` re-checked (see below).
This page explains how each value is measured. Units are meters and radians;
convert before writing them in (1 in = 0.0254 m, 1 mm = 0.001 m, radians = degrees × π / 180).

**General tips**

- Use calipers for anything small; a tape measure is not precise enough on a
  1/24-scale vehicle (a 2 mm error is ~1.5% of the wheelbase).
- Measure each thing 3 times, and use the average.
- Put the vehicle on a flat surface at its normal ride height, with the
  battery installed, before any measurement that depends on load.

## `wheelbase_meters`

Distance from the **center of the front axle to the center of the rear axle**.

1. Park the vehicle on a flat surface with the wheels pointing straight ahead.
2. With calipers, measure the distance from the front hub center to the rear
   hub center on one side (use the center of the axle end / hub cap).
3. Repeat on the other side. The two should match within ~1 mm; average them.

## `front_track_width_meters` and `rear_track_width_meters`

Distance between the **centers of the left and right tires**, front axle and
rear axle respectively. Measure and record **both** - do not assume they
match. On the SCX24 they don't: the front and rear axles can have genuinely
different track widths (front wider, in one measured unit).

1. Measure the outer edge to outer edge of the two tires on one axle, and the
   inner edge to inner edge.
2. Track width = (outer distance + inner distance) / 2 (this lands on the
   center of each tire's tread).
3. Repeat for the other axle using the same method. Cross-check each against
   a turning-circle measurement (drive full lock, measure the radius; a
   tighter measured turning circle than the bicycle-model prediction usually
   means the rear track width - which sets the bicycle-model reference
   point - is narrower than assumed).

## `tire_diameter_meters`

1. Measure the outer diameter of each tire with calipers at 3 different points
   around the circumference (foam/rubber tires are rarely perfectly round).
2. Average all the readings across all four tires.
3. Note that a loaded tire squishes slightly, so this number is only
   approximate; the encoder calibration below is the authoritative source for
   distance traveled.

## `max_steering_angle_radians`

The angle of the front wheels at full steering lock, one side. Two options:

**A. Direct measurement.** Command full lock, place the vehicle on a sheet of
paper, trace a line along the wheel's tread and another along the vehicle's
centerline, and measure the angle between them with a protractor. Convert to
radians. Because the real front wheels don't turn identically (Ackermann
geometry), measure both and use the average.

**B. From the turning circle (more faithful to the model).**

1. Command full lock and a slow constant speed; let the vehicle drive a full
   circle with a marker or chalk dot on the rear axle center's path (or trace
   the rear-axle path with tape marks every ~10 cm).
2. Measure the radius `R` of the circle traced by the **rear axle center**.
3. `max_steering_angle_radians = atan(wheelbase_meters / R)`.

The 2026-10-22 measurement used a variant that needs no centre point: the
truck starts perpendicular to a tape line with both rear tires on it and
drives a half circle at full lock; mark both rear tire positions at the start
and at the end. Outer-to-outer = 2R + rear track, inner-to-inner =
2R - rear track, so their average is 2R at the rear-axle centre, and their
difference (the rear track) checks the marks. Three runs per side; the runs
are in `config/vehicle.cfg`.

Check both ways if you can; they should agree within a couple of degrees. Do
left and right lock separately - they may not be symmetric. Use the smaller of
the two.

### 2026-10-25 drive check: run-to-run spread ~7 %

The Week 6 half circle (full left, 0.05 m/s, EKF on) gave an effective rear-axle
radius of **0.595 m** (δ ≈ 0.220 rad), from the rear-hub chord average 1.186 m.
An earlier run the same day gave 0.64 m (δ ≈ 0.204). Observed so far:
**0.595-0.670 m**, −7.5 % to +4.2 % of the model's 0.643 m. The same command
turns about **7 %** tighter or wider from run to run, in line with the
2026-10-22 spread (0.185-0.221 rad). `max_steering_angle_radians` stays **0.204**: one run
is not a new mean, and any tolerance on a full-lock turn has to allow the spread
(the real-vs-sim check uses ±10 %, docs/kalman-filter.md).

## `meters_per_encoder_tick`

How far the vehicle moves for each encoder tick. **This cannot be measured by
turning the wheel by hand** - the SCX24's worm-drive axle won't back-drive
freely, so it must be measured with the vehicle under its own power.

**Current value: `3.9336e-5` m/tick (MEASURED 2026-09-29), avg 25422 counts/m.**

### 2026-09-29 measurement

AS5600 on the left rear wheel, 4096 counts/rev. Method actually used (simpler
than the full procedure below):

- **1.000 m** taped course, rear wheel lined up at both the start and finish
  lines, gentle throttle.
- Run 1 excluded: the vehicle was nudged past the line.
- Runs 2 and 3 kept: **25430** and **25414** counts (0.06% apart), avg
  **25422 counts/m** -> `1 / 25422 = 3.9336e-5` m/tick.
- Calculated from the unloaded `tire_diameter_meters`,
  `pi * 0.054 / 4096 = 4.1417e-5`, which is 5.3% too large; the effective
  rolling diameter under load is 51.3 mm (soft tire squash).

### 2026-10-25 check with the EKF: confirmed, unchanged

Week 6 straight, 2 m at 0.10 m/s, rear-axle centre marked on the tape:
tape **2.073 m**, EKF (√(x² + y²) from `/odom`) **2.041 m**, **−1.5 %**.
That is within the 3 % pass of the bench (docs/ros-bridge.md), so `3.9336e-5`
stays.

### Re-check procedure (after the electronics are installed)

The procedure below is more rigorous and is the planned re-check once the
electronics are mounted: the extra weight squashes the tires more, which
shrinks the rolling diameter and changes this value.

1. Lay out a straight, flat, **3.000 m** course with masking tape: a start line
   and a finish line, measured with a tape measure (tape one edge, measure
   edge to edge, and mark it clearly). Use a rear-axle-center reference: line up
   the rear axle with the start line, and stop when the rear axle reaches the
   finish line.
2. Read the encoder count with the vehicle stationary at the start (`ticks_start`).
3. Drive it straight at slow, steady speed to the finish line under power,
   and stop there with the rear axle on the line.
4. Read the count again (`ticks_end`).
5. `meters_per_encoder_tick = 3.0 / (ticks_end - ticks_start)`.
6. Repeat at least 5 times, in both directions if the vehicle can reverse
   cleanly. Discard obvious outliers (wheel slip, veered off the line).
   Average the rest. The spread between runs tells you how much to trust it;
   if runs differ by more than ~2%, look for wheel slip or a loose encoder
   mount before proceeding.
7. Recalibrate whenever tires are changed or worn noticeably, or the
   encoder is remounted.

### Open item: Gazebo wheel size

The Gazebo model (`rover.urdf.xacro`) still sizes its wheels from the unloaded
`tire_diameter_meters` (0.054 m), not the 51.3 mm effective rolling diameter.
Simulated distance per wheel revolution is therefore ~5% longer than on the
real truck. It didn't affect the Week 6 real-vs-sim check: `sim_adapter` converts
the sim wheel's angle with the same URDF radius (`tire_diameter_meters / 2`), so
the sim's EKF sees the sim's true speed, and the sim straight came out at 0.00 %
against ground truth (docs/kalman-filter.md, "Real vs sim"). Still open if the sim
is ever compared with the truck revolution for revolution.

## After updating the file

Run the test suite (`ctest --test-dir build --output-on-failure`); one test
loads `config/vehicle.cfg` and checks it still parses, and asserts the
current `meters_per_encoder_tick` exactly. If you change that value (e.g.
after the post-electronics re-check), update the assertion in
`tests/test_vehicle_config.cpp` at the same time.
