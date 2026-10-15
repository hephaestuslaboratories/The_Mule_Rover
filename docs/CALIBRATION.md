# Calibration

How every measured constant in this stack was obtained, what tool produced it,
where it lives, and what would make it stale. The guiding rule throughout was
*measure with a bag, change one thing, measure again*; nothing below was set
by feel.

Contents

1. [Locked constants and where they live](#1-locked-constants-and-where-they-live)
2. [Drivetrain geometry and encoder decode](#2-drivetrain-geometry-and-encoder-decode)
3. [Motor wiring: the single-wheel isolation test](#3-motor-wiring-the-single-wheel-isolation-test)
4. [Straight-line trim](#4-straight-line-trim)
5. [Turn calibration](#5-turn-calibration)
6. [IMU conventions and fusion gates](#6-imu-conventions-and-fusion-gates)
7. [LiDAR extrinsics](#7-lidar-extrinsics)
8. [Footprint and inflation](#8-footprint-and-inflation)
9. [ToF belt geometry](#9-tof-belt-geometry)
10. [Nav2 tuning by bag A/B](#10-nav2-tuning-by-bag-ab)
11. [Firmware control constants](#11-firmware-control-constants)
12. [When to re-calibrate](#12-when-to-re-calibrate)

---

## 1. Locked constants and where they live

Change any of these only with a bag that proves the change.

| Constant | Value | Lives in | Status |
|---|---|---|---|
| `WHEEL_DIAMETER_M` / `wheel_diameter_m` | 0.120 | firmware **and** launch | measured wheel |
| `WHEEL_BASE_M` / `wheel_base_m` | 0.37 | firmware **and** launch | tape, centre-to-centre |
| `COUNTS_PER_REV` / `counts_per_rev` | 267 | firmware **and** launch | 2× decode, do not change decode |
| `PIN_IN1` / `PIN_IN2` | 5 / 4 | firmware | matches physically crossed wires |
| encoder ISR signs | left `a==b → −−`, right `a==b → ++` | firmware | |
| `LEFT_WHEEL_TRIM` / `RIGHT_WHEEL_TRIM` | 1.00 / 0.969 | firmware | from straight-line test |
| `yaw_sign` | −1.0 | bridge default + launch | |
| `cmd_w_scale` | 0.85 | bridge default + launch | verified over 1729 samples |
| `spin_yaw_scale` | 1.0 | bridge default + launch | was 0.73, retired after wiring fix |
| `rev_w_trim` / `fwd_w_trim` | −0.015 / 0.0 | bridge default + launch | |
| `vyaw_fusion_gate` | 0.08 rad/s | launch | from bag odom-step analysis |
| `cmd_timeout_s` | 0.8 s | launch | |
| `lidar_x`, `lidar_y`, `lidar_yaw` | −0.335, 0.0127, −0.0202 | URDF | x verified to 0.4 mm; yaw provisional |
| footprint | `[[0.24,0.2025],[0.24,−0.2025],[−0.44,−0.2025],[−0.44,0.2025]]` | nav2_params (both costmaps) | derived from wheel span |
| `inflation_radius` / `cost_scaling_factor` | 0.50 / 2.0 | nav2_params, both costmaps **and** RPP's `cost_scaling_dist` / `inflation_cost_scaling_factor` | A/B'd |
| ToF caps | 0.47 front/rear, 0.55 corners | tof_belt.yaml | floor test |
| down-looker tilt / baseline / threshold | 43° / 0.53 m / 0.52 m | URDF + tof_belt.yaml | derived from its own reading |
| velocity_smoother accel | ±0.8 m/s² | nav2_params | matches firmware ramp |

The firmware prints `WHEEL_DIAMETER_M`, `WHEEL_BASE_M`, `MAX_SPEED_MPS` and
`WATCHDOG_MS` in its boot banner, which the bridge forwards to
`/rover/link_status`. Check them against the launch after any reflash.

Every parameter has to exist in **both** the node's declared default and the
launch dictionary. Running nodes have answered `ros2 param get` with stale
values four times; when anything smells stale, `pkill -9 -f <node>`, verify
with `ps`, restart the ROS daemon, and bump the firmware banner version on
every flash so you can prove which build is running.

## 2. Drivetrain geometry and encoder decode

**Wheels.** 3601 Series Rhino, 120 mm diameter. Track (drive-wheel
centre-to-centre) 0.37 m by tape. Both numbers were changed from the previous
chassis (95 mm / 0.34 m) on 2026-05-22 and are deliberately duplicated in the
firmware and the bridge launch parameters; the firmware uses them for its
wheel-speed PID and the bridge for odometry, and if they diverge odometry
drifts systematically while the PID still tracks.

**Encoder decode.** The firmware attaches a `CHANGE` interrupt to channel A
only and reads channel B to get direction. That is a 2× decode, and 267 is the
counts-per-revolution *under that decode*. Switching to 4× quadrature would
double the counts and silently halve the bridge's distance estimate. The two
ISRs count in opposite senses (`left: a==b → −−`, `right: a==b → ++`) because
the motors face each other across the chassis.

**Frames.** The raw `/wheel/left_velocity` and `/wheel/right_velocity`
topics are in the firmware's frame, where positive means "the wheel turns the
way the motor calls forward", which is the LiDAR end. The bridge's
`invert_linear: true` negates `linear.x` symmetrically on the outgoing `CMD`
and on the published `/wheel/odom_twist` so that +x is the drive-wheel end
(casters trailing) and command and odometry stay in one frame. Yaw is
invariant under that 180° flip, so ω is untouched. The calibration scripts
negate the raw wheel topics (`body = −raw`) for the same reason, and the yaw
differential `(vL − vR)/base` is what matches the body yaw rate in that
mirrored frame.

**Distance check.** `calibration/analyze_straight_bag.py --tape d1,d2,…`
divides the encoder-integrated distance by tape-measured ground truth for each
run. Within 2 % the wheel diameter is left alone; beyond that it prints the
corrected effective diameter `0.120 / ratio`, to be applied in both places.
Current calibration is inside 2 %.

## 3. Motor wiring: the single-wheel isolation test

For months spins ran roughly 12× too fast and a `spin_yaw_scale` of 0.73 and a
pre-warp lookup table had been bolted on to compensate. The tool that found the
real cause is `calibration/wheel_profile.py`. You command a single wheel
(`/cmd_vel` with `v = ±w·b/2` so the target for one side is zero) and record
`/cmd_vel`, `/wheel/left_velocity`, `/wheel/right_velocity`. For each burst
it prints the second-by-second measured velocity of both wheels in body frame
next to the per-wheel targets, then two verdicts: did the *expected* wheel
move or its mirror, and was it a clean single-wheel move (ratio of the
quieter wheel to the louder under 0.3) or were both loops fighting.

The answer was "mirrored": the MDDS30 direction wires are physically crossed,
so GPIO 4 lands on IN2 and GPIO 5 on IN1. Straight driving had always worked
because both signs were equal; spins reversed and the PID fought itself.
`PIN_IN1 = 5`, `PIN_IN2 = 4` in the firmware now encode reality, the yaw
fudge and the pre-warp table are gone, `spin_yaw_scale` returned to 1.0, and
the ESP32 pin table in the README is the one to trust.

## 4. Straight-line trim

Procedure in [`calibration/STRAIGHT_LINE_TEST.md`](../rover_bringup_extended_slam/calibration/STRAIGHT_LINE_TEST.md).

**Pre-check (do not skip).** Everything assumes REP-103, counter-clockwise =
positive yaw. With the bringup running, `ros2 topic echo /imu/data --field
angular_velocity` and rotate the rover CCW by hand: `z` must go positive. If
it does not, set `axis_z_sgn: -1` in the bridge before anything else, because
every trim recommendation would otherwise come out backwards.

**Acquire.** `bash calibration/straight_test.sh 0.15 5 3` records a bag and
walks you through three forward and three backward 5 s bursts at 0.15 m/s on
one floor surface (calibration is per surface; 0.10 m/s is below the stiction
band and lurches). Mark the start with tape and measure each run's travel to
the drive axle.

**Analyse.** `analyze_straight_bag.py <bag> --current-right-trim <fw value>
--tape …`. For each burst it integrates `/wheel/odom_twist` for signed
distance, unwraps the IMU quaternion yaw for drift, and forms curvature
κ = Δyaw / distance. Forward and backward runs give the *same sign* of κ for a
genuine wheel asymmetry, which is what lets them be averaged (distance-
weighted) and what distinguishes a trim problem from a dragging caster, which
flips sign with direction. Differential-drive algebra gives the asymmetry
ε = κ · b and the new trim `RIGHT_WHEEL_TRIM × (1 − ε)`. Positive κ means the
rover curves left, so the right side is effectively faster and its trim goes
down.

`yaw_profile.py` prints the same runs second by second and splits yaw into a
cruise part and a stop-window part. Steady growth during cruise is real
asymmetry; a jump in the last second is stop behaviour, which the firmware's
symmetric slew ramp was added to fix, and must not be trimmed out.
`analyze_straight_bag_no_cmd.py` is a fallback for bags without `/cmd_vel`
that detects runs from measured speed and excludes anything with |κ| over
0.3 rad/m as a deliberate turn.

**Result.** `RIGHT_WHEEL_TRIM = 0.969`, applied to the right wheel's velocity
*setpoint* in the firmware so the inner PID holds it (a post-PID PWM offset
would be erased by the integral, the lesson from the retired yaw assist).
Verification criterion is |drift| under ~1.5° over 0.75 m both ways; it
converges in one or two iterations.

**Open.** Offline analysis of the gyro yaw rate while commanded dead straight
(with a 3σ significance test) came out positive (left) on three
consecutive nav runs (+0.0126, +0.0230, +0.0227 rad/s) but no single run
reached significance because the test route turns too much. One long straight
corridor would settle it.

## 5. Turn calibration

`calibration/analyze_turn_bag.py <bag> [--meas d1,d2,…]`. Record bursts of
in-place spins and arcs (`/cmd_vel`, `/imu/data`, `/wheel/odom_twist`, both
wheel velocities). For the steady-state window of each burst (after a 1.5 s
ramp skip) it compares three yaw rates: commanded `w_cmd`, IMU `w_imu` (quaternion
delta over time, with the mean gyro printed alongside), and the raw wheel
differential `w_enc = (vL − vR)/b`. Two ratios fall out:

- **execution ratio `w_imu / w_cmd`**: does the rover do what it is told? A
  constant offset across rates means the effective wheelbase under skid
  differs from the geometric one; a rate-dependent one (bad at low ω) is
  stiction and points at `TURN_BIAS_PWM`.
- **odometry ratio `w_imu / w_enc`**: what the bridge's `spin_yaw_scale`
  should be, equivalently geometric/effective wheelbase. Encoders over-report
  yaw when the wheels scrub.

`--meas` takes physically measured total rotations per run (protractor or
floor marks) so the IMU itself can be validated: within 3 % the IMU yaw is a
valid ground truth for everything else. Repeats are grouped by rounded
(v, w) and averaged; arcs also print commanded vs achieved radius.

**What the data established.**

- A commanded 180° spin landed at 180°; the yaw estimate is honest. An earlier
  "2.1 s for 90°" stopwatch reading was a misread.
- Offline analysis of 1729 turning samples from navigation bags gave
  gyro/`cmd_vel` = 1.04× and gyro/`cmd_sent` = 1.22×. The drivetrain
  over-rotates by 22 % relative to what is written to the wire, and
  `cmd_w_scale: 0.85` cancels it almost exactly. It is doing real work; do not
  "fix" it.
- `yaw_sign: −1.0` reconciles the firmware's angular convention with REP-103
  on the ROS side.
- `rev_w_trim: −0.015` is a small yaw bias applied only when reversing nearly
  straight (|ω| under 0.05, |v| over 0.05).
- The drivetrain floor: spins need at least ~0.3 rad/s commanded, which is why
  behavior_server's rotational velocities are 0.30 and the recovery speeds in
  the BT sit above it.

**Still open: the left/right asymmetry test.** The rover historically wedged
almost only on right turns in tight spots. Every Nav2 parameter is
left/right symmetric, so only two directional things can explain it: the
provisional `lidar_yaw` and the wheel trims. The decisive test is a bag of
equal-magnitude left and right spins analysed with `--meas`; if right turns
come out sharper the drivetrain is at fault and no Nav2 tuning will fix it.
Do not change the firmware trims before that bag exists.

## 6. IMU conventions and fusion gates

- The BNO085 is read in UART mode at 100 Hz. Its native gyro reports return
  zeros on this chip over every transport tried, so angular velocity is
  derived in firmware from the quaternion delta. At 100 Hz that derivative is
  about twice as noisy per sample as at 50 Hz; `imu_gyro_cov` (3e-4) in the
  bridge is the knob if the EKF looks jittery.
- Axis remap defaults to identity (chip +X = rover +X, labelled side up). Only
  the integer sign parameters may be set from the launch file; the letter
  parameters must stay as node defaults because of YAML's `y → true`
  coercion.
- `imu0_relative: true` in the EKF because there is no magnetometer, so the
  heading is only meaningful relative to start.
- `imu0_remove_gravitational_acceleration: false`: the firmware requests the
  `LINEAR_ACCELERATION` report, which the chip has already de-gravitied.
  Removing it again injected a phantom 9.81 m/s² (verified 2026-07-16).
  Accelerations are not fused at all at present.
- ZUPT thresholds (`vx` 0.03 m/s, `ωz` 0.05 rad/s, 0.2 s delay, 1e-4
  covariance) were chosen so the filter engages only when both wheels and
  gyro agree the rover is parked. It is disabled in nav mode because the
  drivetrain's pulsed low-speed turns dip under the gyro threshold.
- `vyaw_fusion_gate: 0.08 rad/s`. The EKF config's own note says skid-steer
  wheels slip above ~0.06 rad/s, but the gate sat at 0.15 while a typical nav
  steering command was 0.138, so slipping wheel yaw was trusted through the
  whole normal band (one bag showed wheel +0.42 vs gyro +2.12 rad/s at the
  same instant). Lowering the gate to 0.08 took the largest odom→base_link
  step from 0.350 m to 0.016 m and shrank slam_toolbox's corrections.

## 7. LiDAR extrinsics

`tools/lidar_cal.py --true-distance <m>`. Park the rover square to a flat
wall, tape-measure wall to drive-axle centre. The script reads the current
`base_link → laser_frame` transform from TF once, converts each
`/scan_filtered` into `base_link`, keeps the forward strip (x > 0.05,
|y| ≤ 0.35 m), fits a line by PCA (the eigenvector of the smallest covariance
eigenvalue is the wall normal), and over up to 30 scans reports mean and SD of
perpendicular distance, mean and SD of wall tilt, and RMS residual. It then
prints the corrected `lidar_x` if the distance error is at least 1 cm and the
corrected yaw if the tilt is at least 0.75°.

Results: `lidar_x = −0.335` verified to 0.4 mm. Do not re-investigate LiDAR
range or offset as a cause of phantom collisions. `lidar_y = 0.0127` (1.27 cm
left of centre) by tape. `lidar_yaw = −0.0202` rad (−1.16°) came from the
tilt reading of one session and is **applied but unverified**: a 7.5 mm
difference in wall distance between the two hubs over the 0.37 m track is
exactly 1.16°, so parking error cannot be excluded. To settle it, either
re-run against two or three different walls and check the tilt reproduces, or
tape wall→left-hub and wall→right-hub; if they match, set it to 0.0. It is the
cheapest of the two candidates for the right-turn asymmetry.

The URDF `body_offset_x` was `+0.10` and drew the chassis mirrored, which made
RViz useless as a measuring instrument until it was corrected to `−0.10`. Nav2
uses the `footprint` parameter, so this was cosmetic for navigation.

## 8. Footprint and inflation

Chassis 0.62 × 0.315 m, drive axle 21 cm from the front and 41 cm from the
back. The wheels protrude: `track_width` 0.37 + `wheel_width` 0.045 = 0.415 m
across. The footprint's ±0.2025 m therefore covers the *wheels*, not the body
box, and its longitudinal extent −0.44…+0.24 includes padding at both ends.
Circumscribed radius = √(0.44² + 0.2025²) = 0.484 m.

`inflation_radius` was 0.40, which is less than that radius, so NavFn
(orientation-blind, avoiding only cells that carry cost) routed paths a mean
0.425 m from lethal cells, inside the envelope the rear corners actually
sweep. Raising it to 0.50 on both costmaps was A/B'd on a fixed route:
recovery activations 9 → 0, LiDAR returns inside the footprint 9 → 0, failed
goals 1 → 0, clearance p05 0.068 → 0.092 m, at an ~11 % time-to-goal cost
repaid by not wedging. With `cost_scaling_factor` 2.0 the band falls off
gently and biases paths to corridor centres. Both numbers are mirrored into
RPP's inflation model (`cost_scaling_dist`, `inflation_cost_scaling_factor`)
and must move together.

## 9. ToF belt geometry

Six VL53L1X. All geometry was measured, not designed:

| Sensor | Lens height | Mount | Cap |
|---|---|---|---|
| front_left / front_right | 13.5 cm, 20 cm apart | forward | 0.47 m |
| corner_left / corner_right | 15.5 cm | 45° outward | 0.55 m |
| rear | 13.5 cm | rear face | 0.47 m |
| down | 36.5 cm | pitched 43° | uncapped, filtered |

The cap rule is `height / tan(13.5° + 2°)`: half the 27° cone plus a margin,
so the floor stays outside the cone. A floor test confirmed every sensor
reads its cap on clear tile. The rear cap covers the BT's 0.30–0.70 m BackUp
distances.

The down-looker's tilt was designed at 40° but is set to 43° because its own
clear-floor reading gives `asin(0.365 / 0.538) = 42.7°`. Its baseline
(0.53 m) is the measured clear-floor slant range, oscillating upward but never
below, so the obstacle threshold is placed 1 cm under it; an object shortens
the reading by about `height / sin 43°`, so that catches anything over ~7 mm.

Lessons recorded in the handoffs: the sensors' protective film produced ~4 cm
phantom readings and was on all six but only broke the two near a ledge (a
shared variable is not a ruled-out variable); a 77 mm box and a grey dumbbell
went undetected and were never diagnosed (the three echoes to run are
`/tof/down`, `/tof/down_obstacle`, `/tof/front_left` with the object placed;
if the cone is the problem, the fix is a per-sensor ROI, which the node does
not yet support).

## 10. Nav2 tuning by bag A/B

The method: hold the map, start pose, goal, power mode and RViz-off constant;
change one parameter; record with `tools/record_nav.sh`; compare the two bags
offline, reading them in the order hardware → compute → safety → function →
speed, because each layer can fake a problem in the one below it. (The
author's bag post-mortem script that produced the numbers below is not part
of this release.) The reference "current best" run is
`nav_20260819_203831` (0 recoveries, 56.4 s per goal, 0.274 m/s mean moving
speed, clearance p05 0.089 m) against the baseline `nav_20260819_114641`.

What that process established, in the order it was learned:

| Finding | Evidence | Resulting value |
|---|---|---|
| CPU exhaustion shows up as `/tf` starvation, not in the CPU meter | costmap 0.03 m dropped `/tf` 79.6 → 57.2 Hz; `controller_frequency` 20 dropped it to 47.4, both with odom steps to 0.5 m downstream | `/tf` ≈ 79.6 Hz is checked first in every bag; `controller_frequency 15`, local resolution 0.05 |
| Every recovery had failed since the config was written | FRAME CHECK: robot inside the odom-framed costmap 100 % of the time, mean 4.4 m outside when read in map | `behavior_server.global_frame: odom` |
| The custom BT never loaded | `/behavior_tree_log` recovery counts matched the stock tree; `/plan_smoothed` always empty | Humble parameter name |
| RPP over-braked ~1.5× near obstacles | 49.8 % of driving time on the regulation floor; mean speed 0.198 → 0.224 **and** clearance improved after the fix | RPP inflation model matched to the costmap |
| Speed was capped and floored in the wrong places | 23.7 % of time pinned at 0.28, 44 % pinned at the 0.15 floor | `desired_linear_vel 0.32`, smoother max 0.32, floor 0.20 |
| The doorway wedges were approach-angle, not geometry tuning | identical coordinates to six decimals, path always at 45–57° | `rotate_to_heading_min_angle 0.52` |
| Inflation under the circumscribed radius let the rear corners hit | path-to-lethal mean 0.425 m vs 0.484 m sweep | `inflation_radius 0.50` |
| Wheel yaw was fused through the slip band | wheel +0.42 vs gyro +2.12 rad/s | `vyaw_fusion_gate 0.08` |
| Planned stops overshot ~3.6 cm | Nav2 planned 1.8/2.2 m/s² against a firmware ramp of 0.8 | smoother accel/decel ±0.8 |
| Wobble is a controller floor | 1.23–1.33 flips/m at 0.25 m/s did not respond to lookahead, curvature lookahead, control rate, resolution, inflation or smoother weights | stop tuning it |

Smac planners (Hybrid-A* and Lattice) were tried in a separate tree and
deadlock at the pinch points because they validate the footprint at the start
pose and the recoveries then abort too. NavFn's refusal to validate the start
is why it works. The experiment did establish that the costmap already
considers the rover in collision at those doorways, which has never been
physically measured (≥ 1.0 m would mean the map is wrong, ~0.8 m genuine
geometry).

The CPU A/B between the Python and C++ packages, the stated purpose of the
port, has not been run. `ps` lifetime averages suggest the four hot nodes
together take about 15 % of one core.

## 11. Firmware control constants

| Constant | Value | Why |
|---|---|---|
| `MAX_SPEED_MPS` | 1.25 | free-run top speed scaled from 0.99 m/s at 95 mm wheels; feedforward normaliser |
| `KP / KI / KD` | 0.6 / 0.40 / 0.02, × `PID_SCALE` 150 | per-wheel velocity loop; not what the trim test tunes |
| PWM clamp | ±180 of 255 | anti-windup engages at saturation |
| `TURN_BIAS_PWM` | 47 | breakaway for in-place spins (feedforward alone gives ~20 PWM at 0.1 m/s; breakaway is ~47) |
| `KICK_PWM` / `TURN_KICK_MS` | 27 / 200 | extra kick at spin onset |
| `MAX_LIN_ACCEL` / `MAX_ANG_ACCEL` | 0.8 m/s² / 3.0 rad/s² | slew ramp; a 0.21 m/s stop takes ~0.26 s and ~3 cm, and both sides decelerate identically |
| `WATCHDOG_MS` / `CMD_TIMEOUT_MS` | 500 / 1000 | hard stops; bypass the ramp |
| `CONTROL_PERIOD_MS` / `ENCODER_REPORT_MS` | 20 / 100 | 50 Hz loop, 10 Hz report |
| PWM | 20 kHz, 8-bit | encoder quantisation (~5 %) and slip are the upstream limits; do not raise the resolution |

## 12. When to re-calibrate

| If you change… | Re-run |
|---|---|
| wheels, tyres, or floor surface | straight-line test (§4), distance check |
| chassis track or motor mounts | `WHEEL_BASE_M` in both places, then turn calibration (§5) |
| encoder, motor driver or wiring | `wheel_profile.py` single-wheel test (§3) before anything else |
| IMU mount | yaw-sign pre-check (§4), axis sign parameters |
| LiDAR mount | `lidar_cal.py` against two walls (§7); update URDF |
| footprint or inflation | mirror into RPP's two inflation parameters; A/B on recorded bags (§10) |
| ToF mounts | heights in URDF, caps in `tof_belt.yaml`, floor test, down-looker tilt from its own reading (§9) |
| firmware ramp | `velocity_smoother` accel/decel |
| any Nav2 parameter | `ros2 param list /<node>` to confirm Humble declares it, then a bag |
