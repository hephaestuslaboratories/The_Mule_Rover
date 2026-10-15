# Straight-Line Calibration Test

**Goal:** replace trim guessing with a measured `RIGHT_WHEEL_TRIM`, and verify
distance calibration (`WHEEL_DIAMETER_M`) at the same time.

**Key fact:** `linear.x: 0.10` in `/cmd_vel` is **0.10 m/s** (closed-loop
velocity, NOT a PWM percentage). Expected travel = speed × duration:

| speed (m/s) | 5 s travel | note |
|---|---|---|
| 0.10 | 0.50 m | too slow — below stiction band, lurchy starts. avoid |
| 0.15 | 0.75 m | good test speed |
| 0.21 | 1.05 m | cruise speed — most representative, needs ~1.6 m runway |

---

## 0. One-time pre-check — IMU yaw sign (2 minutes, DO NOT SKIP)

The whole analysis assumes REP-103: **counter-clockwise rotation = yaw
increases (+)**. If the sign is flipped, every trim recommendation comes out
backwards. With the bringup running:

```bash
ros2 topic echo /imu/data --field angular_velocity
```

Rotate the rover CCW (to its left) by hand: `z` must be **positive**
(CW → negative). Note the result. If it is REVERSED, stop and report back —
we flip `axis_z_sgn` in the bridge before any calibration (and it also explains
the old YAW_ASSIST fighting).

## 1. Setup

* Flat, uniform floor (hardwood/tile — pick ONE surface, calibration is
  per-surface). Runway ≥ travel + 0.5 m, clear both directions.
* Painter's tape at the start line; tape measure along the runway
  (ground truth for the distance check).
* Bringup running (`ros2 launch rover_bringup_extended_slam
  rover_bringup.launch.py`). Nothing autonomous drives by default.
* Note the CURRENT firmware values before starting:
  `RIGHT_WHEEL_TRIM`, `LEFT_WHEEL_TRIM`, `YAW_ASSIST` (should be false),
  `WHEEL_DIAMETER_M` (0.120).

## 2. Acquire data

```bash
cd /workspaces/isaac_ros-dev/<path-to>/calibration
bash straight_test.sh 0.15 5 3        # speed, seconds, runs per direction
```

The script records a bag to `/workspaces/isaac_ros-dev/slam_bags/straight_cal_*`
and walks you through 3 forward + 3 backward bursts. After each run:
* write down the tape-measured distance (start mark → drive-axle mark),
* note drift direction by eye (sanity check vs the analysis),
* reposition to the start line.

## 3. Analyze

```bash
python3 analyze_straight_bag.py /workspaces/isaac_ros-dev/slam_bags/straight_cal_<stamp> \
    --current-right-trim 0.95 \
    --tape 0.74,0.76,0.75,0.73,0.76,0.74      # your measurements, run order (optional)
```

Per run it prints encoder distance, IMU yaw drift, and curvature; then:

* **`RIGHT_WHEEL_TRIM: old -> new`** — the measured trim. Update the constant
  in `enc_ros_cmd_vel_motor_control_pid_esp32s3.ino`, reflash.
* **encoder/tape ratio** — if > 2% off, it prints the corrected effective
  `WHEEL_DIAMETER_M`. Update it in **BOTH** places or odometry lies to SLAM:
  firmware `WHEEL_DIAMETER_M` **and** launch `wheel_diameter_m`
  (then `colcon build` is NOT needed — launch params reload on next launch).

## 4. Verify (close the loop)

Reflash with the new trim and repeat step 2 once (1 run each way is enough).
PASS: |yaw drift| < ~1.5° over 0.75 m, both directions. If it overshot the
other way, the analysis of the verification bag gives the next (small)
correction — convergence takes 1–2 iterations.

## 5. Decision table — what result means what update

| Finding | Update | Where |
|---|---|---|
| Curvature ≠ 0, consistent sign | `RIGHT_WHEEL_TRIM` per script output | firmware, reflash |
| Curvature flips sign fwd vs back | NOT a trim issue — suspect a dragging/cocked caster; inspect hardware before software | hardware |
| encoder/tape ratio > 1.02 or < 0.98 | `WHEEL_DIAMETER_M` per script output | firmware + bridge launch param, keep in sync |
| Encoder dist ≠ cmd×duration by ≫ ramp margin | velocity loop not tracking — check `/wheel/left_velocity` `/wheel/right_velocity` for saturation/asymmetry | investigate before trusting anything else |
| Yaw sign check reversed (step 0) | `axis_z_sgn: -1` | bridge launch param — and revisit YAW_ASSIST after |
| Wild run-to-run variation | floor/caster inconsistency — more runs, one surface | procedure |

## Notes

* Forward AND backward runs matter: a real wheel asymmetry produces the SAME
  curvature sign in both directions (the analyzer exploits this). A caster
  problem typically does not — that contrast is diagnostic.
* PID gains (KP/KI/KD) are NOT what this test tunes. If the velocity loop
  tracks the setpoint (encoder dist ≈ cmd × duration), gains are fine.
* Calibration is per-surface. Re-run on carpet if you map carpeted rooms.
