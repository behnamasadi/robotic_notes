# Drive scenarios for VIO / LIO evaluation

Each scenario stresses a different failure mode. Run them in this order,
recording a separate bag per scenario via `scripts/record_run.sh <tag>`,
then compare metrics with `scripts/eval.sh`.

| Tag                 | Drive profile                                                              | What it stresses                                                  |
|---------------------|----------------------------------------------------------------------------|-------------------------------------------------------------------|
| `static`            | Robot stationary for 30 s.                                                 | Bias initialisation, gravity alignment.                            |
| `slow_loop`         | Slow forward + ~5 deg/s yaw, complete one closed loop, return to start.    | Loop closure, scale drift over a small traverse.                  |
| `fast_straight`     | 2.5 m/s forward for 20 m, then full stop.                                  | Motion-blur, IMU saturation, brake transients.                     |
| `sharp_turn`        | 90° yaw in <2 s while moving at 1 m/s.                                     | Rotation-only drift (VIO sensitive), wheel-slip (wheel odom).      |
| `featureless_wall`  | Drive 10 m parallel to a tunnel wall with the camera ~50 cm from the wall. | VIO degenerates (few features); LIO should hold up.                |

## Running a scenario

Three-terminal workflow inside the sim container (or via `docker compose
exec sim`):

```bash
# Terminal 1 — sim:
ros2 launch explorer_r2_sim cave.launch.py world:=cave

# Terminal 2 — VIO + LIO + ground-truth-path publisher:
ros2 launch explorer_r2_sim vio.launch.py &
ros2 launch explorer_r2_sim lio.launch.py &
ros2 run explorer_r2_sim gt_to_path.py

# Terminal 3 — record + drive scenario, then Ctrl-C when done:
ros2 run explorer_r2_sim record_run.sh slow_loop
# (drive the joystick / keyboard / rqt_robot_steering to execute the
# slow_loop profile; Ctrl-C when finished)
```

## Reading the metrics

`evo_ape` reports **A**bsolute **P**ose **E**rror — the headline trajectory-error
number. RMSE is what's usually quoted.

`evo_rpe` reports **R**elative **P**ose **E**rror over a sliding window —
captures how much drift accumulates per metre of travel.

Rough sanity thresholds for this rig (1024-beam Ouster + RealSense-style
mono RGBD + MEMS-grade IMU):

| Scenario           | VIO APE RMSE healthy | LIO APE RMSE healthy |
|--------------------|----------------------|----------------------|
| `static`           | < 0.05 m             | < 0.02 m             |
| `slow_loop` (~20 m)| < 0.8 m              | < 0.3 m              |
| `fast_straight`    | < 1.5 m              | < 0.5 m              |
| `sharp_turn`       | < 0.6 m              | < 0.2 m              |
| `featureless_wall` | VIO often diverges   | < 0.5 m              |

These are rough bands — tune to your config, but if VIO RMSE doubles
between two runs of the same scenario you've regressed something.
