# Trajectory analysis — current state, workflow, and case studies

This doc has three parts:

1. **[Current state](#current-state)** — what the codebase ships right
   now, the OpenVINS knobs that are on/off, and the latest measured
   numbers.
2. **[Workflow](#workflow-record--analyse--compare)** — exact commands
   to record a bag, run the analyser, and compare against a previous
   run.
3. **[Case studies](#case-study-1--forward-drive-mono-vio-fails)** —
   what specific bags taught us, with the failure modes and the fixes
   that came out of each. New iterations get appended at the bottom.

---

## Current state

### Numbers from the most recent runs

| Run                             | GT path | LIO err | LIO ratio | VIO err | VIO ratio | Notes |
|---------------------------------|--------:|--------:|----------:|--------:|----------:|-------|
| `run_20260514T154145Z` (v1)     | 40.27 m | 0.64 m (1.6 %) | 1.01 | 195.28 m (485 %) | 7.21 | Forward drive. VIO init contaminated by spawn drop. Pre-CLAHE config. |
| `run_20260514T162146Z` (v2)     | 58.52 m | **0.15 m (0.3 %)** | **1.01** | **4030 m (6887 %)** | **378** | Figure-8. CLAHE + aggressive tracker → catastrophic VIO blow-up. Reverted. |
| `run_20260514T182108Z` (v3)     | 51.28 m | **0.16 m (0.3 %)** | **1.01** | 139.75 m (272 %) | **5.40** | Tracker reverted to defaults + camera bumped to 640×480. **70× better than v2**, slightly better than v1. Z-axis drift visible — still a parallax/scene-texture problem ([case study 3](#case-study-3--640480-revert-mono-vio-back-to-baseline)). |
| `run_20260514T184552Z` (v4)     | 17.89 m | **0.09 m (0.5 %)** | **1.00** | 20.22 m (113 %) | **1.62** | **Circle outside the tunnel, no lattice texture.** VIO ratio 5.40 → 1.62. Remaining 113 % end-pos error is almost entirely the unobservable initial yaw — Umeyama-aligned APE would be tiny ([case study 4](#case-study-4--out-of-the-tunnel-circle-mono-vio-finally-works)). |
| `run_20260514T210126Z` (v5)     | 23.17 m | 0.13 m (0.6 %)     | 1.01     | 33.61 m (145 %) | 3.59     | **Stereo OpenVINS** (added rs_front_right camera, use_stereo:true). Surprise: *worse* than v4 mono. Two confounders — sim RTF dropped (bandwidth saturated by 2× 640×480@30) so every estimator got less data, AND static init fired again in 1.3 ms and converged to 180° yaw. Stereo never got a chance to fix yaw. Fix shipped: `init_imu_thresh: 50.0` forces dynamic init ([case study 5](#case-study-5--stereo-openvins-doesnt-magically-fix-things)). |
| `run_20260514T212506Z` (v6)     | 35.80 m | **0.09 m (0.3 %)** | **1.00** | 7.50 m (21 %)  | **1.17** | **Stereo + dynamic init: the big win.** Aligned APE 0.48 m, aligned+scale 0.47 m → 9× better than v4 mono, 20× better than v5 stereo-static-init. Scale ratio 1.17 (was 3.59) — stereo basically eliminated scale drift. ZUPT now firing consistently with 168 stereo features. Mono-VIO era is over for this codebase ([case study 6](#case-study-6--stereo-dynamic-init-is-actually-the-answer)). |

LIO is rock solid. **VIO is now usable** as of v6 — stereo + dynamic
init together resolve the unobservable-yaw + scale-drift failure
modes that dominated v1–v5.

### Why VIO trajectories look so bad — the headline conclusion

Across every bag we've recorded, VIO trajectories appear catastrophically
wrong in raw RViz / `summary.md` numbers. But after running
`evo_ape -a` (Umeyama alignment, which rotates the VIO frame onto GT
before computing error), **most of the apparent error vanishes**.
Proof from v4:

```
                 RMSE       Meaning
─────────────  ────────    ────────────────────────────────────────
Raw           17.80 m       apparent VIO error
Aligned -a     4.25 m       ← 76 % of error was just an unobservable yaw rotation
Aligned -as    1.63 m       ← additional 62 % was scale drift; remainder is real shape error
```

**The root cause is unobservable yaw in mono IMU-only init:** gravity
fixes the IMU's pitch and roll, but it can't fix yaw (rotation around
the gravity axis). OpenVINS' static init has to pick *some* yaw and
ends up with whatever the first IMU samples imply — effectively
random relative to gz world. From then on the entire VIO trajectory
is correct in shape (modulo the parallax-limited scale drift) but
rotated into the wrong frame.

This explains the user-visible "VIO is going in the wrong direction"
across every run from v1 onwards. It is **not a config bug**. It's
a fundamental limitation of monocular VIO without a heading sensor.
Fixes: magnetometer fusion (`/magnetometer` already published —
needs `robot_localization`), stereo VIO (parallax fixes yaw
observability), or a known-heading prior (rover spawns facing world
+X — would require a small patch to OpenVINS' init).

See [case study 4](#case-study-4--out-of-the-tunnel-circle-mono-vio-finally-works)
for the full evidence.

### Why published VIO numbers look so much better than ours

You may have come into this expecting "in a noise-free, perfectly
synchronised simulator the VIO should be near-perfect" — and then been
disappointed by our 17.80 m raw APE. That's a reasonable expectation,
but it doesn't match how VIO results are actually reported in the
literature. Three things are usually elided in the headline numbers:

1. **"VIO" in papers almost always means stereo VIO.** OpenVINS'
   headline numbers from the original paper come from the *stereo*
   EuRoC configs. Mono numbers from the *same* paper are 2–4× worse.
   "OpenVINS does X cm on EuRoC" is a stereo statement; people quote
   it without saying so. Our mono setup is a different animal entirely,
   and going stereo is exactly the recommended fix.

2. **Published APE is Umeyama-aligned (`evo_ape -a`).** Every benchmark
   you'll read — KITTI, EuRoC, TUM-VI — uses the aligned metric, which
   subtracts out the unobservable initial yaw + scale offset *before*
   computing error. The "0.5 m APE on a 100 m trajectory" number you
   see in a paper is what we call `-as` aligned-with-scale APE. Our
   v4 -as is **1.63 m on 17.89 m (9 %)**. Real-world EuRoC mono numbers
   from OpenVINS' own paper are around 10–20 % aligned-APE. **So our
   "shape" performance is actually in the ballpark of published mono
   VIO** — we just don't normally report the same metric. The raw APE
   number that looked catastrophic is mostly the yaw offset that every
   paper invisibly subtracts before reporting.

3. **Public benchmark scenes are parallax-friendly by construction.**
   The EuRoC MAV drone yaws, pitches, and rolls naturally — that
   generates rich parallax on every axis on every frame, which makes
   monocular depth observable. Our rover is constrained to a flat
   plane with limited yaw rate, looking *forward* into a corridor. KITTI
   has the same problem (car driving forward → mono VO struggles, which
   is exactly what the user's own
   [OpenCVProjects/docs/kitti.ipynb](https://github.com/behnamasadi/OpenCVProjects/blob/master/docs/kitti.ipynb)
   shows with `goodFeaturesToTrack` + KLT). Our v4 (circle outside the
   tunnel) gave VIO actual parallax, and the path-length ratio dropped
   immediately from 5.40 to 1.62.

There's also a more honest reason that's worth saying out loud:
**clean sim images aren't easier than real images, they're just
*different*.** Real cameras have rolling shutter, vignetting, AE
flicker, motion blur — but they also have *unmodelled, varied texture*
in nearly every frame. Our gz tunnel has a regular hexagonal lattice
that's actively hostile to KLT correspondence (case study 3). Our
outdoor area in v4 had varied wood and concrete textures — closer to
what real cameras see — and VIO immediately did better. Sim
"cleanness" buys you no parallax, no scene variety, and (as we found
with the CLAHE backfire in case study 2) actively breaks real-camera
tuning tricks.

**Net of all three**: we are not getting bad VIO results because
something is broken. We're getting *normal* mono-VIO results for a
ground-constrained rover in sometimes-pathological scenes, and the
apparent badness is mostly amplified by the unaligned APE metric
we've been looking at. This is exactly why every commercial AGV uses
**LiDAR + IMU**, not mono VIO — and exactly why LIO on this rig is
0.09 m on a 17.89 m drive while mono VIO is fighting fundamental
unobservabilities.

### Where to go from here

Two reasonable next steps depending on what you want to learn:

| Want | Do |
|---|---|
| A trustworthy real-world trajectory estimator on this rig | LIO is already at 0.5 %. Ship it; treat VIO as a fallback for lidar-degraded scenes (smoke, fog, dust). |
| To make mono VIO genuinely competitive | Wire up stereo OpenVINS (a second forward-facing camera in the SDF + two config-line changes). Resolves ~91 % of v4 error. |
| To compare VIO algorithms head-to-head | Stereo OpenVINS first (so the baseline is fair), then add VINS-Fusion as a `third_party/` submodule with loop closure + optional GPS fusion. Run all three estimators on the same bag and let `analyze_bag.py` compare them. |

### Algorithm-choice cheat sheet — three concrete options

If "switch from OpenVINS" is on the table, here are the three serious
alternatives ranked by expected impact for *this* rig:

#### Option A — Stereo OpenVINS *first* (recommended next step)

Addresses **~91 %** of the v4 error directly (the yaw + scale parts).
Same codebase, same launch flow, same analyser. The piece that's
missing is a *second forward-facing camera in the SDF* — `rs_left` and
`rs_right` look sideways and don't form a stereo pair with `rs_front`.
Adding `rs_front_right` (a few cm to the right of `rs_front`, same
orientation) gives a real stereo baseline.

Workflow: add the camera to `models/explorer_r2/model.sdf`, bridge it
in `config/bridge.yaml`, add a `cam1:` block to
`config/openvins/kalibr_imucam_chain.yaml`, flip `use_stereo: true` and
`max_cameras: 2` in `estimator_config.yaml`. Rebuild colcon, record a
new bag with the same drive profile as v4.

Expected outcome on a v4-style drive: aligned APE drops from 4.25 m to
roughly 0.5–1 m, raw APE drops from 17.80 m to 1–3 m — within reach of
LIO numbers.

#### Option B — VINS-Fusion (the natural OpenVINS competitor)

[HKUST-Aerial-Robotics/VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion).
What you actually get vs OpenVINS:

| Feature                       | OpenVINS (current)          | VINS-Fusion                                                            |
|-------------------------------|-----------------------------|------------------------------------------------------------------------|
| Backend                       | MSCKF filter                | Sliding-window bundle adjustment — tighter optimisation, more accurate per-frame |
| Front-end                     | KLT (default) or ORB        | Always feature-based (good descriptors + RANSAC)                       |
| Loop closure                  | Topics published, rarely fires | **First-class DBoW2 loop closure** — corrects accumulated drift on return-to-known-place |
| GPS fusion                    | Not directly                | **Built-in `vins_node` + `global_fusion` that consumes GPS** — could anchor our `/navsat` |
| Mono yaw observability        | unobservable                | unobservable (same physics)                                            |
| Maturity                      | very actively maintained    | very actively maintained                                               |
| Citation count                | high                        | higher                                                                 |

VINS-Fusion's edge over OpenVINS for this rig: **loop closure** and
**GPS fusion**. Loop closure would visibly snap the trajectory back to
truth on return-to-start drives. GPS fusion is currently the only thing
in this list that would *actually* solve the unobservable-yaw problem
on a single sensor stack (a brief translation under GPS resolves
heading).

Add as `third_party/VINS-Fusion` submodule + new `vins.launch.py` + one
line in `analyze_bag.py`'s `ODOM_TOPICS`. Architecture already supports
this — same pattern we used for FAST_LIO.

#### Option C — Kimera-VIO (skip unless you need full SLAM)

[MIT-SPARK/Kimera-VIO](https://github.com/MIT-SPARK/Kimera-VIO).
Heavier; also supports RGBD which would directly use our
`/rs_front/depth`. Comparable to VINS-Fusion for trajectory accuracy,
but bigger memory footprint and more setup work. Worth picking up only
if you want the full Kimera SLAM stack (semantic mapping, mesh
reconstruction, etc.) on top of VIO.

### Recommended order of operations

Don't try to compare alternatives until you have a fair baseline:

1. **Step 1 — Stereo OpenVINS** (option A above). Sets the "real"
   mono-vs-stereo numbers for this rig. *Without this, comparing
   any other VIO is meaningless — you'd be comparing two crippled
   mono setups.*
2. **Step 2 — Add VINS-Fusion alongside** (option B). Run all three
   estimators (OpenVINS-stereo, VINS-Fusion-stereo, FAST_LIO) on the
   same bag. `analyze_bag.py` is topic-agnostic; one line to add the
   VINS topic.
3. **Step 3 — Optionally try Kimera-VIO** if Steps 1–2 leave you
   wanting semantic mapping or mesh reconstruction.

Steps 1 and 2 are independent of each other in scope; either can be
done first if Step 1 has been completed.

### What changed since defaults — currently shipping

These are the deltas from upstream OpenVINS defaults that are
**currently in the config / launch files**. The reasoning for each
is in [`docs/PARAMETERS.md`](PARAMETERS.md).

| Where                         | Knob                  | Default     | Currently   | Why                                                                          |
|-------------------------------|-----------------------|-------------|-------------|------------------------------------------------------------------------------|
| `launch/vio.launch.py`        | `VIO_START_DELAY_SEC` | 0           | **8.0 s**   | Don't subscribe to `/imu` until the spawn-drop transient is over.            |
| `config/.../estimator_config` | `init_window_time`    | 2.0 s       | **5.0 s**   | Average over enough stationary samples for clean gravity-vector estimate.    |
| `config/.../estimator_config` | `init_imu_thresh`     | 1.5         | **0.3**     | Lower threshold matches our low-noise gz IMU (σ ≈ 1e-2 m/s²).                |
| `config/.../estimator_config` | `init_dyn_use`        | false       | **true**    | Dynamic-init fallback if static path misses.                                 |
| `config/.../estimator_config` | `calib_cam_extrinsics` | true       | **false**   | Sim calibration is exact; online refinement just adds noise.                  |
| `config/.../estimator_config` | `calib_cam_intrinsics` | true       | **false**   | Same.                                                                        |
| `config/.../estimator_config` | `calib_cam_timeoffset` | true       | **false**   | Same.                                                                        |
| `config/.../estimator_config` | `try_zupt`            | false       | **true**    | Zero-velocity update prevents bias drift while idle.                         |
| `config/.../kalibr_imu_chain` | `Tw, Ta, R_*, Tg`     | (omitted)   | **identity/zero** | Required even when no correction needed — omitting them crashes OpenVINS. |

The front-end tracker knobs (`histogram_method`, `num_pts`,
`fast_threshold`, `min_px_dist`, `track_frequency`) are at **upstream
defaults**, after a one-iteration experiment where flipping them to
"low-light cave" values made VIO 50× worse — see
[Case study #2](#case-study-2--clahe-aggressive-tracker-backfire) below.

### Scripts that ship with the repo

| Script                              | Purpose                                                                                                |
|-------------------------------------|--------------------------------------------------------------------------------------------------------|
| `scripts/gt_to_path.py`             | Auto-started by `world.launch.py`. Bridges `/ground_truth/pose` (gz) → `/ground_truth/odom` + `/path`. |
| `scripts/lidar_field_adapter.py`    | Auto-started by `lio.launch.py`. Adds `ring` + `t` fields gz lidar omits but FAST_LIO needs.           |
| `scripts/record_run.sh <tag>`       | Wraps `ros2 bag record` for all the topics analyse_bag wants.                                          |
| `scripts/analyze_bag.py <bag>`      | **The main analyser.** Reads bag → produces plots + summary.md.                                        |
| `scripts/eval.sh <bag>`             | Wraps `evo_traj` + `evo_ape` for offline Umeyama-aligned error.                                        |

---

## Workflow: record → analyse → compare

After making config changes you want to evaluate, run this loop.

### 1. Bring up Mode 1 and wait for VIO to start

```bash
cd ~/ros2_ws/src/explorer_r2_sim
docker compose down            # clean state
xhost +local:root
docker compose up
```

VIO is intentionally delayed 8 s after launch. Watch terminal 1 for:

```
[run_subscribe_msckf-X]: process started with pid [...]
```

That's t+8s. Stay stationary another 5–10 s so static init has clean
data.

### 2. Record

```bash
# In another terminal:
mkdir -p ~/ros2_ws/runs
docker compose exec sim bash -ic "ros2 bag record \
  -o /ws/runs/run_$(date -u +%Y%m%dT%H%M%SZ) \
  /imu /rs_front/image /rs_front/camera_info \
  /lidar/points /lidar/points_lio \
  /ground_truth/pose /ground_truth/odom /ground_truth/path \
  /ov_msckf/odomimu /ov_msckf/pathimu \
  /Odometry /path \
  /cmd_vel /tf /tf_static"
```

### 3. Drive a useful profile

Suggested 80–90 s mix:

| 0–15 s     | Stationary             | Lets VIO init on clean IMU data |
| 15–30 s    | Slow forward, ~0.3 m/s | Easiest motion                  |
| 30–40 s    | **STOP**               | Watch RViz: green should freeze if ZUPT works |
| 40–70 s    | Figure-8 / strong yaw  | Mono VIO's best parallax case   |
| 70–80 s    | Stop                   | Second ZUPT test                |

Ctrl-C the bag recorder when done.

### 4. Analyse

`analyze_bag.py` is a pure-Python script with **no ROS dependency**.
Its requirements are just `numpy`, `matplotlib`, `rosbags`, and
optionally `opencv-python` (for the camera-frame samples). You can run
it from any of three places — pick whichever matches your habits:

#### Option A — conda env on the host (recommended for repeat use)

Use **Python 3.12** to match the ROS 2 Jazzy base image — that keeps
`rclpy` imports clean if you ever mix this Python with a sourced ROS
overlay. The repo's conda env (`explorer_r2_sim`) is already on 3.12.13.

```bash
conda activate explorer_r2_sim

# Analyser deps + evo's full runtime deps. evo itself ships with the env;
# scipy/pandas/pyyaml/seaborn/numexpr/pygments are evo's runtime deps it
# doesn't pull in automatically.
pip install numpy matplotlib rosbags opencv-python \
            scipy pandas pyyaml seaborn numexpr pygments greenlet   # once

cd ~/ros2_ws
python src/explorer_r2_sim/scripts/analyze_bag.py runs/run_<UTC>
```

> **Do not pass `--user` or `--break-system-packages` inside a conda
> env.** `--user` installs to `~/.local/lib/python3.12/site-packages`,
> which Python imports *before* the conda env — so the `--user` numpy
> shadows the conda numpy and you end up with two copies fighting.
> Plain `pip install` (no flags) puts everything in
> `~/anaconda3/envs/explorer_r2_sim/lib/python3.12/site-packages/`,
> where it belongs.

If you previously ran `pip3 install --user --break-system-packages …`
and want to undo the shadowing:

```bash
pip uninstall --user numpy matplotlib rosbags opencv-python \
                     contourpy fonttools kiwisolver pillow pyparsing \
                     cycler lz4 ruamel.yaml zstandard
```

#### Output (all three options)

The script auto-creates `<bag>_report/` next to the bag and writes:

| File                    | What it shows                                                           |
|-------------------------|-------------------------------------------------------------------------|
| `summary.md`            | Topic rates, trajectory stats, end-position error, path-length ratio, IMU sanity, auto-interpretation. |
| `trajectory_xy.png`     | Top-down XY of all three estimators. Circle = start, square = end.       |
| `trajectory_3d.png`     | Full 3D view (X, Y, Z), same colour palette as RViz.                     |
| `trajectory_xyz.png`    | Per-axis (X, Y, Z) over normalised time — best for spotting drift on one axis. |
| `imu_accel.png`         | \|accel\| vs the 9.81 m/s² line + per-axis. Stationary should sit at 9.81. |
| `imu_gyro.png`          | Gyro components over time.                                              |
| `camera_frames/*.png`   | Six evenly-spaced /rs_front/image samples — visual sanity check.        |

### 5. (Optional) Umeyama-aligned APE with `evo`

`evo_ape -va` aligns the estimator's frame to GT before computing
error — that subtracts out the unobservable initial yaw and tells you
the *true* drift, separately from frame misalignment.

```bash
# Inside the conda env from Option A, evo is already present.
# If you skipped Option A, install it with: pip install evo
mkdir -p /tmp/eval && cd /tmp/eval
evo_traj bag2 ~/ros2_ws/runs/run_<UTC> \
  /ground_truth/odom /ov_msckf/odomimu /Odometry --save_as_tum
evo_ape tum ground_truth_odom.tum ov_msckf_odomimu.tum -va --plot --plot_mode xy
evo_ape tum ground_truth_odom.tum Odometry.tum            -va --plot --plot_mode xy
```

### 6. Compare against a previous run

Append the new row to the [numbers table](#numbers-from-the-most-recent-runs)
at the top. The analyser flags whether each estimator looks healthy /
has scale drift / has yaw offset in its `summary.md`; comparing two
bags' summaries side-by-side tells you whether a config change helped
or hurt.

#### What "healthy" looks like for each metric

After the v2 case study and the revert + camera-resolution bump, the
targets for the next bag (call it v3) are:

| Metric                                                            | v1 (pre-fixes)     | v2 (CLAHE + aggressive) | v3 target                                       |
|-------------------------------------------------------------------|-------------------:|------------------------:|-------------------------------------------------|
| **VIO path-length ratio**  (`<vio_path> / <gt_path>`)              | 7.21               | 378                     | **< 5** = clean improvement; **< 2** = genuinely usable mono VIO |
| **VIO end-position error**                                         | 195 m (485 %)      | 4030 m (6887 %)         | **< 30 m** on a ~50 m drive                     |
| **VIO Z during a stop** (look at `trajectory_xyz.png`)             | rising line        | exploding               | **flat line** — confirms ZUPT is firing         |
| **`imu_accel.png`** \|accel\| at rest                              | ~9.81 m/s²         | ~9.81 m/s²              | unchanged — IMU itself is fine                  |
| **LIO end-position error**                                         | 0.64 m (1.6 %)     | 0.15 m (0.3 %)          | ≲ 0.5 m. If this regresses, *something else* broke. |

#### Capture the VIO init log too

The OpenVINS init log tells you whether init happened on clean data.
Open a third terminal during the bag run and dump it:

```bash
docker compose logs sim 2>&1 | grep -E "\[init\]" \
  > /tmp/vio_init_$(date -u +%Y%m%dT%H%M%SZ).log
```

Healthy init (after the 8 s startup delay + 5 s window) looks like:

```
[init]: successful initialization in ~5.0 seconds
[init]: orientation = (small, small, small, ≈1)   ← quaternion near identity
[init]: bias gyro  = (~0, ~0, ~0)
[init]: bias accel = (~0, ~0, small)
```

Diagnosis if it doesn't:

| What you see                                                         | Likely cause                                                                     |
|----------------------------------------------------------------------|----------------------------------------------------------------------------------|
| `init in 0.0003 seconds`                                             | Triggered on spawn-drop impact (the v1 bug — should be cured by the 8 s delay).  |
| `orientation = (0, 0, 1, 0)` or similar with large quaternion z      | 180° yaw — IMU-only init can't recover yaw. Drive a figure-8 so dynamic init can refine it. |
| `bias accel.z` far from zero                                         | Init grabbed a tilted-rover gravity vector; let the rover settle longer next time. |
| No `[init]` line at all                                              | Init never fired — `init_imu_thresh` too high or `init_window_time` too short for your drive profile. |

#### What goes in the row

Once you've run the analyser, three fields are enough for the table:

- `LIO err` — from `End-position error vs GT` in `summary.md`, the
  `|err|` value and its `|err| / GT_path` percentage.
- `LIO ratio` — from `Path-length ratio (estimator / GT)` in
  `summary.md`.
- Same two columns for VIO.

Paste the row (or just send me the `summary.md`) and the analyser's
auto-interpretation block is usually self-explanatory.

---

## Aside: APE, RPE, and Umeyama alignment — what the numbers mean

When you read "VIO RMSE is 0.48 m" in this doc or in any VIO paper,
it's almost never the raw error. Here's what each metric actually is
and why we report aligned values most of the time.

### Setup

Two time-synchronised trajectories:

| | |
|---|---|
| **GT** | `{ Tgt(t1), Tgt(t2), …, Tgt(tN) }` — each `Tgt` is an SE(3) pose (position + orientation) of the rover at time `ti` |
| **Est** | `{ Test(t1), …, Test(tN) }` — same timestamps, from VIO/LIO |

### APE — Absolute Pose Error

The per-sample error at timestamp `ti` is

```
Ei = Tgt(ti)⁻¹ ∘ Test(ti)         (full SE(3) form)
ei = pgt(ti)   - pest(ti)         (position-only form, ‖·‖ in metres)
```

Aggregate by RMSE:

```
APE_RMSE = √( (1/N) · Σ ‖ei‖² )
```

`max`, `mean`, `median` are computed the same way. `evo_ape` outputs
all of them.

**Catch with raw APE**: it implicitly assumes `Tgt` and `Test` are in the
*same world frame*. They almost never are.

- `Tgt` is in the gz world frame (x-forward, y-left, z-up at the world origin).
- `Test` is in whatever frame the estimator chose at init. For OpenVINS that's `global`; for VINS-Fusion it's `world`; for FAST_LIO it's `camera_init`. These frames are **arbitrarily rotated** relative to gz world because IMU-only init can't determine yaw (gravity has no yaw component) — the unobservable-yaw problem from
[the headline conclusion](#why-vio-trajectories-look-so-bad--the-headline-conclusion).

So a *correct* estimator with a wrong-yawed world frame still gives a
huge raw APE. The error is measuring the frame mismatch, not the
estimator quality.

### Umeyama alignment — strip out the unobservable frame offset

Umeyama (1991) gives a closed-form rigid (or similarity) transform `S`
that minimises

```
Σ ‖ Tgt(ti) - S ∘ Test(ti) ‖²
```

- `-a` rigid alignment:  S = (R, t)  — removes initial rotation + translation
- `-as` similarity alignment:  S = (s·R, t)  — removes rotation + translation + uniform scale

`-as` adds a single scalar scale factor on top of `-a`. For mono VIO,
scale is also weakly observable, so `-as` is the metric used to
report "true trajectory shape error" in mono benchmarks.

After alignment, the aligned APE is

```
APE_RMSE_aligned = √( (1/N) · Σ ‖ Tgt(ti) - S ∘ Test(ti) ‖² )
```

This number is what mono-VIO papers report. It strips out the parts
of the trajectory the estimator fundamentally *cannot* determine
without an external heading sensor, leaving only the *avoidable* error
(drift, scale drift, bias errors, tracker mistakes).

### RPE — Relative Pose Error

A second metric used in published benchmarks. For a window `Δ`:

```
Erel(ti) = (Tgt(ti)⁻¹ ∘ Tgt(ti+Δ)) ∘ (Test(ti)⁻¹ ∘ Test(ti+Δ))⁻¹
```

Each `Erel` measures how well the estimator captures *incremental*
motion over Δ seconds. RPE is *unaffected* by initial-yaw and scale
ambiguities because both trajectories are made relative to time `ti`.
Lower RPE = more locally consistent estimator. KITTI's headline
benchmark is RPE-based for this reason.

### Which to use when

| Goal | Metric | Why |
|---|---|---|
| Compare estimator *shapes* against truth in this project | **`-as` aligned APE** | Removes the unobservable yaw + scale; what mono VIO papers report |
| Compare estimators against each other on the same bag | **`-a` aligned APE** | Both estimators have arbitrary yaw, so align both to GT and they're comparable |
| Measure *short-term* drift | **RPE** | Doesn't care about initial frame, measures local consistency |
| "Does the trail overlay GT in RViz?" | **raw APE** | But useless as a quality metric without a heading sensor |

For our project, **aligned APE (`-a` or `-as`) is the honest number**.
The raw APE dominates the v1-v5 case-study tables because it's
visceral and matches what you see in RViz — but the conclusions are
based on aligned numbers.

## Aside: ROVIO on the drone didn't have "arbitrary yaw" — why?

User raised the perfectly fair question: their colleague ran ROVIO on
a drone with **6-axis IMUs** (no magnetometer) and **two IMUs mounted
in opposite orientations** for bias cancellation, and never saw the
"trajectory is in the opposite direction" failure we keep observing
on this rover. What's different?

The honest answer is three things, none of which is the IMU layout:

1. **They were comparing ROVIO to ROVIO.** With no ground truth in
   the field, you can't *see* the arbitrary yaw offset — the
   estimator's trail is in its own internal world frame, and as long
   as motion is consistent within that frame, everything "looks
   right." We see the 180° flip on this rig because we have gz GT
   and compute raw `‖VIO − GT‖`. The colleague's drone almost
   certainly had the same offset, invisible.
2. **Drone motion is parallax-rich.** Constant rotation + translation
   in 3D means ROVIO's camera-update loop pins yaw to *some*
   consistent value within seconds of takeoff, even though that
   value is arbitrary relative to north. Once pinned, it stays.
3. **Two opposing IMUs cancel bias drift.** With opposite mounting,
   accelerometer/gyro biases of the two units roughly cancel when
   averaged. That stops the wrong-yaw decision from *compounding*
   over time — whatever yaw was chosen at init stays stable for the
   whole flight. **But this doesn't add a yaw observation**; it just
   keeps the wrong yaw from drifting further wrong.

Aligned APE on the colleague's setup, if you'd had a Vicon system to
measure ground truth, would have shown the same yaw rotation as we're
seeing here. It just never had to be reported, so it didn't matter
for navigation.

**Conclusion for our project**: the arbitrary-yaw effect is
fundamental to any mono-VIO (or stereo-VIO without heading sensor)
when reported against a fixed-world ground truth. It's an *evaluation
artefact*, not a flaw in the estimator. The aligned-APE numbers we
report in v4 (4.25 m), v6 (0.48 m), and ongoing runs are the honest
quality measures. The "trail goes opposite direction in RViz" view is
the unavoidable cost of having ground truth.

## What you should do next — three concrete options

In rough order of cost vs payoff, **after the current Step 2
(VINS-Fusion) test lands**:

1. **Record v7 with the current setup** (OpenVINS-stereo + VINS-Fusion
   + LIO + GT running in parallel) and let the head-to-head table
   below it speak. Aligned APE for each, ranked. This is the
   *single most informative* thing you can do in the next 30
   minutes. If VINS-Fusion's motion-based init plus loop closure
   produce a meaningfully smaller aligned APE than OpenVINS, we
   know algorithm choice matters and Step 3 (Kimera) is worth doing.
   If the two are roughly tied, algorithm choice barely matters here
   and the next lever is the *sensor stack* (magnetometer or radar).

2. **Add magnetometer fusion via `robot_localization`** if you want
   the trail to *visually overlay GT* in RViz (raw APE that matches
   aligned APE). `/magnetometer` is already published. This is the
   only thing in the current stack that fixes the arbitrary-yaw
   offset at the source rather than papering over it with alignment.
   ~2 hours of integration. Doesn't help estimator quality per se,
   but makes RViz output match GT visually — useful for demos.

3. **Step 3: add Kimera-VIO** as planned. Worth doing only if v7
   shows VINS-Fusion clearly different from OpenVINS — otherwise
   Kimera is unlikely to be transformatively different from either.

**My recommendation**: do (1) first this session. It's the cheapest
information. Decide between (2) and (3) based on what v7 shows.

## Aside: what is parallax, and why is forward driving the worst case?

The short version: **a monocular camera estimates depth from how
things move across the image when the camera itself moves.** Stuff
near you swings across the image fast; stuff far away barely moves.
The *difference* between those motion rates is parallax, and it's the
only information a single 2D image gives you about 3D depth.

You wrote it right: "stationary observer, something near and something
far — the near one looks like it moved more." Same idea for a moving
observer.

### Why driving forward kills it

The bad case isn't *distance* per se — it's that the camera moves
**along its own optical axis** (the line the camera is pointing).

Imagine driving toward a brick wall:

- The brick **directly in front** of you (on the optical axis) just
  gets *bigger* as you approach. It doesn't slide sideways in the
  image; it expands radially. **Zero parallax.**
- A brick **off to one side** does move across the image — but the
  motion is along a radial line outward from the image centre. The
  amount of motion depends on how close the brick is *and* how far
  off-axis it is.
- A brick **at the edge of the image** (90° off-axis) gives the
  maximum parallax, just like a sideways camera move would.

Mathematically, if you're at distance `D` from a point, and you move
forward by `Δd`, the **on-axis** point doesn't shift at all — it just
scales. The **off-axis** point at angle `θ` shifts by roughly
`(Δd · sin θ) / D` in image angle. Plug in `θ = 0` (dead ahead) and
you get zero, regardless of `D`. That's the blind spot.

### The tunnel makes it worse

The geometry of our sim makes this scenario pathological:

- The rover drives forward.
- The camera faces forward (`rs_front`).
- **The tunnel walls are within a few metres of the camera.** The
  ceiling, floor, and side walls are *near* the optical axis — they
  occupy most of the 60° HFOV — so most of what the camera sees is
  close to "dead ahead."
- Tunnel walls are also pretty self-similar: rock texture, dim
  lighting. KLT can track a feature, but if the filter only sees the
  same wall move radially outward as you approach, it doesn't know
  whether the wall is 2 m away or 5 m. **Scale is weakly observable.**

The result: VIO can correctly track *which* features are moving, but
it can't pin down *how fast* the camera is actually moving in metres,
because every consistent (velocity, depth) pair produces the same
image motion. Over a 40 m drive that uncertainty compounds into the
7× scale drift we saw in [Case study 1](#case-study-1--forward-drive-mono-vio-fails).

### Quick numerical example

Camera at origin looking along +X. Two features:

| Feature | Position (X, Y, Z) | Image position before move | Image after camera moves +1 m in X | Image shift |
|---|---|---|---|---|
| Dead ahead, 10 m | (10, 0, 0) | centre | centre | **0 pixels** |
| Dead ahead, 5 m  | ( 5, 0, 0) | centre | centre | **0 pixels** |
| 2 m to the side, 10 m ahead | (10, 2, 0) | 11.3° off-axis | 12.5° off-axis | 1.2° (~3 px) |
| 2 m to the side, 5 m ahead  | ( 5, 2, 0) | 21.8° off-axis | 26.6° off-axis | 4.8° (~13 px) |

The first two rows have **identical image motion** despite being at
different depths. From a single mono camera, you can't tell which is
which. The third and fourth rows do differentiate, but only because
they're off-axis — and points right at the image edge are sparse in a
narrow-corridor scene.

### What fixes it

Three things, in order of how much they help:

1. **Sideways motion** (translation perpendicular to the optical axis).
   A pure sideways move turns every point into a row 4 of the table
   above — strong parallax, strong depth signal. Rotation alone
   doesn't help (it changes pixel positions without changing depth),
   but a yaw + small forward velocity (the figure-8 we recommended)
   gives you continuous sideways translation.
2. **A second camera** (stereo). With a baseline of, say, 10 cm
   between two cameras, you get depth-from-disparity at *every*
   frame — no motion required. Scale is recoverable from the first
   frame onward. This is why the user's stated next step (stereo
   VIO) is the real fix and the other tweaks are just palliatives.
3. **Depth from another sensor** (RGBD camera, lidar). The depth
   image from `rs_front` is already published as `/rs_front/depth`;
   feeding it into a depth-aware VIO (like Kimera) would fix scale
   directly. FAST_LIO does this implicitly with lidar — that's why
   LIO is converging to 1.6 % drift while VIO at 7× is parallax-
   limited.

---

## Case study 1 — forward drive, mono VIO fails

A case study from `runs/run_20260514T154145Z/` — a real bag the user
recorded while teleoperating the rover roughly straight forward across
the tunnel world. The point isn't the specific numbers; it's the
*methodology* — how to spot scale drift, init-induced yaw error, and
mono-VIO failure modes from the raw trajectory data.

## Setup

| | |
|---|---|
| Bag duration | 155 s |
| Topics recorded | `/imu`, `/rs_front/image`, `/lidar/{points,points_lio}`, `/ground_truth/{pose,odom,path}`, `/ov_msckf/{odomimu,pathimu}`, `/Odometry`, `/path`, `/cmd_vel`, `/tf`, `/tf_static` |
| Drive profile | Mostly straight forward, then some curving |
| World | `tunnel` (default) |
| Mode | 1 — full stack via `cave.launch.py` |

## Results

```
Name      N   start  (x, y, z)            end    (x, y, z)            path length
------  ----  -------------------------  --------------------------  -----------
GT      6892  (-0.01, -0.00, -0.01)      (+30.57, +0.53,  -2.69)        40.27 m
LIO      499  (-0.03, -0.00, -0.01)      (+30.46, +0.43,  -3.31)        40.61 m
VIO     9790  (+0.02, -0.00, +0.07)      (-164.62, +6.17, -4.49)       290.23 m
```

```
LIO  end-position error vs GT:  (-0.12, -0.10, -0.62) m   |err| =   0.64 m
VIO  end-position error vs GT:  (-195.19, +5.65, -1.80) m  |err| = 195.28 m
```

LIO tracks GT to within 0.64 m over a 40 m drive — that's 1.5 % final
drift, mostly a 0.62 m Z error. **LIO is healthy.**

VIO accumulates 195 m of end-point error and reports 290 m of path
length while the rover physically travelled 40 m (7× over-estimate of
total motion). **VIO is fundamentally broken in this configuration.**

## Why VIO fails — three compounding causes

### 1. Mono camera + forward-driving = worst-case parallax

A monocular VIO algorithm needs **parallax** to triangulate features
and estimate their depth. When the camera's optical axis is aligned
with the direction of motion (which it is here: `rs_front` looks
forward, the rover drives forward), features near the optical center
get **no parallax** — they just expand in the image as the camera
approaches them.

The information content for depth recovery is concentrated in points
near the image edges, and those points are sparse and fall out of view
quickly. The filter can correctly track those features but it can't
tell whether they're 5 m away or 50 m away, so **scale is very weakly
observable**.

Over a 40 m drive that compounds into a ~7× scale error. The 290 m
path-length VIO reports is consistent with the filter believing each
metre travelled is actually ~7 m.

This is a textbook mono-VIO limitation, not a bug in OpenVINS. Stereo
VIO (or RGBD VIO) fixes it because depth becomes directly observable.

### 2. Init contaminated by the spawn drop

From OpenVINS' init log at startup:

```
[init]: successful initialization in 0.0003 seconds
[init]: orientation = 0.0105, 0.0000, 0.9999, 0.0000      ← (x, y, z, w) = 180° yaw
[init]: bias gyro  = -0.0001, -0.0013,  0.0000
[init]: velocity   =  0.0000,  0.0000,  0.0000
[init]: bias accel = -0.0001, -0.0000, -0.0066
```

Two things are wrong:

- **Init completed in 0.3 milliseconds.** That's not a measurement
  window — that's OpenVINS catching the IMU jerk from the rover's
  spawn drop (model spawns at `z=0.4`, settles around `z≈0.25`, with a
  brief impact transient on the wheels). The "static" init triggered
  on motion data, not on stationary data, so the gravity vector
  estimate is whatever direction the impact happened to push the IMU.
- **`orientation = (0.01, 0, 1.00, 0)`** in `(x, y, z, w)` quaternion
  order is a 180° rotation about Z — i.e. **OpenVINS thinks `global`
  is yawed 180° relative to the rover body at startup**. This is the
  *unobservable yaw* problem: IMU-only static init can recover pitch
  and roll from the gravity vector (gravity has a known direction in
  the world), but yaw is rotation around the gravity axis and gravity
  carries no information about it. OpenVINS picked an arbitrary value
  and happened to land on ~180° this run. Next run it might be 35°
  or −90°.

End result: drive forward → robot moves world +X → VIO reports motion
in `-X_global`. The green RViz trail going the "wrong direction" is
this yaw offset, **not** a code bug.

### 3. Cave environment is texture-poor

SubT cave walls have *some* texture (cracks, gradient shading) but are
much less feature-rich than a typical outdoor or indoor scene. Combined
with the 320×240 image at 30 Hz and OpenVINS' default `fast_threshold:
20`, feature tracks get lost frequently. Between tracking events the
filter propagates on IMU only, and IMU bias drift compounds quickly.

## Confirming the failure mode

Three quick checks distinguish the mono-VIO failures above from
"VIO has a real bug":

1. **Path length ratio.** VIO/GT ≈ 7 → scale drift. If it were close to
   1.0, the issue would be pure rotation. If it were 0.5–2.0, it'd be
   moderate drift. 7 is the smoking gun.
2. **Final pose sign flip.** VIO `x = −164 m` while GT `x = +30 m` →
   the trajectory is in a rotated frame. Run `evo_ape -a` (Umeyama
   alignment) on the same bag and the aligned APE drops dramatically
   if it's *just* yaw. If aligned APE is also huge, then VIO has
   scale drift on top of the rotation (which is what's happening
   here).
3. **LIO comparison.** LIO's 0.64 m error over the same drive
   confirms the rover, the sim, the bridge, and the timing are all
   fine. The issue is specific to the VIO algorithm + this motion
   pattern + this camera.

## Mono-VIO tuning history (what's currently in the config)

Two categories of changes have been tried. **Init / timing fixes
stayed in** because they cured a real bug (init on spawn-drop impact).
**Front-end tracker tweaks were reverted** because they backfired —
see the next section.

| Category | Where | Knob | Default | Currently set to | Notes |
|---|---|---|---|---|---|
| Init / timing  | `vio.launch.py`         | `VIO_START_DELAY_SEC` | 0       | **8.0 s**   | Don't subscribe to `/imu` until spawn drop settles. |
| Init / timing  | `estimator_config.yaml` | `init_window_time`    | 2.0 s   | **5.0 s**   | Average over more stationary samples. |
| Init / timing  | `estimator_config.yaml` | `init_imu_thresh`     | 1.5     | **0.3**     | Match our low-noise gz IMU. |
| Init / timing  | `estimator_config.yaml` | `init_dyn_use`        | false   | **true**    | Dynamic-init fallback if static fails. |
| Sim has exact calibration | `estimator_config.yaml` | `calib_cam_*`         | true    | **false**   | Don't refine SDF-exact calibration online. |
| Stationary drift | `estimator_config.yaml` | `try_zupt`           | false   | **true**    | Required to prevent bias drift while idle. |
| Front-end (REVERTED) | `estimator_config.yaml` | `histogram_method` | HISTOGRAM | HISTOGRAM | Was CLAHE; reverted — see below. |
| Front-end (REVERTED) | `estimator_config.yaml` | `num_pts` | 200 | 200 | Was 400; reverted. |
| Front-end (REVERTED) | `estimator_config.yaml` | `fast_threshold` | 20 | 20 | Was 10; reverted. |
| Front-end (REVERTED) | `estimator_config.yaml` | `min_px_dist` | 10 | 10 | Was 7; reverted. |
| Front-end (REVERTED) | `estimator_config.yaml` | `track_frequency` | 21.0 Hz | 21.0 Hz | Was 30 Hz; reverted. |

## The CLAHE / aggressive-tracker backfire (case study #2)

After the first analysis I tuned the OpenVINS front-end for "low-light
cave" — CLAHE, more features, lower `fast_threshold`. On the v2 bag
(`run_20260514T162146Z`), VIO got **much worse**:

|              | v1 (forward drive)         | v2 (figure-8 + stops)     | Δ          |
|--------------|----------------------------|---------------------------|------------|
| GT path      | 40.27 m                    | 58.52 m                   | longer     |
| LIO end-err  | 0.64 m  (1.6 %)            | 0.15 m  (0.3 %)           | **✅ better** |
| LIO ratio    | 1.01                       | 1.01                      | unchanged  |
| **VIO end-err** | 195.28 m (485 %)        | **4030.04 m (6886 %)**    | **❌ 20× worse** |
| **VIO ratio**   | 7.21                    | **378.80**                | **❌ 52× worse** |

VIO reported **22.2 km** of motion on a 58 m drive — the green trail
exploded out to `(+1073, +1948, −3360) m`.

**Why:** real-camera tuning assumes real-camera characteristics
(rolling shutter, vignetting, dim corners, structured noise). Our gz
camera is *artificially clean*: uniform lighting, no rolling shutter,
only the small σ=0.01 Gaussian noise floor we added in the SDF. The
combination of CLAHE (amplifies local contrast = amplifies that
Gaussian noise into pseudo-texture), `fast_threshold: 10` (accepts the
amplified noise as corners), and `num_pts: 400` (chases all of them)
fed the filter a flood of false feature observations. The tracker
locked onto ghosts; triangulation produced wildly wrong 3D positions;
the filter integrated the resulting "motion."

The lesson: **don't transplant real-camera tuning onto sim data
without thinking about how the image content differs.** Our sim image
is *easier* than real-world, not harder. Conservative defaults work
better.

If you ever switch to:
- a *real* RealSense camera, or
- a textured/dim outdoor Fuel world with more realistic image noise,

then flipping back to CLAHE + 400 pts + threshold 10 is the right
move. For the current gz cave sim, defaults win.

## Case study 3 — 640×480 + revert: mono VIO back to baseline

Bag `runs/run_20260514T182108Z/`. Same drive profile as v2 (stationary
→ slow forward → stop → figure-8 → stop). Two config changes since v2:

- All five front-end tracker tweaks reverted to upstream defaults
  (the v2 backfire diagnosed in [case study 2](#case-study-2--clahe-aggressive-tracker-backfire)).
- Camera resolution bumped 320 × 240 → 640 × 480 (intrinsics
  rescaled in `model.sdf` + `kalibr_imucam_chain.yaml`).

### Results

```
GT  end (-5.38, -2.04, -0.01) m,  path 51.28 m
LIO end (-5.42, -1.92, +0.10) m,  path 51.78 m,  |err| =   0.16 m  (0.3 %)
VIO end (-116.56, -86.07, +10.41) m,  path 276.76 m,  |err| = 139.75 m  (272 %)
```

| | v1 (320×240, no init fix) | v2 (CLAHE + aggressive) | **v3 (640×480, defaults)** |
|---|---:|---:|---:|
| VIO path-length ratio | 7.21 | 378 | **5.40** |
| VIO end-position error | 195 m | 4030 m | **140 m** |
| LIO ratio | 1.01 | 1.01 | 1.01 |
| LIO end-error | 0.64 m | 0.15 m | 0.16 m |

VIO is **70× better than v2** (the tracker revert undid the
catastrophic CLAHE/aggressive failure) and slightly better than the
v1 baseline (the resolution bump and init/timing fixes are helping).
But VIO still has 5.4× scale drift and 140 m of end-position error on
a 51 m drive. Mono VIO in this scene is not yet usable.

### What the plots show

**`trajectory_xy.png` (top-down)**: GT and LIO are nearly on top of
each other in a tiny ~10 m region near the origin. VIO's green trail
sweeps smoothly south-west out to (−116, −86) m — a graceful curve,
not noisy chaos. The smoothness is important: it means the filter is
*confident in a wrong answer*, not flailing. That's a hallmark of a
biased estimate (constant residual being integrated), not a tracker
problem.

**`trajectory_xyz.png` (per-axis vs time)**:
- X: GT/LIO oscillate between −5 and +20 m. VIO monotonically
  decreasing to −116 m.
- Y: GT/LIO near zero. VIO sliding down to −85 m.
- **Z: this is the diagnostic.** GT and LIO stay at z=0 for the
  entire run (rover is on flat ground). VIO's z dips smoothly down
  to −5 m around t = 0.25, then rises back through 0 and continues
  up to +10 m by the end. That smooth parabolic-ish curve is exactly
  the "constant residual acceleration" pattern the user described —
  there's still some small gravity-direction error that the filter
  keeps integrating into position. With ZUPT on the velocity should
  be clamped at the start, but as soon as motion begins the residual
  starts compounding.

The Z-curve magnitude (~15 m vertical excursion over a horizontal
~10 m drive) is much smaller than v2's vertical excursion (3360 m)
but much bigger than would be tolerable. The bias is *much* reduced,
not eliminated.

### Why VIO still drifts — two compounding causes on top of parallax

Looking at the actual camera frames in the bag (`camera_frames/`):

1. **The tunnel walls have a regular hexagonal lattice pattern.**
   Every grid intersection looks identical to every other. Mono VIO
   can lose features (e.g. when the camera rotates or a feature
   leaves the FOV) and re-acquire a *different* feature that the
   tracker / descriptor thinks is the same one. The result is a
   correspondence error → wrong relative pose → drift accumulates.
   KLT is mostly robust to this *frame-to-frame* (small patches,
   small motion), but over a long traverse the cumulative effect
   shows up.

2. **The scene is dim** in the deep tunnel sections. Lower
   signal-to-noise on the feature corners → noisier tracking → bigger
   per-step error → more accumulated drift.

3. (And of course) **forward driving is parallax-starved** — see the
   [Aside on parallax](#aside-what-is-parallax-and-why-is-forward-driving-the-worst-case)
   for why the optical-axis direction is a depth blind spot.

### What's next, ranked

1. **Try the same drive profile in the `cave` world** instead of
   `tunnel`. The cave has rough rock walls with *random*, *non-repeating*
   texture and a more varied geometry. That removes the self-similar
   pattern problem and leaves only the parallax problem to fight.
   This is a one-flag change to the launch (`world:=cave`) — no code
   work. Result will tell you whether the residual drift is from the
   scene or from VIO fundamentals.
2. **Drive with much more yaw than v3 did.** v3 was still mostly
   forward-driving with intermittent figure-8s. A sustained
   yaw-while-driving (curving constantly, never going straight) feeds
   mono VIO continuous parallax. Tighten the figure-8s.
3. **Stereo VIO.** Still the real cure for everything in this
   section. Two `rs_*` cameras already on the robot; one extra
   `cam1` block in `kalibr_imucam_chain.yaml` + `use_stereo: true` +
   `max_cameras: 2`.
4. **RGBD VIO** (e.g. Kimera, which can ingest depth). `/rs_front/depth`
   is already published — no SDF change needed. A different VIO
   submodule, but no second camera required.

## Case study 4 — out-of-the-tunnel circle: mono VIO finally works

Bag `runs/run_20260514T184552Z/`. User drove a teardrop loop *outside*
the tunnel (in the staging area / wooden-shed environment) for 60 s.
Two intentional differences from v3:

- **No more lattice texture** — wooden walls, concrete ground, varied
  outdoor textures. The hex-grid ambiguity from case study 3 is gone.
- **Sustained yaw** — the rover never drove straight for more than a
  second or two. Continuous curving = continuous parallax.

### Numbers — biggest VIO improvement so far

```
GT  end (-4.29, -1.04, -0.01) m,  path 17.89 m
LIO end (-4.30, -1.04, +0.08) m,  err 0.09 m  (0.5 %)   ratio 1.00
VIO end (+13.77, +8.07, +0.16) m, err 20.22 m (113 %)   ratio 1.62
```

VIO path-length ratio dropped from v3's 5.40 → **1.62** — by far the
closest mono VIO has gotten to GT in this codebase.

| | v1 | v2 | v3 | **v4** |
|---|---:|---:|---:|---:|
| VIO ratio | 7.21 | 378 | 5.40 | **1.62** |
| VIO end-error | 195 m | 4030 m | 140 m | **20 m** |

### The XY plot tells the rest of the story

GT and LIO are **perfectly overlapping** (LIO is 0.09 m off GT —
indistinguishable on a 17.89 m trajectory plot). They trace a clean
teardrop loop down-and-left to (−9, −4) and back.

VIO follows GT for the first ~3 m of straight forward driving, then
**diverges right at the start of the curve** and goes the *opposite
direction*: while GT loops down-left, VIO sweeps up-right to (+14, +8).
The shapes are similarly sized and similarly shaped — they're just
reflected through the origin.

This is the **unobservable-yaw signature** showing up undisguised, now
that the scene and parallax problems are out of the way.

### Proving it with `evo_ape -a` (rotation removed)

The user's intuition — "VIO is going in the opposite direction, like
some axis is wrong" — is provable. evo's Umeyama alignment rotates
the VIO trajectory onto GT before computing the error, which subtracts
out any constant rotation between their world frames. If VIO's
trajectory *shape* is correct, aligned-APE will be much smaller than
raw APE. If there's a genuine bug, alignment won't help.

```
                 RMSE     max      meaning
─────────────  ──────  ──────  ───────────────────────────────────────
Raw            17.80 m 24.65 m  apparent VIO error
Aligned (-a)    4.25 m  7.53 m  ← 76 % of error was just yaw rotation
Aligned+scale  1.63 m  3.46 m  ← further 62 % was scale drift
LIO (raw)       0.11 m  0.17 m  reference
(-as)
```

**76 % of the apparent VIO error is purely the wrong initial yaw.**
Once evo rotates VIO's `global` frame onto GT's, error drops by ~4×
without changing a single VIO sample — just relabelling the frame.
After also aligning scale, the residual 1.63 m on a 17.89 m drive
(9 % "true" shape error) is the actual filter performance.

### Why mono VIO has unobservable yaw — and what to do about it

It's not a config bug; it's a fundamental property of IMU-only static
init:

- **Gravity** (a vertical vector) constrains pitch and roll. OpenVINS
  gets these right — that's why the rover stays at z=0 (with some
  drift) instead of pitching into the ground.
- **Yaw** is rotation *around* the gravity axis. Gravity carries zero
  information about it. The IMU at rest produces the same gyro and
  accel readings regardless of which way the rover is facing in the
  world.
- OpenVINS' static init has to pick *some* yaw value to bootstrap.
  Without an external heading reference, it picks based on the first
  IMU readings + the propagated state — effectively random w.r.t. the
  world.

We've seen this consistently across runs:

- v1 init log: `orientation = (0.0105, 0.0000, 0.9999, 0.0000)` ←
  180° yaw rotation
- v4: trajectory mirrored through origin (also ~180° rotation)
- v3: trajectory rotated by some other angle

Mono-VIO yaw drift relative to truth **is unavoidable without an
external heading source**.

### Fixes that resolve this for good (none applied yet)

In rough order of effort vs payoff:

1. **Magnetometer fusion** — `/magnetometer` (`sensor_msgs/MagneticField`)
   is already being bridged out of gz. The Earth's magnetic field gives
   you a heading reference. OpenVINS doesn't support magnetometer
   fusion directly, but:
   - `robot_localization`'s `ekf_node` can fuse `/imu` + `/magnetometer`
     + `/ov_msckf/odomimu` (or any other odometry) and produce a
     yaw-anchored estimate.
   - Or fork OpenVINS' init to accept an external yaw prior on
     bootstrap.
2. **Stereo VIO** — two cameras separated by a baseline get parallax
   even *without* camera motion, so rotation becomes observable in the
   very first frame. Yaw is no longer ambiguous. This is the "real fix"
   for everything we've been chasing.
3. **Known-heading prior** — if the rover always spawns facing world
   +X (which it does in our launch), feeding "yaw = 0" as an init
   constraint nails the yaw deterministically. Not exposed as an
   OpenVINS config knob; would require a small patch to its init code.
   Quick hack.
4. **Loop closure** — if the rover returns to a previously-seen place
   and OpenVINS' DBoW2 loop closure fires, the yaw error gets corrected
   globally. OpenVINS has `/ov_msckf/loop_*` topics — they're enabled
   but only fire when revisits happen.

The other consistent finding from v4: **the trajectory shape, once
yaw-aligned, is much better than the raw numbers suggest**. Mono VIO
on this rig is not broken — it's just missing one observable, and the
remaining drift is in the realm of what's normal for mono.

## Case study 5 — stereo OpenVINS doesn't magically fix things

Bag `runs/run_20260514T210126Z/`. Same outdoor-circle drive profile as
v4 (so direct mono-vs-stereo comparison), but with the new
`rs_front_right` camera enabled and OpenVINS in stereo mode
(`use_stereo: true`, `max_cameras: 2`).

### Expected: ~91 % error reduction. Actual: slight regression.

```
APE          v4 mono     v5 stereo    Δ
─────────  ──────────  ────────────   ───────────
Raw RMSE    17.80 m     20.76 m       slightly worse
Aligned-a    4.25 m      9.89 m       2.3× worse
Aligned-as   1.63 m      2.18 m       slightly worse
LIO ref      0.11 m      0.20 m       sim itself slower
```

Two compounding causes:

### Cause 1: bandwidth-limited sim

Adding a second 640×480@30 RGBD sensor through the gz→ROS bridge
crushed the real-time factor. Topic rates from the v5 bag:

| Topic                    | v4 (mono) | v5 (stereo) | Effect                              |
|--------------------------|----------:|------------:|-------------------------------------|
| `/rs_front/image`        | 10.55 Hz  | **6.43 Hz** | 40 % drop — VIO has fewer frames    |
| `/imu`                   | 86.55 Hz  | **53.02 Hz** | 40 % drop — filter step rate down   |
| `/Odometry` (LIO)        | 4.85 Hz   | **1.96 Hz**  | 60 % drop — LIO scans slower        |
| `/ground_truth/pose`     | 51.38 Hz  | **44.19 Hz** | sim clock itself is slower          |

This alone accounts for the LIO regression (0.09 m → 0.13 m) and a
big chunk of the VIO shape regression. **Stereo VIO with this camera
load is bandwidth-bound on this hardware.**

Fixes to try (in order):
- Drop camera resolution back to 320×240 for stereo (4× less per-frame
  bandwidth). With sub-pixel localisation no longer the bottleneck, this
  is a fair trade.
- Drop camera rate from 30 → 15 Hz. Halves CPU/GPU, costs temporal
  resolution.
- Drop one of the cameras' depth or points topic — VIO only needs
  image + camera_info, not depth/points.

### Cause 2: static init still fired (the real surprise)

OpenVINS init log from the v5 run:

```
[run_subscribe_msckf-12] subscribing to cam (stereo): /rs_front/image
[run_subscribe_msckf-12] subscribing to cam (stereo): /rs_front_right/image
[run_subscribe_msckf-12] [init]: successful initialization in 0.0013 seconds
[run_subscribe_msckf-12] [init]: orientation = -0.0104, 0.0003, -0.9999, 0.0000
```

Stereo *is* engaged (both cameras subscribed). But OpenVINS' init
priority is *static first, dynamic only as fallback* — and static init
**uses only the IMU**. It can determine pitch and roll from gravity,
but not yaw. With static init firing in 1.3 ms, the yaw was decided
before either camera contributed anything. Result: same 180° yaw
flip we saw in v1, v3, v4.

The whole point of going stereo was to fix yaw via parallax. To
actually realise that benefit, dynamic init has to run instead of
static. The way to force that: make `init_imu_thresh` so high that
static init never triggers, then dynamic init (with `init_dyn_use:
true`, already set) takes over and uses stereo features to bootstrap
orientation.

**Fix shipped:** `init_imu_thresh: 0.3 → 50.0` in
`config/openvins/estimator_config.yaml`. With this set, static init
won't trigger short of literally dropping the rover. Dynamic init
takes over and stereo's contribution becomes observable.

### What v5 told us, even if the numbers regressed

- **The stereo wiring works.** Both cameras are subscribed, the filter
  is computing stereo updates. Plumbing-wise, Step 1 is done.
- **`aligned -as` regressed from 1.63 m to 2.18 m.** This is the
  "true shape" error after subtracting both yaw and scale, so it's
  the *honest* trajectory accuracy. It got slightly worse — almost
  certainly because the sim ran at lower RTF, not because stereo is
  actually worse than mono. Need v6 with reduced camera load to
  verify.
- **The yaw problem isn't a sensor problem, it's a config problem.**
  Now that we know static init was firing, the `init_imu_thresh: 50`
  fix should let stereo do its job.

Next bag with these changes should look much more like the expected
"stereo VIO is 5× better than mono."

## Case study 6 — stereo + dynamic init is actually the answer

Bag `runs/run_20260514T212506Z/`. Same outdoor-circle drive as v4/v5
but with the v5 case-study fix shipped: `init_imu_thresh: 0.3 → 50.0`
(forces dynamic init by making the static-init variance threshold
unreachable in normal operation).

### Numbers — first time mono-to-stereo gave the promised win

```
APE              v4 mono   v5 stereo-static   v6 stereo-dynamic
─────────────  ──────────  ────────────────   ──────────────────
Raw RMSE        17.80 m         20.76 m            21.62 m
Aligned (-a)     4.25 m          9.89 m             0.48 m   ← 9× better than v4
Aligned-scale    1.63 m          2.18 m             0.47 m   ← 3.5× better than v4
Path ratio        1.62             3.59              1.17    ← basically GT
End-pos error    20.22 m         33.61 m             7.50 m
LIO ref RMSE     0.09 m           0.13 m             0.09 m   ← sim back to normal
```

**Aligned APE of 0.48 m on a 35.80 m drive is 1.3 %** — the same
ballpark as published stereo-VIO benchmarks. The fact that `-a` and
`-as` numbers are nearly identical (0.48 vs 0.47) is the smoking gun
that **scale is now observed correctly** by stereo. The remaining
7.50 m raw end-position error is almost entirely a constant yaw
offset between OpenVINS' `global` frame and gz world — exactly the
kind of residual we expected to be left with.

### ZUPT is now firing — second big improvement

The init log for v6 is full of:

```
[ZUPT]: passed disparity (0.027 < 0.500, 168 features)
[ZUPT]: passed disparity (0.028 < 0.500, 168 features)
[ZUPT]: passed disparity (0.027 < 0.500, 168 features)
...
```

Hundreds of these lines during stationary periods of the drive. ZUPT
is detecting stationary state from **168 stereo features** of disparity
evidence per frame, well below the `zupt_max_disparity: 0.5` threshold.
Compare to v1-v4 (mono) where ZUPT was *enabled* but rarely fired
because the mono front-end didn't produce enough disparity evidence to
satisfy the gate. **Stereo gives ZUPT the data it needs to actually
work** — this fixes the "trajectory grows while stationary" problem
that dominated case studies 1, 2, and 3.

### Bandwidth status

```
Topic             Target    v5         v6        % of target
/rs_front/image   30 Hz     6.43 Hz    8.04 Hz   ~27 %
/imu              250 Hz   53 Hz      66 Hz      ~27 %
/Odometry (LIO)   ~15 Hz    1.96 Hz    3.39 Hz   ~23 %
/ground_truth/pose 60 Hz   44 Hz      51 Hz      ~85 %
```

The sim is running at roughly **27 % real-time** with the stereo
camera load. That's a steep cost for stereo, but v6 shows it's worth
paying. Future optimisations if RTF becomes a problem (none of these
are needed right now):

- Drop both stereo cameras back to 320×240 (4× less per-frame
  bandwidth, marginal accuracy loss given v6 is already at 1.3 % aligned drift).
- Drop SDF `<update_rate>` from 30 → 15 Hz on both cameras.
- Strip the depth + points bridge entries for rs_front and rs_front_right
  (VIO only consumes `/image` + `/camera_info`).
- Disable `rs_left`, `rs_right`, `rs_back` entirely (or just don't bridge them).

### What v6 means for the project

- Mono VIO is over for this codebase. Stereo is the production VIO
  configuration. Mono can be re-enabled by flipping `use_stereo:
  false`, `max_cameras: 1`, `init_imu_thresh: 0.3` — but there's no
  reason to.
- The remaining 7.50 m raw end-error is the unobservable initial yaw
  in OpenVINS' world frame. Resolving it cleanly would need either
  magnetometer fusion or a known-heading prior. Until then, RViz
  visual trails will look "rotated" relative to GT, but `evo_ape -a`
  numbers are honest.
- Step 2 (VINS-Fusion) and Step 3 (Kimera-VIO) now have a real
  baseline to beat: aligned APE of 0.48 m on a 35.80 m drive in this
  scene.

## Case study 3.5 — orientation jitter and the KLT-vs-descriptor question

While inspecting RViz during the v3 run, the user noticed the VIO Odom
display oscillating "very fast frequency" — three screenshots taken
within 7 seconds (`Screenshot from 2026-05-14 19-51-{08,12,15}.png`)
showed the same scene with `cam0` and `imu` axes at the same position
but pointing in clearly different directions.

The visual artefact is the **VIO Odom arrow display with `Keep: 200`**:
RViz overlays the last 200 messages on `/ov_msckf/odomimu`, each drawn
as a green arrow. OpenVINS publishes at the IMU rate (~75 Hz), so 200
arrows = the latest ~2.7 s of state estimate. When the *position* is
roughly stable but the *orientation* differs from message to message,
RViz renders the cluster as a chaotic forest of green cylinders
pointing in many directions. That forest is the user-visible signature
of **orientation jitter on the per-IMU-step timescale**.

### Why orientation jitters here

Looking at the camera frames in `runs/run_20260514T182108Z/`, the
tunnel walls have a regular hexagonal-lattice pattern. Two failure
modes combine:

1. **Correspondence ambiguity from self-similar texture.** KLT (which
   OpenVINS uses by default — `use_klt: true`) tracks each feature
   patch frame-to-frame by minimizing patch SSD. On a lattice where
   every grid intersection looks identical, the minimisation can
   converge to a *neighbouring* cell when a feature is partially
   occluded or when motion is large. Each false correspondence
   feeds an incorrect geometric constraint into the MSCKF update →
   the filter applies a small orientation correction → next IMU step
   it gets another bad constraint pointing slightly differently → the
   orientation oscillates.
2. **Weak rotation observability in forward-driving mono VIO.** Same
   reason the trail drifts overall: the optical-axis-aligned motion
   gives little parallax, and rotation around the gravity axis (yaw)
   is the weakest of all. Even *correct* correspondences carry less
   information about orientation here than they would for a side-
   looking camera or a yaw-heavy drive.

The combination is the user's "fast-frequency fluctuation."

### Fix #1 — tighten the camera-pixel noise model (applied)

`config/openvins/estimator_config.yaml`:

```yaml
up_msckf_sigma_px: 1  →  2
up_slam_sigma_px:  1  →  2
```

This tells the filter "trust each pixel observation less" — measurement
covariance doubles, MSCKF updates damp by ~4×. Trade-off: a *real*
fast rotation (e.g. the rover hitting a bump and yawing 5° in 50 ms)
will be tracked more slowly. For our rover with smooth 1 rad/s max
yaw, that's fine.

### Fix #2 — try ORB descriptors instead of KLT (proposed, not applied)

The user pointed out a directly relevant finding from
[`behnamasadi/OpenCVProjects/docs/kitti.ipynb`](https://github.com/behnamasadi/OpenCVProjects/blob/master/docs/kitti.ipynb):

> If you use `SIFT` run: `python kitti_vo_sift.py` (works well)
> or if you use `cv2.goodFeaturesToTrack` you will get poor results:
> `python kitti_vo.py`

That's the same KLT-vs-descriptor question for OpenVINS. The relevant
flag is **`use_klt`** in `estimator_config.yaml`:

| `use_klt: true` (current)         | `use_klt: false` (ORB + descriptor matching) |
|-----------------------------------|----------------------------------------------|
| Lucas-Kanade pyramidal optical flow on FAST corners. | FAST corners → ORB binary descriptor → brute-force kNN match with `knn_ratio: 0.70` test. |
| Very fast. Excellent for small frame-to-frame motion. | Slower (descriptor compute + match per frame). |
| Patch-based — robust to mild appearance change, but on self-similar texture can drift to a neighbouring identical-looking patch. | Descriptor-based + Lowe's ratio test: if the best match's distance is too close to the second-best's, the match is rejected. **On a lattice scene, ambiguous matches get filtered out automatically.** |
| What `cv2.goodFeaturesToTrack` + `cv2.calcOpticalFlowPyrLK` is doing in your KITTI notebook. | Equivalent to a stripped-down SIFT-style pipeline (ORB is faster than SIFT, similarly robust to lattice ambiguity because of the kNN-ratio filter). |

For our tunnel-lattice scene, ORB's automatic ambiguity rejection
should reduce the orientation-jitter rate substantially. Cost is
extra CPU per frame; with `num_pts: 200` it's well within budget.

If Fix #1 alone doesn't produce a clean RViz arrow track in v4, the
next experiment is to flip `use_klt: false` and re-run. That's a
single-line config change.

### Visual workaround (no code)

Independent of the underlying fix, you can clean up the RViz display:

- In RViz Displays panel → click **VIO Odom**
- Set **Keep** from `200` to `10`

That hides the long arrow tail. The current VIO arrow + most-recent
~0.1 s of history is enough to see where the filter "is right now,"
without the forest of stale orientations cluttering the view. The
jitter is still in the data — but the visualisation isn't dominated
by it.

## What's left to try on mono VIO

After the timing fixes and the tracker revert, the levers still on
the table for pure-mono are:

1. **Drive a more parallax-friendly profile.** Forward-straight is
   the worst-case for any mono VIO (no parallax along the optical
   axis). The v2 run already mixed in some yaw and got better LIO
   numbers — VIO would benefit too if it weren't being thrown off by
   the tracker chaos. With defaults restored, try v2's profile again.
2. **Longer stationary wait** before the first command (15–20 s),
   especially on bag start. The 8 s delay handles the spawn drop;
   another 5–10 s of stationary lets the filter accumulate clean
   static observations.
3. **Bump `max_clones`** (11 → 15) and **`max_slam`** (50 → 75) for a
   larger sliding window. Costs CPU but stabilises the filter when
   tracks die in the cave.
4. **Stereo VIO** is the real cure. The model has `rs_left` +
   `rs_right`. Adding a `cam1` block to `kalibr_imucam_chain.yaml`
   and flipping `use_stereo: true` + `max_cameras: 2` fixes the scale
   ambiguity for good.

## Comparing multiple VIO implementations side by side

Possible — and a natural next step once we're done squeezing OpenVINS.
The system architecture already supports it:

- `scripts/analyze_bag.py` reads odometry topics by name. To compare a
  second VIO (e.g. VINS-Fusion, Kimera-VIO, OKVIS2) the only changes
  are:
  1. Add the second VIO as a `third_party/` submodule + a small
     `<name>.launch.py`.
  2. Tell `analyze_bag.py` the new topic in its `ODOM_TOPICS` dict and
     give it a colour.
  3. Optionally: add a corresponding RViz display.
- Each VIO is just another process subscribed to `/imu` and
  `/rs_front/image`. They don't interfere with each other.
- `evo_traj bag2 ... /ground_truth/odom /ov_msckf/odomimu /vins/odom
  /kimera/odom --save_as_tum` compares all of them in one shot;
  `evo_res *.zip` gives a side-by-side APE table.

Candidates worth comparing at the mono-cam stage:

| Package        | Repo                                       | Strengths                                                            |
|----------------|--------------------------------------------|----------------------------------------------------------------------|
| **OpenVINS**   | rpng/open_vins (already wired)             | MSCKF, clean code, easy to debug.                                    |
| **VINS-Fusion**| HKUST-Aerial-Robotics/VINS-Fusion          | Tightly-coupled BA + loop closure. Often better on long sequences.   |
| **Kimera-VIO** | MIT-SPARK/Kimera-VIO                       | Built for SLAM + semantic mapping; needs slightly different config.  |
| **DLIOM/SVIN** | various                                    | Newer entrants worth tracking.                                       |

If you want to commit to this, the workflow is the same as for adding
LIO earlier: `git submodule add` under `third_party/`, write the
launch, add the topic to the analyzer.

## Accept that LIO wins on this rig

Cave + lidar + forward driving is LIO's home turf. VIO is here mostly
as a comparison target / sanity check / learning vehicle. If you're
trying to *use* the rover's pose estimate for navigation, fuse LIO
into wheel-odom with `robot_localization` and trust that; treat VIO
as the experimental loop.

---

The workflow above ([Workflow: record → analyse → compare](#workflow-record--analyse--compare))
supersedes a duplicate "Reproducing this analysis" section that used
to live here.
