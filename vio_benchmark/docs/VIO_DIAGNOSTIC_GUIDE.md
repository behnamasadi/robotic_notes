# VIO/LIO diagnostic guide

How to evaluate a visual-inertial (VIO) or LiDAR-inertial (LIO) odometry
estimator, and a structured procedure for finding the cause when it's
broken. Written from the case study of `runs/run_20260515T115223Z`
(scripted slow drive, 2026-05-15) where stereo OpenVINS and stereo
VINS-Fusion both blew up scale by 3.5–3.8× while FAST-LIO stayed close
to ground truth.

The guide has three parts:

1. **The metrics**: what numbers we actually look at and what they mean.
2. **The procedure**: a top-down checklist for diagnosing a bad estimator.
3. **Case study**: how the procedure pinned today's failure on the
   simulated IMU producing impulsive accelerations.

If you're new to evo / APE / RPE / Umeyama, read
[`ANALYSIS.md`](./ANALYSIS.md) first — it explains those concepts
ground-up. This guide builds on them.


## 1 — The metrics

Mostly two categories: **trajectory metrics** (was the estimator right
about where the robot went?) and **input-quality metrics** (was the
sensor data even sensible?). Many bugs hide in input-quality; many
estimator tuners only look at trajectory metrics and chase the wrong
problem.

### 1.1 — Trajectory metrics

#### Path length ratio

```
ratio = estimator_path_length / ground_truth_path_length
```

A healthy estimator gives a ratio close to 1.0. **The single most
useful first-look metric.** It reduces an entire trajectory to one
number and that number is geometry-meaningful:

- **ratio ≈ 1.0** — estimator's metric scale agrees with reality.
- **ratio > 1** — estimator over-shoots length. Common in
  monocular VIO that can't observe scale (forward driving in a tunnel,
  pure rotation). Also common when the camera-IMU extrinsics or IMU
  bias is off and the filter integrates phantom translation.
- **ratio < 1** — estimator under-shoots. Less common; usually
  comes from ZUPT (zero-velocity update) firing during real motion
  and clamping velocity to zero, or a feature tracker that drops
  most frames so the filter "skips" between widely-spaced poses.
- **ratio >> 1 (10+)** — usually an initialization failure followed
  by exponential drift. VINS-Fusion is notorious for this — if the
  init window doesn't see enough excitation it diverges.

Compute via `evo_traj`:

```bash
evo_traj bag2 <bag.mcap> --topic /Odometry --print
# look at the "path length" field
```

Or, in the project's tooling, the field labelled "path length [m]"
in `scripts/analyze_bag.py`'s `summary.md`.

#### Absolute Pose Error (APE)

The Euclidean distance between estimator and ground-truth poses,
*after* aligning them. Two flavours:

```bash
evo_ape bag2 <bag> <gt_topic> <est_topic>      # raw — no alignment
evo_ape bag2 <bag> <gt_topic> <est_topic> -a   # SE(3) alignment (R + t)
evo_ape bag2 <bag> <gt_topic> <est_topic> -va  # Umeyama (R + t + s)
```

What each tells you:

- **No alignment** — measures the literal drift in the estimator's
  reported world frame. If the static TF wiring is correct and the
  estimator inits with the right yaw, this is your real-world
  positioning error.
- **SE(3) alignment** — translation + rotation difference after
  optimally rotating/translating one trajectory onto the other.
  Removes the "wrong initial yaw" bug but keeps scale honest.
- **Umeyama (–va)** — also allows a global scale factor. **If APE
  drops dramatically going from `-a` to `-va`, you have a scale
  problem.** The scale factor evo reports tells you the ratio (close
  cousin of path length ratio above).

#### Relative Pose Error (RPE)

Same idea as APE but measured over fixed-length segments, e.g. every
1 m of travel:

```bash
evo_rpe bag2 <bag> <gt_topic> <est_topic> --delta 1 --delta_unit m
```

RPE catches drift *rate* rather than accumulated drift. Useful for
comparing estimators of different total path lengths — a 50 m
trajectory and a 500 m trajectory aren't comparable in APE but RPE
gives drift per meter.

For a typical "5–10 cm per m of travel" robot you want RPE around
that magnitude. RPE growing linearly with segment length → consistent
drift rate. RPE growing super-linearly → estimator is diverging.

#### End-position error / GT path length

```
err_fraction = ||p_est_end – p_gt_end|| / gt_path_length
```

A blunt summary, but one most people understand intuitively: "after
walking 50 m, the estimator thinks we're 5 m from the right place"
→ 10 %. The KITTI benchmark uses similar metrics ("translation error
[%]"). Anything under ~2 % on long drives is competitive; under 1 %
is excellent. >10 % is broken.

### 1.2 — Input-quality metrics

These are often skipped, and they're often the actual problem.

#### IMU at-rest sanity

When the robot is stationary on flat ground a healthy IMU reports:

```
|accel|  ≈ 9.81 m/s²              (gravity magnitude)
ax       ≈ 0
ay       ≈ 0
az       ≈ +9.81                  (when z-up convention)
gyro     ≈ 0   (all axes, modulo noise)
```

If gravity is split between two axes the IMU is mounted tilted or
its body frame doesn't match what you wrote in the YAML. If
|accel| is off by 5–10 %, the gravity_mag setting needs adjusting.
If az is negative, you have a sign-convention bug somewhere
(very common with REP-103 vs Z-down conventions).

#### IMU bandwidth and saturation

This is the input-quality check almost nobody does, and it's what
today's case study revealed:

```
max(|accel|) over the recording should be < ~30 m/s² for a wheeled
robot, < ~50 m/s² for an aggressively flown drone.
std(accel.x / .y / .z) should be a few m/s², not 10+.
```

If you see spikes >50 m/s², either you're crashing into things or
your IMU is bandwidth-unlimited and aliasing impulsive forces.
**Real MEMS IMUs are physically rate-limited.** Datasheet bandwidths
are typically 50–500 Hz. A sample published at 250 Hz that shows
a single 200 m/s² spike is impossible on real hardware; it's a sim
artefact.

Plot the per-axis accel over time and look for:
- Single-sample spikes (impossible on real IMU — bandwidth bug)
- Plateaus at the same value (sensor saturating at its range)
- Constant offset (un-modelled bias — VIO can usually handle this
  if it estimates bias online)

#### IMU rate consistency

Check that the IMU is publishing at its configured rate. If the SDF
says 250 Hz but you measure 215 Hz, the sim is running at
RTF = 215/250 = 0.86. That's fine in itself but it means:
- Your VIO is processing 14 % fewer IMU samples than it expects
- Pre-integration windows are still in *sim time*, so they
  contain fewer samples → noisier integration
- Time offsets between IMU and camera can drift over long runs

#### Camera rate and timestamp jitter

Same idea for cameras. If the SDF configures 30 Hz and the bag
captures 10 Hz, either:
- The sim isn't producing frames fast enough (RTF < 1, GPU not
  used, software rendering)
- The bag writer is dropping frames (default `keep_last: 10`
  reliable QoS subscriber queue overflows)
- The estimator is downsampling internally (VINS-Fusion's `freq`
  parameter)

Verify with `ros2 topic hz` *during recording* and compare to
the bag's effective rate (count / duration) afterwards.

Stereo synchronisation: left and right frame timestamps should
differ by less than ~1 ms for tight stereo matching. Most stereo
front-ends use approximate time sync with ~30 ms tolerance, but
beyond ~50 ms you start losing correspondences.

#### Calibration consistency

The intrinsics + distortion + camera-IMU extrinsics in your
estimator YAMLs must match the actual sensor producing the data.
In sim, that means matching the SDF. Cross-checks:

```
SDF <camera> width/height/fx/fy/cx/cy/distortion
  ↔ openvins/kalibr_imucam_chain.yaml intrinsics/distortion_coeffs
  ↔ vins/cam0.yaml + cam1.yaml projection_parameters/distortion_parameters

SDF <sensor> pose
  ↔ T_imu_cam in kalibr_imucam_chain.yaml (right after coord-frame R)
  ↔ body_T_cam in vins.yaml

SDF imu_sensor update_rate
  ↔ kalibr_imu_chain.yaml update_rate
  ↔ vins.yaml acc_n / gyr_n (continuous-time noise density)
```

A 0.01 m baseline mismatch in stereo extrinsics → ~10 % scale
error on triangulated depth → ~10 % path-length error. A 5°
rotation mismatch between camera and IMU → wrong gravity vector
in the filter → systematic horizontal drift.


## 2 — The diagnostic procedure

When an estimator is broken, run this checklist top-down. Each step
either confirms the issue or moves you down to the next level. **Don't
skip steps.** It's tempting to start with calibration when you see
scale drift, but that's the third step, not the first.

### Step 0 — Is the bag what you think it is?

```bash
python3 scripts/analyze_bag.py runs/<bag>
```

Look at the topic-rates table in `summary.md`. Verify:

- Both stereo cameras are present (if you expected stereo)
- IMU is at expected rate (or close, modulo RTF)
- Ground-truth is publishing
- All estimator output topics are present

**Today's gotcha:** in `runs/run_20260515T103115Z`, only
`/rs_front/image` was recorded — not `/rs_front_right/image`. The
"stereo VIO" in that bag was effectively mono. Always check the
topic list first.

### Step 1 — Is LIO healthy?

If you have a LiDAR-inertial estimator running in parallel, check
its trajectory first. **LIO failing usually means the simulation
itself is broken** (physics, TF tree, IMU completely garbled). VIO
failing while LIO succeeds means the problem is downstream of the
LiDAR — in the camera path or in the IMU's interaction with VIO
specifically.

In today's case: FAST-LIO end-point error 2.93 m on an 11 m drive,
path-length ratio 1.45×. Slight scale issue but the trajectory
shape is correct. **LIO works → physics + TF + LiDAR are OK.** That
ruled out a whole class of simulator-level bugs.

### Step 2 — Look at the trajectory shape

Open the `trajectory_xy.png` plot from `analyze_bag.py`. Compare
each estimator's path to GT, ignoring scale for a moment.

- **Same shape, different scale** → calibration baseline or
  intrinsics mismatch
- **Wrong direction** (e.g. estimator goes -x when GT goes +x) →
  initial yaw error — dynamic init picked the wrong rotation
- **Sign-flipped axis** (e.g. y inverted) → coordinate-convention
  mismatch (REP-103 vs other) or a left-handed/right-handed frame
  bug in the estimator config
- **Explodes off to thousands of metres** → divergence, typically
  from initialization failure
- **Walks off perpendicular to GT** → gravity vector wrong → filter
  thinks gravity is horizontal acceleration

Today's case study: OpenVINS ended at `(-6.25, -3.02)` while GT
ended at `(+6.77, +2.16)` — **rotated 180° around z**. VINS ended
at `(+2.57, -6.16)` — y-component sign-flipped. The two VIOs
disagree with each other, which rules out a shared static-TF bug
and points at dynamic initialisation producing different (wrong)
global-frame yaws.

### Step 3 — Compare scale ratios across drive speeds

If you have two recordings of the same scenario at different speeds
and the scale ratio is similar, **fast motion is NOT the cause.**
Common myth: "VIO blows up because the robot drove too fast."
Sometimes true (KLT feature tracker loses features when inter-frame
pixel displacement is huge) but only at very high speeds and low
frame rates. Below ~1 m/s and above ~20 Hz, motion speed is rarely
the limiting factor.

Today: fast drive in v9 → OpenVINS 5.18×. Scripted slow drive in
v10 → OpenVINS 3.55×. Same order of magnitude → speed isn't the
cause. (The reduction from 5.18 → 3.55 is real but consistent with
"shorter trajectory accumulates less drift", not "speed mattered".)

### Step 4 — Calibration consistency check

Read the SDF and every estimator's calibration YAML side-by-side.
Match each field. The list in §1.2 "Calibration consistency"
enumerates the cross-checks.

Most useful: **compute the implied stereo baseline two ways.**
- From the SDF: `||rs_front.pose.t – rs_front_right.pose.t||`
- From the YAML: `||T_imu_cam0[0:3,3] – T_imu_cam1[0:3,3]||`

If they disagree by N %, your scale ratio is off by N %. A 1 m
YAML baseline with 0.1 m real → 10× scale ratio.

Today: every field matched. Calibration ruled out.

### Step 5 — IMU sanity (NOW the smoking gun usually shows up)

Open `imu_accel.png` and `imu_gyro.png`. Mentally compare to the
expected motion profile.

- Stationary phases: |accel| flat at ~g, gyros at 0 (modulo noise)
- Forward driving at constant v: |accel| still ~g, gyros at 0
  (rover not turning)
- Turning: small gyro_z bump, accel essentially gravity
- Acceleration phase: small (1–2 m/s²) ax bump, then back to g

Anything dramatically off — single-sample spikes, plateau-clipping,
the gravity vector being on the wrong axis — is your bug.

Today: massive accel spikes (50 to >200 m/s², many g) at every
cmd_vel direction change. This is impossible for a real IMU and
broke both VIOs' integration. **This was the actual root cause.**

### Step 6 — Read the estimator's own runtime log

If steps 0–5 haven't pinned it, the estimator may be telling you
exactly what's wrong if you look. For OpenVINS, that's
`/proc/<pid>/fd/1` while it runs, or whatever your launch captured
to a log file. For VINS-Fusion, similar.

Look for:
- The init banner (does it say it initialised in stereo? mono?
  static? dynamic?)
- Feature counts per frame (is the tracker losing features?)
- ZUPT acceptances during motion (zero-velocity update firing
  when the robot is actually moving = critical bug)
- Filter divergence warnings
- chi² test failures on measurement updates

Today (post-mortem): the OpenVINS stdout while the rover sat still
shows `[ZUPT]: accepted |v_IinG| = 0.210` constantly. That 0.21 m/s
"phantom velocity" being slammed to zero every frame implies the
filter's velocity estimate is drifting between updates — consistent
with the IMU-spike hypothesis.


## 3 — Case study: today's diagnosis

The sequence above, applied to the slow-drive control bag.

| step | check | finding |
|---|---|---|
| 0 | bag has both cameras + IMU + GT? | yes — `/rs_front/image`, `/rs_front_right/image`, `/imu`, `/ground_truth/path` all present at expected rates |
| 1 | LIO healthy? | end-point error 2.93 m / 11 m = 26 % — not perfect (1.45× scale) but trajectory shape matches GT. Physics + TF + LiDAR OK. |
| 2 | VIO trajectory shape? | OpenVINS rotated 180° in yaw, VINS sign-flipped on y. Estimators disagree with each other and with GT → dynamic init / IMU integration problem, not a shared wiring bug. |
| 3 | scale ratio fast vs slow? | v9 fast: 5.18×. v10 slow: 3.55×. Same order — motion speed isn't the cause. |
| 4 | calibration cross-check (SDF ↔ both YAMLs)? | every field matches: intrinsics 554.26 / 320.5 / 240.5, distortion -0.02 / 0.003 / 0 / 0, baseline 0.10 m, T_imu_cam rotation matches REP-103 / optical conversion. NOT the cause. |
| 5 | IMU plot | **smoking gun**: 50–200 m/s² spikes on every cmd_vel direction change. Stationary phases clean (|accel|≈g, ax≈0, az≈g). Motion phases have impulsive accels impossible on real hardware. |
| 6 | OpenVINS runtime log | constant ZUPT acceptances at |v_IinG|=0.21 m/s — the filter thinks it's moving 0.21 m/s when it isn't, then clamps to zero, every frame. Consistent with bad IMU integration between updates. |

### Why the spikes happen

The gz-sim IMU sensor reads the **instantaneous acceleration** that
the physics solver produces. The `gz-sim-diff-drive` plugin
implements wheel velocity control by stepping the target speeds
on each `/cmd_vel` message. When the target velocity changes
suddenly (forward → reverse, straight → arc), the wheel angular
velocities jump → tyre-ground contact constraints generate large
impulsive forces → those forces appear in the IMU as huge accels.

Real IMUs are physically bandwidth-limited — internal anti-alias
filters and MEMS resonant dynamics smear those impulses across
many samples and clip them at the sensor's full-scale range
(typically ±16 g for cheap parts, ±200 g for high-range industrial
units). The simulated IMU has neither model: it captures the
physics-solver impulse as a single-sample value with no smearing
and no clipping.

### The fix (proposed, not yet committed)

A small ROS 2 node:

```python
# imu_lowpass_node.py — subscribes /imu, applies a 1-pole IIR
# low-pass at ~80 Hz cutoff, republishes /imu_filtered. The cutoff
# is chosen to match a Bosch BMI088-class MEMS accelerometer
# bandwidth and to stay well above the rover's mechanical dynamics
# (~5–10 Hz for a 1 m² robot on rubber wheels).
```

Then point OpenVINS, VINS-Fusion, FAST-LIO at `/imu_filtered`
instead of `/imu`. Re-record the slow drive and check if the
spikes are gone from the plot and whether the VIO trajectories
recover.

Alternative (more invasive but cleaner): write a custom gz-sim
IMU plugin that includes a bandwidth model. Replaces the stock
`gz-sim-imu-system`.

The user-side experience after the fix should be: **VIO scale
ratio drops to ~1.0, LIO unchanged.** If the ratio is still off
after low-pass filtering, the diagnosis was wrong and we go back
to step 5 with the new data.


## 4 — Reference: the tools we used today

| tool | what for |
|---|---|
| `scripts/analyze_bag.py` | One-shot trajectory + IMU summary from any rosbag |
| `scripts/recording_health.sh` | Live 10-second pre-recording sanity check (cameras / GPU / stereo wiring) |
| `scripts/perf_probe.sh` | CPU / GPU / topic-rate snapshot during a live sim |
| `scripts/scripted_drive.py` | Deterministic open-loop drive — repeatable VIO control experiments |
| `nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv,noheader -lms 1000` | GPU-side check (is gz-sim actually using the GPU?) |
| `eglinfo` inside container | Confirm EGL vendor (NVIDIA vs Mesa/llvmpipe) |
| `evo_ape`, `evo_rpe`, `evo_traj` | The standard trajectory metrics |
| `head -c N /proc/<pid>/fd/1` inside container | Snapshot a running process's stdout when no logfile exists |
| `ros2 topic hz /imu` | Verify rate matches SDF nominal × RTF |
| `python3 -c "import yaml; …"` over `metadata.yaml` | Parse a rosbag's effective topic rates without opening the mcap |


## 4b — Why LIO survives bad IMU data and VIO doesn't

A puzzling observation from this project: **on the exact same gz-sim IMU
stream**, FAST-LIO produced clean trajectories (0.3–1.5 % end-point error)
while OpenVINS and VINS-Fusion blew up scale by 3–10×. Same impulses,
opposite outcomes. This is worth understanding because the answer
determines what you can and can't get away with in sim.

### The IMU plays a fundamentally different role in each

**In VIO**, the IMU is the **scale and velocity sensor.** Between two
camera frames (33 ms apart at 30 Hz), the filter integrates the IMU
forward — typically ~250 samples per camera interval — to propagate
position, velocity, and orientation. Then the camera observation comes
in and weakly corrects via triangulated features. The cameras can
constrain depth (via stereo baseline) and orientation (via feature
flow) but **velocity has to come from IMU integration**.

If a spike of 200 m/s² hits the IMU for one 4 ms sample, that's
0.8 m/s of phantom velocity error injected per spike. Over a 33 ms
camera frame, that integrates to ~13 mm of position error. Across a
187 s recording at 30 Hz, you accumulate ~73 m of drift — exactly the
order of magnitude we measured for our 3–10× scale blow-ups.

The camera-side correction can fight this, *if* the filter's IMU noise
model accurately reflects the noise. A standard EKF/MSCKF assumes
IMU noise is small Gaussian. When the actual noise is impulsive, the
filter's covariance estimates are wildly wrong, and the camera
correction becomes too weak to drag the state back to truth.

**In LIO** (FAST-LIO specifically), the IMU is **just an initial-guess
provider** for ICP. Each LiDAR scan returns tens of thousands of
points; scan-to-scan ICP directly observes the relative motion
between scans via geometric correspondences. Even if the IMU says
"I moved 1 m forward in the last 0.1 s" but you actually moved 0.1 m,
ICP looks at the two point clouds, sees they overlap after a 0.1 m
shift, and corrects to that. The IMU's bad estimate makes ICP take
more iterations to converge, but the *final* answer comes from the
geometric registration, not from the IMU integration.

### The structural difference

```
VIO :  IMU integration  →  state  →  camera CORRECTS state
        ^^^^^^^^^^^^^^^^^^^^^^^^^^
        impulses live here; correction is weak

LIO :  IMU integration  →  initial guess  →  ICP REPLACES state
                                              ^^^^^^^^^^^^^^^^^
                                              geometry, not IMU-derived
```

In VIO, the IMU is on the *integrator path* — its errors compound
over time and the camera fight them with limited authority. In LIO,
the IMU is on the *hint path* — its errors get thrown away every scan
because ICP doesn't trust the IMU for absolute position.

### Practical consequences

- **Sim recordings are usable for LIO comparison** — gz-sim's IMU
  artefacts are warm-start noise to FAST-LIO, not a position-error
  source.
- **Sim recordings are NOT usable for VIO comparison** — the IMU
  artefacts compound through integration and produce trajectories
  that no real-world equivalent would generate. VIO results in our
  sim consistently 3-10× the real-world equivalent.
- **This is also why ORB-SLAM3 "explodes" when IMU goes bad.** Its
  inertial mode integrates the IMU in the same architectural role as
  OpenVINS / VINS-Fusion; bad IMU → bad propagation → factor graph
  diverges. Confirmed by experience reports from this project.
- **Pure visual SLAM (no IMU)** is yet another regime: ORB-SLAM3 in
  vision-only mode would be unaffected by IMU artefacts but loses the
  scale anchor that IMU provides. Useful as a control experiment.


## 5 — On trusting simulated IMU data

A separate but related question this guide should address: **is gz-sim's
IMU good enough for VIO development?** The answer is "partly", and the
parts it gets wrong are exactly the parts that break VIO. Knowing
this up front saves a lot of time.

### What gz-sim's IMU models

The `imu` sensor in Gazebo Harmonic reads the rigid body's instantaneous
acceleration and angular velocity from the physics solver and adds
configurable Gaussian noise + bias random walk. That's all.

### What gz-sim's IMU does NOT model

Real MEMS IMUs have a mechanical bandwidth (proof mass + suspension acts
as a 2nd-order low-pass filter, typically 100–500 Hz cutoff), an
anti-alias filter before the ADC, hard saturation at the full-scale range
(±16 g for cheap parts, ±200 g for industrial), and quantization at
16 bits. None of these are in the gz-sim sensor — its accel output is
effectively a full-precision float of the *instantaneous* physics state.

### Why this matters

Two consequences worth internalising:

**1. The simulator can produce IMU signals that no real device would
ever produce.** Any impulsive force in the physics solver appears in the
IMU as a single-sample spike of arbitrary magnitude. Today's case
study: 200 m/s² (~20 g) spikes from instantaneous wheel-velocity steps
in the DiffDrive plugin. A real BMI088 would clip that to ±16 g and
smear it across ~10 samples through its 145 Hz internal filter.
**A VIO tuned on real IMUs will choke on sim IMUs that produce these
signals.** The usual concern about sim-to-real is "sim is too easy"; here
it's the opposite — sim is *unrealistically harsh* in one specific way.

**2. The physics is also too "perfect"** in the sense that there's no
motor dynamics, no gearbox compliance, no wheel-ground compliance. Real
robots smear acceleration commands across many ms. The
`gz-sim-diff-drive` plugin steps wheel velocities instantly when
`/cmd_vel` arrives, which is what causes the impulsive forces in the
first place. So today's bug is two layers of unrealism stacked: physics
producing impulses that wouldn't happen on real hardware, *and* the IMU
sensor passing those impulses through without the bandwidth limiting
that would absorb them on real hardware.

### How the community actually develops VIO

Look at any VIO paper from the last 5 years (OpenVINS, VINS-Fusion,
Kimera, ORB-SLAM3, MSCKF, ROVIO). Benchmark numbers are reported
against **real-world datasets** — EuRoC MAV, TUM-VIO, KITTI,
NTU-VIRAL — not against simulation. Simulation is used for:

- *Algorithm correctness* — "does my filter converge under clean data?"
  Easy to check with perfect-IMU sim.
- *Stress testing* — "what if I add N% calibration error?" Inject the
  error, see what breaks.
- *Ground-truth-rich debugging* — when something fails on real data,
  reproduce it in sim with GT to isolate algorithm-vs-data.

What simulation is **not** trusted for: deciding whether a VIO will work
on a specific physical platform. Only real-hardware testing answers
that.

### Implications for this project

For comparing OpenVINS / VINS-Fusion / FAST-LIO / Kimera in our sim:
- The comparison is **only meaningful if the simulated IMU is realistic
  enough** that the differences come from algorithm tradeoffs, not from
  one estimator being more robust to sim artefacts.
- The path forward is one (or both) of:
  - Smooth the input side: a `cmd_vel_smoother` between teleop and
    the rover so wheel commands ramp instead of step.
  - Smooth the output side: a small `imu_lowpass` republisher node
    that turns `/imu` into `/imu_filtered` with a ~80 Hz cutoff
    matching a real MEMS bandwidth. Point all estimators at the
    filtered topic.

Both are valid for algorithm comparison. Neither is a substitute for
running the same VIO on EuRoC or a real recording when you want
real-world performance numbers.

### Bottom line

Yes, simulation is a legitimate VIO development tool — but only after
you've verified the simulated IMU produces signals consistent with
real hardware. If you skip that check, you'll spend hours debugging
algorithms when the problem is the sensor model. The diagnostic
procedure in §2 of this guide explicitly puts IMU-quality (§step 5)
before estimator runtime logs (§step 6) for exactly this reason.


## Related

- [`ANALYSIS.md`](./ANALYSIS.md) — APE / RPE / Umeyama explained
  bottom-up; v7 failure case study; per-failure-mode root-cause
  notes.
- [`PERFORMANCE.md`](./PERFORMANCE.md) — separate issue: why
  gz-sim was rendering on the CPU and not the GPU. Different bug,
  same kind of "input quality" thinking.
- [`SESSION_2026-05-15.md`](./SESSION_2026-05-15.md) — narrative
  log of today's debug session including the GUI/EGL detour.
- [`PARAMETERS.md`](./PARAMETERS.md) — what each knob in
  `estimator_config.yaml` and `vins.yaml` does.
