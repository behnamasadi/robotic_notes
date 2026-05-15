# VIO/LIO diagnostic guide

How to evaluate a visual-inertial (VIO) or LiDAR-inertial (LIO)
odometry estimator, and a structured procedure for finding the cause
when it's broken. Built from the methodology developed during this
project's investigation of VIO behaviour — sim-specific case studies
have been removed; only the general procedure and the simulated-IMU
conclusions remain.

The guide has four parts:

1. **The metrics** — what numbers we actually look at and what they mean.
2. **The procedure** — a top-down checklist for diagnosing a bad estimator.
3. **Why LIO survives bad IMU but VIO doesn't** — the architectural
   reason that determines what tooling and data each can use.
4. **On trusting simulated IMU data** — what gz-sim's IMU does and
   doesn't model, and why it can't be the canonical surface for VIO
   comparison.

If you're new to `evo` / APE / RPE / Umeyama, read
[`ANALYSIS.md`](./ANALYSIS.md) first — it explains those concepts
bottom-up. This guide builds on them.


## 1 — The metrics

Mostly two categories: **trajectory metrics** (was the estimator
right about where the robot went?) and **input-quality metrics** (was
the sensor data even sensible?). Many bugs hide in input-quality;
many estimator tuners only look at trajectory metrics and chase the
wrong problem.

### 1.1 — Trajectory metrics

#### Path length ratio

```
ratio = estimator_path_length / ground_truth_path_length
```

A healthy estimator gives a ratio close to 1.0. **The single most
useful first-look metric.** It reduces an entire trajectory to one
number and that number is geometry-meaningful:

- **ratio ≈ 1.0** — estimator's metric scale agrees with reality.
- **ratio > 1** — estimator over-shoots length. Common in monocular
  VIO that can't observe scale (forward driving in a corridor, pure
  rotation). Also common when the camera-IMU extrinsics or IMU bias
  is off and the filter integrates phantom translation.
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

#### Absolute Pose Error (APE)

The Euclidean distance between estimator and ground-truth poses,
*after* aligning them. Three flavours:

```bash
evo_ape bag2 <bag> <gt_topic> <est_topic>      # raw — no alignment
evo_ape bag2 <bag> <gt_topic> <est_topic> -a   # SE(3) alignment (R + t)
evo_ape bag2 <bag> <gt_topic> <est_topic> -va  # Umeyama (R + t + s)
```

What each tells you:

- **No alignment** — measures the literal drift in the estimator's
  reported world frame. If the static-TF wiring is correct and the
  estimator inits with the right yaw, this is your real-world
  positioning error.
- **SE(3) alignment** — translation + rotation difference after
  optimally rotating/translating one trajectory onto the other.
  Removes the "wrong initial yaw" bug but keeps scale honest.
- **Umeyama (–va)** — also allows a global scale factor. **If APE
  drops dramatically going from `-a` to `-va`, you have a scale
  problem.** The scale factor `evo` reports is a close cousin of the
  path-length ratio above.

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
→ 10 %. The KITTI benchmark uses a similar metric ("translation error
[%]"). Under ~2 % on long drives is competitive; under 1 % is
excellent. >10 % is broken.

### 1.2 — Input-quality metrics

These are often skipped, and they're often the actual problem.

#### IMU at-rest sanity

When the robot is stationary on flat ground, a healthy IMU reports:

```
|accel|  ≈ 9.81 m/s²              (gravity magnitude)
ax       ≈ 0
ay       ≈ 0
az       ≈ +9.81                  (when z-up convention)
gyro     ≈ 0   (all axes, modulo noise)
```

If gravity is split between two axes, the IMU is mounted tilted or
its body frame doesn't match what you wrote in the YAML. If
|accel| is off by 5–10 %, the gravity-magnitude setting needs
adjusting. If `az` is negative, you have a sign-convention bug
somewhere (very common with REP-103 vs Z-down conventions).

#### IMU bandwidth and saturation

This is the input-quality check almost nobody does, and it's where a
broken simulated IMU usually shows up:

```
max(|accel|) over the recording should be < ~30 m/s² for a wheeled
robot, < ~50 m/s² for an aggressively flown drone.
std(accel.{x|y|z}) should be a few m/s², not 10+.
```

If you see spikes >50 m/s², either the platform is crashing into
things or the IMU is bandwidth-unlimited and aliasing impulsive
forces. **Real MEMS IMUs are physically rate-limited.** Datasheet
bandwidths are typically 50–500 Hz. A sample published at 250 Hz
that shows a single 200 m/s² spike is impossible on real hardware —
it's a sim artefact.

Plot the per-axis accel over time and look for:
- Single-sample spikes (impossible on real IMU — bandwidth bug)
- Plateaus at the same value (sensor saturating at its range)
- Constant offset (un-modelled bias — VIO can usually handle this
  if it estimates bias online)

#### IMU rate consistency

Check that the IMU is publishing at its configured rate. If your
source advertises 250 Hz but you measure 215 Hz, the data is being
produced at ~86 % of the configured rate. That's fine in itself but
it means:
- The estimator is processing 14 % fewer IMU samples than it expects.
- Pre-integration windows are timestamp-aligned, so they contain
  fewer samples → noisier integration.
- Time offsets between IMU and camera can drift over long runs.

#### Camera rate and timestamp jitter

Same idea for cameras. If the source advertises 30 Hz and the bag
captures 10 Hz, either:
- The data wasn't produced fast enough.
- The bag writer dropped frames (default `keep_last: 10` reliable
  QoS subscriber queue overflows).
- The estimator is downsampling internally (e.g. VINS-Fusion's
  `freq` parameter).

Stereo synchronisation: left and right frame timestamps should
differ by less than ~1 ms for tight stereo matching. Most stereo
front-ends use approximate time sync with ~30 ms tolerance, but
beyond ~50 ms you start losing correspondences.

#### Calibration consistency

The intrinsics + distortion + camera-IMU extrinsics in your
estimator YAMLs must match the actual sensor producing the data.
Cross-checks:

```
camera width/height/fx/fy/cx/cy/distortion
  ↔ openvins/kalibr_imucam_chain.yaml intrinsics/distortion_coeffs
  ↔ vins/cam0.yaml + cam1.yaml projection_parameters/distortion_parameters

camera-IMU extrinsics
  ↔ T_imu_cam in kalibr_imucam_chain.yaml
  ↔ body_T_cam in vins.yaml

IMU rate
  ↔ kalibr_imu_chain.yaml update_rate
  ↔ vins.yaml acc_n / gyr_n (continuous-time noise density)
```

A 0.01 m baseline mismatch in stereo extrinsics → ~10 % scale error
on triangulated depth → ~10 % path-length error. A 5° rotation
mismatch between camera and IMU → wrong gravity vector in the filter
→ systematic horizontal drift.


## 2 — The diagnostic procedure

When an estimator is broken, run this checklist top-down. Each step
either confirms the issue or moves you down to the next level.
**Don't skip steps.** It's tempting to start with calibration when
you see scale drift, but that's the fourth step, not the first.

### Step 0 — Is the bag what you think it is?

```bash
python3 scripts/analyze_bag.py runs/<bag>
```

Look at the topic-rates table in `summary.md`. Verify:

- Both stereo cameras are present (if you expected stereo)
- IMU is at expected rate
- Ground-truth is publishing
- All estimator output topics are present

A common silent failure: `ros2 bag record -a` started before a topic
was publishing, so it was never subscribed. The "stereo VIO" in that
recording becomes effectively mono with no warning. Always check the
topic list before drawing any conclusions.

### Step 1 — Is LIO healthy?

If you have a LiDAR-inertial estimator running in parallel, check
its trajectory first. **LIO failing usually means the source data is
broken** (TF tree, IMU completely garbled, missing scans). VIO
failing while LIO succeeds means the problem is downstream of the
LiDAR — in the camera path or in the IMU's interaction with VIO
specifically. See §4b for the structural reason this asymmetry
exists.

### Step 2 — Look at the trajectory shape

Open the `trajectory_xy.png` plot from `analyze_bag.py`. Compare
each estimator's path to GT, ignoring scale for a moment.

- **Same shape, different scale** → calibration baseline or
  intrinsics mismatch.
- **Wrong direction** (e.g. estimator goes −x when GT goes +x) →
  initial yaw error; dynamic init picked the wrong rotation.
- **Sign-flipped axis** (e.g. y inverted) → coordinate-convention
  mismatch (REP-103 vs other) or a left-handed/right-handed frame
  bug in the estimator config.
- **Explodes off to thousands of metres** → divergence, typically
  from initialization failure.
- **Walks off perpendicular to GT** → gravity vector wrong → filter
  thinks gravity is horizontal acceleration.

If two estimators disagree with each other *and* with GT, that
rules out a shared static-TF bug and points at the estimators'
initialisations producing different (wrong) global-frame yaws.

### Step 3 — Compare scale ratios across drive speeds

If you have two recordings of the same scenario at different speeds
and the scale ratio is similar, **fast motion is NOT the cause.**
Common myth: "VIO blows up because the robot drove too fast."
Sometimes true (KLT feature tracker loses features when inter-frame
pixel displacement is huge), but only at very high speeds and low
frame rates. Below ~1 m/s and above ~20 Hz, motion speed is rarely
the limiting factor.

### Step 4 — Calibration consistency check

Read the source-data calibration and every estimator's calibration
YAML side-by-side. Match each field. The list in §1.2 "Calibration
consistency" enumerates the cross-checks.

Most useful: **compute the implied stereo baseline two ways.** From
the source (or from `camera_info` if you have it) and from the
YAML's `T_imu_cam` translations. If they disagree by N %, your
scale ratio is off by N %.

### Step 5 — IMU sanity (the smoking gun usually shows up here)

Open the per-axis accel plot. Mentally compare to the expected
motion profile.

- Stationary phases: |accel| flat at ~g, gyros at 0 (modulo noise).
- Forward driving at constant v: |accel| still ~g, gyros at 0
  (robot not turning).
- Turning: small gyro_z bump, accel essentially gravity.
- Acceleration phase: small (1–2 m/s²) ax bump, then back to g.

Anything dramatically off — single-sample spikes, plateau-clipping,
the gravity vector being on the wrong axis — is your bug. In sim
recordings the most common finding is impulsive accel spikes at
every cmd_vel direction change; see §4 below.

### Step 6 — Read the estimator's own runtime log

If steps 0–5 haven't pinned it, the estimator may be telling you
exactly what's wrong if you look. For OpenVINS, that's
`/proc/<pid>/fd/1` while it runs, or whatever your launch captured
to a log file. For VINS-Fusion, similar.

Look for:
- The init banner (does it say it initialised in stereo? mono?
  static? dynamic?)
- Feature counts per frame (is the tracker losing features?)
- ZUPT acceptances during motion (zero-velocity update firing when
  the robot is actually moving = critical bug).
- Filter divergence warnings.
- χ² test failures on measurement updates.


## 3 — Why LIO survives bad IMU data and VIO doesn't

A puzzling observation worth understanding because the answer
determines what tooling and data each estimator class can rely on:
**on the exact same IMU stream**, FAST-LIO produces clean
trajectories (0.3–1.5 % end-point error) while OpenVINS and
VINS-Fusion blow up scale by 3–10× when the IMU is bad. Same
impulses, opposite outcomes.

### The IMU plays a fundamentally different role in each

**In VIO**, the IMU is the **scale and velocity sensor.** Between
two camera frames (33 ms apart at 30 Hz), the filter integrates the
IMU forward — typically ~250 samples per camera interval — to
propagate position, velocity, and orientation. Then the camera
observation comes in and weakly corrects via triangulated features.
The cameras can constrain depth (via stereo baseline) and
orientation (via feature flow) but **velocity has to come from IMU
integration**.

If a spike of 200 m/s² hits the IMU for one 4 ms sample, that's
0.8 m/s of phantom velocity error injected per spike. Over a 33 ms
camera frame, that integrates to ~13 mm of position error. Over a
187 s recording at 30 Hz, you accumulate ~73 m of drift — exactly
the order of magnitude of the 3–10× scale blow-ups observed in
practice.

The camera-side correction can fight this, *if* the filter's IMU
noise model accurately reflects the noise. A standard EKF/MSCKF
assumes IMU noise is small Gaussian. When the actual noise is
impulsive, the filter's covariance estimates are wildly wrong, and
the camera correction becomes too weak to drag the state back to
truth.

**In LIO** (FAST-LIO specifically), the IMU is **just an
initial-guess provider** for ICP. Each LiDAR scan returns tens of
thousands of points; scan-to-scan ICP directly observes the
relative motion between scans via geometric correspondences. Even
if the IMU says "I moved 1 m forward in the last 0.1 s" but you
actually moved 0.1 m, ICP looks at the two point clouds, sees they
overlap after a 0.1 m shift, and corrects to that. The IMU's bad
estimate makes ICP take more iterations to converge, but the
*final* answer comes from the geometric registration, not from the
IMU integration.

### The structural difference

```
VIO :  IMU integration  →  state  →  camera CORRECTS state
        ^^^^^^^^^^^^^^^^^^^^^^^^^^
        impulses live here; correction is weak

LIO :  IMU integration  →  initial guess  →  ICP REPLACES state
                                              ^^^^^^^^^^^^^^^^^
                                              geometry, not IMU-derived
```

In VIO the IMU is on the *integrator path*: its errors compound over
time and the camera fights them with limited authority. In LIO the
IMU is on the *hint path*: its errors get thrown away every scan
because ICP doesn't trust the IMU for absolute position.

### Practical consequences

- **Simulator recordings are usable for LIO comparison** — the IMU
  artefacts a typical robotics simulator produces are warm-start
  noise to FAST-LIO, not a position-error source.
- **Simulator recordings are NOT usable for VIO comparison** — the
  IMU artefacts compound through integration and produce
  trajectories that no real-world equivalent would generate.
- **ORB-SLAM3's inertial mode "explodes when the IMU goes bad"** for
  the same reason: it integrates the IMU in the same architectural
  role as OpenVINS / VINS-Fusion; bad IMU → bad propagation →
  factor-graph diverges.
- **Pure visual SLAM (no IMU)** is yet another regime: ORB-SLAM3 in
  vision-only mode is unaffected by IMU artefacts but loses the
  scale anchor IMU provides. Useful as a control experiment.


## 4 — On trusting simulated IMU data

Why this project's benchmarking lives on real datasets (EuRoC,
TUM-VIO) rather than on the SubT rover simulator, even though the
sim is fully working for LIO.

### What gz-sim's IMU models

The `imu` sensor in Gazebo Harmonic reads the rigid body's
instantaneous acceleration and angular velocity from the physics
solver, adds configurable Gaussian noise + bias random walk, and
publishes. That's all.

### What it does NOT model

Real MEMS IMUs have:
- A mechanical bandwidth (proof mass + suspension acts as a 2nd-order
  low-pass filter, typically 100–500 Hz cutoff).
- An analog anti-alias filter before the ADC.
- Hard saturation at the full-scale range (±16 g for cheap parts,
  ±200 g for industrial).
- 16-bit quantization.

None of these are in the gz-sim sensor — its accel output is
effectively a full-precision float of the *instantaneous* physics
state.

### Why it matters

Two consequences worth internalising:

**1. The simulator can produce IMU signals that no real device would
ever produce.** Any impulsive force in the physics solver appears in
the IMU as a single-sample spike of arbitrary magnitude. Wheel
contact transitions, instantaneous velocity steps from a diff-drive
plugin, and discontinuous joint controllers all generate these. A
real BMI088 would clip those impulses to ±16 g and smear them
across ~10 samples through its 145 Hz internal filter. **A VIO tuned
on real IMUs chokes on signals that real hardware can't physically
produce.** The usual concern about sim-to-real is "sim is too easy";
here it's the opposite — sim is *unrealistically harsh* in one
specific way.

**2. The physics is also too "perfect"** — no motor dynamics, no
gearbox compliance, no wheel-ground compliance. Real robots smear
acceleration commands across many ms. The default diff-drive plugin
steps wheel velocities instantly when `/cmd_vel` arrives, which is
what causes the impulsive forces in the first place. So the bug is
two layers of unrealism stacked: physics producing impulses that
wouldn't happen on real hardware, *and* the IMU sensor passing those
impulses through without the bandwidth limiting that would absorb
them on real hardware.

### How the community actually develops VIO

Every VIO paper from the last 5 years (OpenVINS, VINS-Fusion,
Kimera, ORB-SLAM3, MSCKF, ROVIO) reports benchmark numbers against
**real-world datasets** — EuRoC MAV, TUM-VIO, KITTI, NTU-VIRAL — not
against simulation. Simulation is used for:

- *Algorithm correctness* — "does my filter converge under clean
  data?" Easy to check with a perfect-IMU sim.
- *Stress testing* — "what if I add N % calibration error?" Inject
  the error, see what breaks.
- *Ground-truth-rich debugging* — when something fails on real data,
  reproduce it in sim with GT to isolate algorithm-vs-data.

What simulation is **not** trusted for: deciding whether a VIO will
work on a specific physical platform. Only real-hardware testing
answers that.

### Implications for this benchmark

- Real datasets (EuRoC, TUM-VIO, etc.) are the canonical surface.
  See [`COMPARISON.md`](COMPARISON.md) for results, [`DATASETS.md`](DATASETS.md)
  for download recipes.
- Workarounds for the simulator artefact exist (`cmd_vel_smoother`
  + an `imu_lowpass` republisher with realistic bandwidth and
  saturation), but they're band-aids; they don't address the
  physics-solver artefacts at the source.
- The sim remains useful for LIO, for behavioural tests where IMU
  artefacts are not in the critical path, and for project-specific
  scenarios (e.g. SubT cave environments) that no public dataset
  covers. But the canonical *quality* numbers come from real data.


## Related

- [`ANALYSIS.md`](./ANALYSIS.md) — APE / RPE / Umeyama explained
  bottom-up; parallax and forward-driving aside; mono unobservable
  yaw; KLT vs descriptor tracker; ROVIO arbitrary-yaw aside.
- [`COMPARISON.md`](./COMPARISON.md) — head-to-head numbers from
  EuRoC.
- [`DATASETS.md`](./DATASETS.md) — EuRoC + TUM-VIO + other dataset
  recipes.
- [`PARAMETERS.md`](./PARAMETERS.md) — what each knob in
  `estimator_config.yaml` and `vins.yaml` does.
