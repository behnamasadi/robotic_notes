# Trajectory analysis — methodology and concepts

This document collects the conceptual knowledge built up while
investigating VIO behaviour: what the metrics actually measure, why
trajectories drift in characteristic ways, and what algorithm choices
trade off against each other. Sim-specific case studies that originally
lived here have been removed — they were tied to recordings that no
longer matter for the real-data comparison. The numbered comparison
numbers live in [`COMPARISON.md`](COMPARISON.md); the structured
diagnostic procedure lives in [`VIO_DIAGNOSTIC_GUIDE.md`](VIO_DIAGNOSTIC_GUIDE.md).

## APE, RPE, and Umeyama alignment — what the numbers mean

When you read "VIO RMSE is 0.48 m" in this doc or in any VIO paper,
it's almost never the raw error. Here's what each metric actually is
and why we report aligned values most of the time.

### Setup

Two time-synchronised trajectories:

| | |
|---|---|
| **GT** | `{ Tgt(t1), Tgt(t2), …, Tgt(tN) }` — each `Tgt` is an SE(3) pose (position + orientation) at time `ti` |
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

**Catch with raw APE**: it implicitly assumes `Tgt` and `Test` are in
the *same world frame*. They almost never are. `Tgt` is in the
dataset's world frame; `Test` is in whatever frame the estimator chose
at init. For OpenVINS that's `global`; for VINS-Fusion it's `world`;
for FAST-LIO it's `camera_init`. These frames are **arbitrarily
rotated** relative to the world because IMU-only init can't determine
yaw — gravity has no yaw component (see
[Mono VIO has unobservable yaw](#mono-vio-has-unobservable-yaw)).

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
| Compare estimator *shapes* against truth on a single bag | **`-as` aligned APE** | Removes the unobservable yaw + scale; what mono VIO papers report |
| Compare estimators against each other on the same bag | **`-a` aligned APE** | Both estimators have arbitrary yaw, so align both to GT and they're comparable |
| Measure *short-term* drift | **RPE** | Doesn't care about initial frame, measures local consistency |
| "Does the trail overlay GT in RViz?" | **raw APE** | Useless as a quality metric without a heading sensor — measures frame mismatch, not algorithm error |


## Parallax — and why forward driving is the worst case

The short version: **a monocular camera estimates depth from how
things move across the image when the camera itself moves.** Stuff
near you swings across the image fast; stuff far away barely moves.
The *difference* between those motion rates is parallax, and it's the
only information a single 2D image gives you about 3D depth.

Stationary observer, near + far points: the near one *appears* to
move more relative to the far one. Same idea for a moving observer.

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

### Tunnel / corridor geometry makes it worse

If the platform is in a tight corridor:

- Forward motion is along the camera's optical axis.
- The corridor walls are within a few metres of the camera.
- Ceiling, floor, and side walls occupy most of the FOV — so most of
  what the camera sees is close to "dead ahead."
- Self-similar texture (rock, brick, hex lattice) compounds the
  problem because the feature tracker can latch onto identical-looking
  neighbouring patches.

The result: VIO can correctly track *which* features are moving, but
it can't pin down *how fast* the camera is actually moving in metres,
because every consistent (velocity, depth) pair produces the same
image motion.

### Quick numerical example

Camera at origin looking along +X. Two features:

| Feature | Position (X, Y, Z) | Before move | After camera moves +1 m in X | Image shift |
|---|---|---|---|---|
| Dead ahead, 10 m | (10, 0, 0) | centre | centre | **0 pixels** |
| Dead ahead, 5 m  | ( 5, 0, 0) | centre | centre | **0 pixels** |
| 2 m to the side, 10 m ahead | (10, 2, 0) | 11.3° off-axis | 12.5° off-axis | 1.2° (~3 px) |
| 2 m to the side, 5 m ahead  | ( 5, 2, 0) | 21.8° off-axis | 26.6° off-axis | 4.8° (~13 px) |

The first two rows have **identical image motion** despite being at
different depths. From a single mono camera, you can't tell which is
which.

### What fixes it

Three things, in order of how much they help:

1. **Sideways motion** (translation perpendicular to the optical axis).
   A pure sideways move turns every point into row 4 of the table
   above — strong parallax, strong depth signal. Rotation alone
   doesn't help (it changes pixel positions without changing depth),
   but a yaw + small forward velocity (a figure-8) gives continuous
   sideways translation.
2. **Stereo cameras.** With a baseline of, say, 10 cm between two
   cameras, you get depth-from-disparity at *every* frame — no motion
   required. Scale is recoverable from the first frame onward. This
   is the real fix for forward-driving scale loss.
3. **Depth from another sensor** (RGBD camera, LiDAR). Feeds depth
   directly into a depth-aware VIO. LIO does this implicitly with
   LiDAR scan registration — see
   [`VIO_DIAGNOSTIC_GUIDE.md §4b`](VIO_DIAGNOSTIC_GUIDE.md#4b--why-lio-survives-bad-imu-data-and-vio-doesnt)
   for the structural reason LIO is also robust to bad IMU on top of
   this.


## Mono VIO has unobservable yaw

It's not a config bug; it's a fundamental property of IMU-only static
init:

- **Gravity** (a vertical vector) constrains pitch and roll. The
  estimator gets these right at init — that's why the trajectory
  stays close to z=0 (with some drift) instead of pitching into the
  ground.
- **Yaw** is rotation *around* the gravity axis. Gravity carries zero
  information about it. The IMU at rest produces the same gyro and
  accel readings regardless of which way the platform is facing in
  the world.
- The static-init code has to pick *some* yaw value to bootstrap.
  Without an external heading reference, it picks based on the first
  IMU readings + the propagated state — effectively random with
  respect to the world.

Symptom: trajectory looks mirrored, rotated, or pointing in the
opposite direction when overlaid on GT in the world frame. Aligned APE
(`-a` or `-as`) is fine because Umeyama removes the rotation; raw APE
is huge.

### Fixes that resolve this for good

In rough order of effort vs payoff:

1. **Stereo VIO** — two cameras separated by a baseline observe depth
   from the first frame onward, and rotation becomes observable from
   stereo correspondences. Yaw is no longer ambiguous. This is the
   "real fix" for everything mono VIO can't decide.
2. **Magnetometer fusion** — the Earth's magnetic field gives a
   heading reference. Most filter-based VIOs don't support it
   directly, but a downstream `robot_localization`-style EKF can fuse
   `/imu` + `/magnetometer` + `/<estimator>/odometry` and produce a
   yaw-anchored estimate.
3. **Known-heading prior** — if the platform always spawns facing a
   known direction (e.g. world +X), feeding "yaw = 0" as an init
   constraint nails the yaw deterministically. Usually requires a
   small patch to the estimator's init code.
4. **Loop closure** — if the platform returns to a previously-seen
   place and the estimator's place-recognition layer fires, the yaw
   error gets corrected globally. ORB-SLAM3 and Kimera-VIO have this;
   OpenVINS' MSCKF does not have direct loop closure (it has
   `/ov_msckf/loop_*` topics but only fires on revisits).


## Tracker choice: KLT vs descriptor matching

VIO front-ends track features between consecutive frames. The two
common choices:

| **KLT (Kanade-Lucas-Tomasi)** | **Descriptor matching (ORB, SIFT)** |
|---|---|
| Lucas-Kanade pyramidal optical flow on corner features. | Compute a binary or float descriptor per corner, brute-force or FLANN match next frame's corners against previous. |
| Very fast — image-warp + intensity SSD per patch. | Slower — descriptor compute + match per frame. |
| Patch-based: robust to mild appearance change. | Descriptor-based: robust to viewpoint + illumination change. |
| On self-similar texture (brick walls, tile floors, hex lattice), the patch optimiser can converge to a *neighbouring identical* patch — silent false correspondence. | Lowe's ratio test (`d1/d2 < 0.7`) rejects ambiguous matches automatically. |

The failure mode that distinguishes them: KLT drifts under
self-similar texture or large inter-frame motion; descriptor matchers
fail more cleanly by returning fewer matches but the ones they return
are unambiguous.

### When to switch

- **Default to KLT.** Cheaper, gives more matches per frame, fine in
  texture-rich scenes (outdoor, varied indoor).
- **Switch to ORB/descriptors** when the scene is texture-poor or
  highly repetitive (corridors, factory floors, large blank walls)
  and you see orientation jitter that the IMU can't explain.

OpenVINS exposes this as a single config flag in
`estimator_config.yaml`:
```yaml
use_klt: true   # default
use_klt: false  # ORB + binary descriptor + kNN ratio
```

Trade-off: extra CPU per frame for ORB. With `num_pts: 200` it's well
within budget on a modern desktop.


## Aside: ROVIO on a drone didn't have "arbitrary yaw" — why?

If you've seen a colleague run ROVIO (or any mono VIO) on a drone with
**6-axis IMUs** (no magnetometer) and **two IMUs mounted in opposite
orientations** for bias cancellation, and never observed the
"trajectory points in the opposite direction" failure we describe
above — three things explain that, none of which is the IMU layout:

1. **They were comparing ROVIO to ROVIO.** With no ground truth in
   the field, you can't *see* the arbitrary yaw offset — the
   estimator's trail is in its own internal world frame, and as long
   as motion is consistent within that frame, everything "looks
   right." You'd see the 180° flip on a benchmark rig with GT and
   raw `‖VIO − GT‖`. The drone almost certainly had the same offset,
   invisible.
2. **Drone motion is parallax-rich.** Constant rotation + translation
   in 3D means the camera-update loop pins yaw to *some* consistent
   value within seconds of takeoff, even though that value is
   arbitrary relative to north. Once pinned, it stays.
3. **Two opposing IMUs cancel bias drift.** Accelerometer/gyro
   biases of two units roughly cancel when averaged. That stops the
   wrong-yaw decision from *compounding* over time — whatever yaw was
   chosen at init stays stable for the whole flight. **But this
   doesn't add a yaw observation**; it just keeps the wrong yaw from
   drifting further wrong.

Aligned APE on the colleague's setup, if you'd had Vicon GT, would
have shown the same yaw rotation as we see in our benchmark. It just
never had to be reported, so it didn't matter for navigation.

**Conclusion**: the arbitrary-yaw effect is fundamental to any
mono-VIO (or stereo-VIO without heading sensor) when reported against
a fixed-world ground truth. It's an *evaluation artefact*, not a flaw
in the estimator. The aligned-APE numbers are the honest quality
measures. The "trail goes opposite direction in RViz" view is the
unavoidable cost of having ground truth.


## Related

- [`COMPARISON.md`](COMPARISON.md) — head-to-head numbers from EuRoC
- [`VIO_DIAGNOSTIC_GUIDE.md`](VIO_DIAGNOSTIC_GUIDE.md) — the 7-step
  diagnostic procedure for broken VIO + §4b (LIO-vs-VIO IMU role) +
  §5 (on trusting simulated IMU data)
- [`PARAMETERS.md`](PARAMETERS.md) — what each knob in
  `estimator_config.yaml` and `vins.yaml` does
- [`DATASETS.md`](DATASETS.md) — EuRoC + TUM-VIO + others, recipes
