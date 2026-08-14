# lio_benchmark

Gazebo simulator for LiDAR-inertial odometry: a SubT-style rover
(16-beam LiDAR + IMU + stereo cameras), three cave/tunnel worlds, and
four estimators wired in side-by-side — three pure odometry frontends
(FAST-LIO + DLIO + KISS-ICP) plus one full SLAM stack with loop
closure (LIO-SAM). All containerised.

## Layout

```
lio_benchmark/
├── src/explorer_r2_sim/    ROS 2 package — rover model, worlds, launches,
│   ├── models/explorer_r2/ configs, RViz, scripts, Dockerfile + compose
│   ├── worlds/             cave.sdf, tunnel.sdf
│   ├── config/             bridge.yaml, lio.yaml, dlio.yaml, kiss_icp.yaml, lio_sam.yaml, joy_teleop.yaml
│   ├── launch/             cave.launch.py, lio.launch.py, dlio.launch.py, kiss_icp.launch.py, world.launch.py
│   ├── rviz/               RViz configs
│   ├── scripts/            record_run.sh, analyze_bag.py, gt_to_path.py, …
│   ├── docker/             sim.Dockerfile + entrypoint
│   └── docker-compose.yml
├── third_party/FAST_LIO/   submodule (hku-mars/FAST_LIO, ROS 2 branch) — built inside the `sim` container
├── third_party/DLIO/       submodule (vectr-ucla/direct_lidar_inertial_odometry, feature/ros2) — built inside `sim`
├── third_party/kiss-icp/   submodule (PRBonn/kiss-icp, main) — built inside `sim`
├── third_party/LIO-SAM/    submodule (TixiaoShan/LIO-SAM, ros2) — kept for source reference; not
│                           symlinked into `sim`. The `lio_sam` compose service builds its own image
│                           via `docker/lio_sam.Dockerfile` (ROS Jazzy + GTSAM 4.1 from source)
│                           and clones LIO-SAM fresh at build time
└── docs/PERFORMANCE.md     gz-sim GPU/render perf notes (EGL pivot)
```

## Quick start

```bash
# 1. From the repo root (where .git lives — robotic_notes/, NOT
#    lio_benchmark/). Fetches FAST_LIO, DLIO, and their nested submodules.
cd ~/workspace/robotic_notes
git submodule update --init --recursive

# 2. cd into the sim package and bring everything up.
cd lio_benchmark/src/explorer_r2_sim
xhost +local:root
docker compose up --build
```

Two containers come up: **`sim`** (Gazebo, RViz, FAST-LIO, DLIO,
KISS-ICP, ground truth — ROS Jazzy) and **`lio_sam`** (LIO-SAM in its
own image, also ROS Jazzy, GTSAM 4.1 from source). They share host
networking + the same `ROS_DOMAIN_ID` so DDS connects them
transparently. RViz, running inside `sim`, overlays all four estimator
trajectories against ground truth:

- **gold** — ground truth
- **blue** — FAST-LIO (LIO, no loop closure)
- **green** — DLIO (LIO, no loop closure)
- **orange** — KISS-ICP (LiDAR only, no IMU, no loop closure)
- **purple** — LIO-SAM (LIO + factor-graph SLAM with loop closure)

If you only want the three odometry frontends (faster startup, no
GTSAM/LIO-SAM build), bring `sim` up explicitly: `docker compose up sim`.

Drive with a joystick (default) or `teleop_twist_keyboard` in another
terminal. Pick a different world with `WORLD=cave docker compose up`.

## Which container for which command

`docker compose up` brings up both services on host networking,
sharing `ROS_DOMAIN_ID=42`. They see each other's topics + TF as one
ROS graph.

| Service | What runs inside | Image | When to exec into it |
|---|---|---|---|
| **`sim`** | Gazebo, RViz, ros_gz_bridge, FAST-LIO, DLIO, KISS-ICP, gt_to_path, joy/teleop | `explorer_r2_sim/sim:jazzy` (our Dockerfile — ROS Jazzy + GZ Harmonic) | Almost everything: teleop, ros2 topic / run / launch / bag, TF inspection, recording runs, parameter changes |
| **`lio_sam`** | LIO-SAM's four nodes (`imuPreintegration`, `imageProjection`, `featureExtraction`, `mapOptimization`) plus the static TFs LIO-SAM needs internally | `lio_sam_jazzy:latest` (`docker/lio_sam.Dockerfile` — ROS Jazzy + GTSAM 4.1 from source) | Only to debug LIO-SAM itself |

Day-to-day, you exec into **`sim`**:

```bash
# Generic shell
docker compose exec sim bash

# Drive the rover from a keyboard
docker compose exec sim ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Inspect topics
docker compose exec sim ros2 topic list
docker compose exec sim ros2 topic hz /Odometry              # FAST-LIO
docker compose exec sim ros2 topic hz /dlio/odom_node/odom   # DLIO
docker compose exec sim ros2 topic hz /kiss/odometry         # KISS-ICP
# LIO-SAM uses BEST_EFFORT QoS:
docker compose exec sim ros2 topic echo --once --qos-reliability best_effort /lio_sam/mapping/odometry

# Inspect TF
docker compose exec sim ros2 run tf2_ros tf2_echo explorer_r2/odom explorer_r2/base_link

# Re-launch with a different rover-TF mode
docker compose run --rm sim ros2 launch explorer_r2_sim cave.launch.py rover_tf:=wheel

# Record a run
docker compose exec sim bash
ros2 run explorer_r2_sim record_run.sh tunnel_drive_1
```

## What you see in RViz

The rover mesh in RViz is anchored to **ground truth**, not to wheel
odometry. The TF that positions `explorer_r2/base_link` is broadcast by
`scripts/gt_to_path.py` from gz's `/world/<world>/dynamic_pose/info`
stream (the simulator's true pose, computed from physics — not from any
sensor). The wheel-odometry TF that the gz `DiffDrive` plugin produces
is intentionally not bridged into ROS — see the `# ─── TF` block in
`config/bridge.yaml` for the comment, and the `TransformBroadcaster`
section near the bottom of `gt_to_path.py` for the GT TF publisher.

This is a deliberate visualization choice for benchmarking. With the
rover anchored to truth, a drifting LIO estimator shows up as its
**arrow trail offsetting from the rover** — the rover stays where it
actually is, and the failure is visible as a diverging coloured line.
The opposite arrangement (rover positioned by wheel odometry or by one
estimator's output) makes the rover *itself* drift around RViz while
ground truth becomes the moving target, which is confusing and gives
the wrong intuition about who's right.

Wheel odometry hasn't been thrown away — it's still published as a
topic on `/model/explorer_r2/odometry` (`nav_msgs/Odometry`). If you
want to compare wheel-odo to the LIO estimators, subscribe to that
topic and add an `Odometry` display in RViz with a fifth colour.

### Frames you'll see in `/tf`

When the sim is running, three rover-position frames coexist in the
TF tree — one for ground truth and one for each LIO estimator that
publishes a TF:

```
explorer_r2/odom                ← RViz Fixed Frame
├── explorer_r2/base_link       ← TRUTH (from gt_to_path.py)
│   ├── explorer_r2/lidar       ← sensor poses on the true robot
│   ├── explorer_r2/imu         ←   (from gz PosePublisher's
│   └── …                       ←    publish_sensor_pose)
├── camera_init                 ← static identity (lio.launch.py)
│   └── body                    ← FAST-LIO's ESTIMATE of the rover
│                                 (camera_init → body, sent every scan
│                                  from laserMapping.cpp)
└── dlio/odom                   ← static identity (dlio.launch.py)
    └── dlio/baselink           ← DLIO's ESTIMATE of the rover
        ├── dlio/imu            ←   (DLIO publishes its own sensor
        └── dlio/lidar          ←    chain off its baselink frame)
```

KISS-ICP intentionally doesn't publish any TF in our setup — we set
`publish_odom_tf: false` in `config/kiss_icp.yaml` so its pose is
exposed only as `/kiss/odometry`. The RViz `KISS-ICP Odom` display
renders that topic as a trail of orange arrows without needing TF.

So if you ever notice "two robots" in RViz, you're not seeing a bug —
you're seeing the truth (`explorer_r2/base_link`) and one of the LIO
estimates (`body` or `dlio/baselink`) disagreeing. When the LIO is
accurate the two coincide and look like a single robot; when it
drifts they separate, and the gap is the visible error. This is the
whole point of the layout.

A quick way to confirm which transform comes from where:

```bash
ros2 run tf2_ros tf2_echo explorer_r2/odom explorer_r2/base_link  # ground truth
ros2 run tf2_ros tf2_echo camera_init body                        # FAST-LIO estimate
ros2 run tf2_ros tf2_echo dlio/odom dlio/baselink                 # DLIO estimate
```

### Why ground truth is the default anchor

When you ask "what positions the rover mesh?", there are three
reasonable answers — ground truth, wheel odometry, or one of the LIO
estimates. Each gives a different intuition about who's right:

- **Ground truth (current default).** Rover is always at its actual
  pose; LIO arrow trails diverge from it when wrong. Cleanest signal
  for a benchmark — you're seeing the *error*, not arguing about it.
- **Wheel odometry.** Matches what real ROS stacks do on hardware
  (`odom → base_link` from encoders). Rover visibly drifts as
  encoders accumulate error; ground truth becomes the moving target
  the LIO arrows are chasing. Useful for the "real robot" feeling,
  less useful as a clean comparison because two things move at once.
- **One LIO estimate.** That estimator looks perfect; everything else
  looks wrong relative to it. Misleading — circular.

We picked ground truth because the project is a benchmark; the goal is
to see who drifts and by how much, not to recreate the sensor stack of
a real robot. The opt-out below covers the case where you do want the
real-robot view.

If you want the rover mesh itself to follow wheel odometry instead of
ground truth — which is closer to what a real ROS stack does on
hardware — launch with `rover_tf:=wheel`:

```bash
docker compose run --rm sim ros2 launch explorer_r2_sim cave.launch.py rover_tf:=wheel
```

With `rover_tf:=wheel` the gt_to_path.py TF broadcaster is disabled
and gz's DiffDrive wheel-odometry TF is bridged from
`/model/explorer_r2/tf` instead. The rover mesh now drifts as
encoders accumulate error; ground truth becomes the moving target the
LIO arrows are chasing — useful if you want the "real robot" feeling,
less useful as a clean visual benchmark of who's actually right.
Default stays `rover_tf:=ground_truth`.

Map clouds from each LIO (`/cloud_registered`, `/dlio/odom_node/
pointcloud/deskewed`, `/kiss/local_map`) are wired in the RViz config
but disabled by default — toggle individual `*-Map` displays on if you
want to see them. The live `Lidar Cloud` display showing the current
`/lidar/points` scan stays on by default.

## Algorithms compared

Both estimators do the same job: figure out where the robot is by
combining a slow LiDAR (10 Hz scans, very accurate geometry) with a
fast IMU (200 Hz, gives you acceleration and angular rate). Neither
sensor is enough alone — the IMU drifts within seconds if you just
integrate it, and a single LiDAR scan tells you nothing about pose
until you can compare it to something previous. The whole game in LIO
is: use the IMU to fill in the short gaps between scans, use the LiDAR
to correct the IMU drift whenever a new scan arrives. FAST-LIO and DLIO
just disagree on *how*.

### FAST-LIO — Kalman filter with iteration

FAST-LIO tracks a state vector with everything it cares about packed
into one column:

$$\mathbf{x} = [\;\mathbf{R}\;|\;\mathbf{p}\;|\;\mathbf{v}\;|\;\mathbf{b}_g\;|\;\mathbf{b}_a\;|\;\mathbf{g}\;]$$

- $\mathbf{R}, \mathbf{p}, \mathbf{v}$ — robot orientation, position, velocity.
- $\mathbf{b}_g, \mathbf{b}_a$ — slow drifts on the gyro and accelerometer.
  Real IMUs always have these; they wander over minutes.
- $\mathbf{g}$ — gravity in the world frame. The IMU doesn't know which
  way is up at boot, so we estimate this rather than assume $(0,0,-9.81)$.

Between LiDAR scans (so for roughly 100 ms at a time), FAST-LIO
**propagates** the state by integrating IMU samples — basically
$\mathbf{v} \leftarrow \mathbf{v} + (\mathbf{R}\mathbf{a} - \mathbf{g})\,dt$,
$\mathbf{p} \leftarrow \mathbf{p} + \mathbf{v}\,dt$, repeat at 200 Hz —
while tracking how much uncertainty has accumulated.

When a new scan arrives, FAST-LIO does an **iterated Kalman update**.
For each point $\mathbf{p}_i^L$ in the scan, it finds the closest plane
in the running map (defined by a normal $\mathbf{n}_i$ and a point $\mathbf{q}_i$
on it), and treats it as a measurement: "this LiDAR point should lie
on this plane". The residual is the signed distance:

$$h_i(\mathbf{x}) = \mathbf{n}_i^\top(\mathbf{R}\,\mathbf{p}_i^L + \mathbf{p} - \mathbf{q}_i)$$

The filter adjusts $\mathbf{x}$ to drive all those residuals toward zero
*at once*, weighted by their uncertainties. "Iterated" means it does
the update, re-finds correspondences with the new (better) pose, and
updates again — typically three passes — to converge.

The map is stored as an **incremental k-d tree** (the "ikd-Tree" the
HKU-MARS paper made famous): nearest-neighbour lookups in $O(\log n)$,
and new points can be inserted without rebuilding the tree from scratch.

### DLIO — direct optimisation, no filter

DLIO throws out the Kalman machinery and asks a simpler question every
scan: *what robot pose makes this new scan line up best with the recent
map?* Then it solves that as a least-squares problem.

What DLIO keeps around:

- A **continuous-time trajectory** $\mathbf{T}(t)$ — a function that
  returns a pose for any timestamp, built by integrating IMU forward.
- A pile of **keyframes** — previous scans with their estimated poses,
  saved whenever the robot has moved or rotated enough since the last one.

**How DLIO uses the IMU.** Not to propagate a filter — to figure out
*where each individual LiDAR point was when its laser actually fired*.
A 10 Hz LiDAR takes ~100 ms to sweep one full rotation, and the robot
moves during that sweep. For each point $i$ stamped at time $t_i$:

$$\hat{\mathbf{p}}_i^L = \mathbf{T}(t_\text{end})^{-1}\,\mathbf{T}(t_i)\,\mathbf{p}_i^L$$

This "deskews" the scan — i.e., transforms every point as if the whole
scan had been captured at the same instant ($t_\text{end}$). FAST-LIO
deskews too, but assumes constant velocity over the scan; DLIO uses the
real IMU samples in between, which matters when the robot is yawing
quickly.

**How DLIO uses the LiDAR.** It grabs the $k$ nearest keyframes,
stitches them into a local **submap**, and aligns the new (deskewed)
scan to that submap via **Generalised ICP**:

$$\min_{\mathbf{T}}\;\sum_i\;\mathbf{d}_i^\top
\bigl(\mathbf{C}_i^A + \mathbf{T}\,\mathbf{C}_i^B\,\mathbf{T}^\top\bigr)^{-1}
\mathbf{d}_i,\qquad
\mathbf{d}_i = \mathbf{T}\mathbf{p}_i^B - \mathbf{p}_i^A$$

The $\mathbf{C}_i$ covariances describe what the neighbourhood around
each point looks like — plane-ish, line-ish, corner-ish — so the
optimiser pulls harder along directions the geometry actually constrains
(across a wall) and lets it slide along directions it doesn't (down a
corridor). The output pose $\mathbf{T}$ gets pushed back into the
trajectory $\mathbf{T}(t)$ so the next scan's deskew has the latest
correction.

**The "smoothing" step.** DLIO blends the GICP pose with the IMU
prediction through a lightweight geometric observer — basically a
proportional filter with fixed gains $K_p, K_v, K_q, K_{ab}, K_{gb}$
(in `params.yaml`):

$$\dot{\hat{\mathbf{p}}} = \hat{\mathbf{v}} + K_p\,(\mathbf{p}_\text{GICP} - \hat{\mathbf{p}})$$

…and similarly for velocity, orientation, accel bias, gyro bias. This is
the closest thing DLIO has to a Kalman update — no covariance algebra,
just hand-tuned gains.

### KISS-ICP — LiDAR only, no IMU

KISS-ICP is the simplest of the three by far, on purpose. No filter, no
keyframe submap, no IMU input. It does one thing: align every new scan
to a voxelised running map using **point-to-point ICP**, and call that
the new pose. The paper title literally argues that LiDAR odometry has
been over-engineered; their pitch is "keep it simple, stupid".

What KISS-ICP keeps:

- A **voxel-hashed map** (using `tsl::robin_map` for speed) — every point
  drops into a voxel, and if a voxel already has $\le 20$ points, new
  ones get discarded. Downsampling for free, no octree, no k-d rebuilds.
- A **constant-velocity motion prediction**: $\mathbf{T}_k = \mathbf{T}_{k-1}\,\Delta\mathbf{T}_{k-1}$
  where $\Delta\mathbf{T}_{k-1}$ is whatever transform the last scan
  produced. That's the entire dynamics model — no accel, no gyro.

How it uses LiDAR (and *only* LiDAR):

1. **Deskew** the new scan using the constant-velocity guess — linearly
   interpolate the pose between $t_\text{start}$ and $t_\text{end}$ of
   the scan.
2. **Align** the deskewed scan to the voxel map with classical ICP,
   robustified by a Geman–McClure kernel that down-weights outliers:
   $$\mathbf{T}^* = \arg\min_{\mathbf{T}}\sum_i \rho\!\bigl(\|\mathbf{T}\mathbf{p}_i^\text{scan} - \mathbf{q}_i^\text{map}\|\bigr)$$
   where $\mathbf{q}_i^\text{map}$ is the nearest voxel point to
   $\mathbf{T}\mathbf{p}_i^\text{scan}$.
3. **Adaptive correspondence threshold** — start loose, tighten as ICP
   converges. This is the one clever trick that lets KISS-ICP work on
   different sensors / datasets without re-tuning.
4. **Update the map** by adding the registered scan, subject to the
   voxel downsampling.

Why it's in this benchmark. Two reasons:

- **Sanity baseline.** If KISS-ICP works well on your data, you probably
  don't need IMU at all. If LIO is much better than KISS-ICP, that's
  evidence the IMU is actually doing something useful.
- **What does the IMU buy you?** With LIO above (FAST-LIO, DLIO) and
  ICP-only below (KISS-ICP), the gap *is* the IMU contribution. In
  smooth driving it tends to be small; in fast yaw or after collisions
  (see § Why collisions wreck LIO) it can be enormous.

### LIO-SAM — LIO + factor-graph SLAM with loop closure

LIO-SAM is structurally different from the other three: it's a full
**SLAM** stack, not just an odometry frontend. The frontend looks
similar to FAST-LIO/DLIO (LOAM-style edge/plane feature extraction,
IMU pre-integration as a high-rate motion prior), but the backend is
a **factor graph** maintained by **iSAM2** (incremental smoothing).
Every keyframe becomes a node, every odometry constraint becomes an
edge, and — crucially — every verified **loop closure** also becomes
an edge.

What LIO-SAM tracks per scan:

- **IMU pre-integration factor.** Integrates the IMU stream between
  consecutive keyframes and adds a relative-pose factor between them.
  Same idea as FAST-LIO's prediction step, but expressed as a graph
  edge instead of a state propagation.
- **LiDAR odometry factor.** LOAM-style edge + plane scan-to-submap
  registration adds another relative-pose factor between consecutive
  keyframes — the equivalent of FAST-LIO's iterated update or DLIO's
  GICP step.
- **Loop closure factor.** A separate thread runs **radius-based
  place candidate search** (any past keyframe within
  `historyKeyframeSearchRadius` and older than
  `historyKeyframeSearchTimeDiff` is a candidate) and verifies each
  candidate with ICP against the current scan. Surviving candidates
  add a long-range constraint edge to the graph. *(Note: the upstream
  LIO-SAM ros2 branch uses radius search rather than Scan Context for
  place recognition — see § Loop closure in 3D LiDAR-SLAM for the
  general approach; LIO-SAM keeps it simpler than the canonical
  Scan-Context-based pipeline, but augmented forks exist.)*

When a loop fires:

$$\mathbf{X}^* = \arg\min_{\mathbf{X}} \sum_{(i,j) \in \mathcal{E}} \|\mathbf{z}_{ij} \ominus h(\mathbf{x}_i, \mathbf{x}_j)\|^2_{\Sigma_{ij}}$$

…over all edges $\mathcal{E}$ in the factor graph — odometry edges
(short range, between consecutive keyframes), pre-integration edges,
and loop edges (long range, between current and revisited
keyframes). **iSAM2 doesn't re-solve the whole graph; it incrementally
re-linearizes only the variables affected by the new factor**, which
is why loop closures can be integrated online (≈ tens of ms) instead
of seconds.

The visible difference in RViz: when LIO-SAM detects a loop, the
**purple trajectory snaps** — the trail jumps to fix accumulated
drift in one frame. The other three estimators never do this; they
just drift forever.

LIO-SAM also publishes a global map (`/lio_sam/mapping/map_global`)
which is the deduplicated union of all keyframe clouds, transformed
into the loop-closed pose graph. This is the only one of the four
estimators that produces a *globally consistent* map.

### So what's actually different?

| Question | FAST-LIO | DLIO | KISS-ICP | LIO-SAM |
|---|---|---|---|---|
| Uses IMU? | Yes — tightly coupled in the filter | Yes — drives continuous-time prediction | **No, LiDAR only** | Yes — IMU pre-integration factor |
| Estimation framework | Iterated EKF | GICP + geometric observer | Point-to-point ICP | **Factor graph + iSAM2 (SLAM)** |
| Scan matched to? | Whole map (ikd-Tree) | Submap of $k$ nearest keyframes | Whole voxel-hashed map | Local keyframe submap (LOAM features) |
| Deskew | Per-scan, constant velocity | Per-point, full IMU trajectory | Per-scan, constant velocity | Per-scan with IMU |
| Correspondence | Point-to-plane | Plane-to-plane (GICP, covariance-aware) | Point-to-point + robust kernel | LOAM edges + planes |
| Bias / gravity | Co-estimated as filter state | Observer gains $K_{ab}, K_{gb}$; gravity fixed at boot | Not modelled — no IMU | Co-estimated as graph variables |
| Map structure | Incremental k-d tree | Keyframe clouds + nanoflann | Voxel hash map | Keyframe clouds + factor graph |
| **Loop closure** | No | No | No | **Yes — radius search + ICP + iSAM2** |
| Typical tunable parameters | ~15 | ~25 | **~5** | ~40 |
| Boot-time calibration | Not required | 3 s standstill (we disable for sim) | None | Not required |

When each one tends to win:

- **FAST-LIO** is more **robust in sparse / weakly-featured
  environments** (long featureless tunnels, open spaces). The filter
  accumulates small, consistent IMU evidence even when LiDAR is
  uninformative.
- **DLIO** is more accurate **under aggressive motion** (fast yaw, hard
  turns) because of per-point continuous-time deskewing. It can lose
  lock in featureless corridors where the keyframe submap lacks geometry
  for GICP to converge.
- **KISS-ICP** is **competitive when motion is smooth** (constant
  velocity is the truth most of the time). It falls behind LIO whenever
  the rover accelerates hard, yaws fast, or collides — it has no IMU to
  catch those events. It's also the cleanest sanity check: if KISS-ICP
  diverges on your data, the LiDAR alone isn't enough and you need
  inertial fusion.
- **LIO-SAM** is the one to use when **the robot revisits places**
  (loops, traversing the same corridor in both directions, exploring a
  ring). It pays a CPU + memory tax (factor graph, iSAM2, ICP
  verification of every loop candidate) — overkill for short open
  trajectories — but it's the only estimator that produces a *globally
  consistent* trajectory and map. On a few-hundred-metre run with no
  loops, expect it to perform similarly to FAST-LIO.

The Gazebo IMU is noise-free, which removes the bias-estimation
advantage filters usually have on real hardware — DLIO will look
slightly better here than it would on a physical robot. Watch the
blue (FAST-LIO), green (DLIO), and orange (KISS-ICP) paths in RViz
against the gold ground truth; the failure modes diverge in
characteristic places — see the next section.

## LIO-SAM integration notes

LIO-SAM gets a sibling container instead of being built inside the
`sim` image. Two reasons drove this split:

1. LIO-SAM uses `gtsam_unstable::IncrementalFixedLagSmoother`, and
   the Noble apt repo ships only GTSAM 4.2 with no `unstable`
   variant. The container builds **GTSAM 4.1 + unstable from
   source** to provide it.
2. Keeping it in its own container means a broken LIO-SAM build
   doesn't take down the rest of the benchmark — `docker compose up
   sim` still works.

Both containers run **ROS Jazzy + Cyclone DDS**, so messages flow
cross-container natively — no type-hash mismatch, no bridge daemon.

### Files involved

- `third_party/LIO-SAM/` — submodule (TixiaoShan/LIO-SAM, ros2 branch). Kept for source reference; the build clones a fresh copy inside the image.
- `docker/lio_sam.Dockerfile` — self-contained image: ROS Jazzy + apt build deps + GTSAM 4.1.1 + unstable from source + LIO-SAM colcon build with the two Eigen→Eigen3 sed patches.
- `config/lio_sam.yaml` — sim-specific params override (16-beam Ouster topology, `lio_sam/*` namespaced frames, LiDAR–IMU extrinsic matching FAST-LIO/DLIO, topic names pointing at `/lidar/points_lio` + `/imu`). Mounted into the lio_sam container over upstream's stock `params.yaml`.
- `launch/lio_sam_container.launch.py` — mounted into the lio_sam container and run instead of upstream's `run.launch.py`. Starts the four LIO-SAM nodes + three static TFs:
  - `explorer_r2/odom → lio_sam/map` (glue into sim's fixed frame)
  - `lio_sam/map → lio_sam/odom` (bridge — LIO-SAM only publishes this transform after the first loop closure, leaving the chain disconnected on short straight drives; the static keeps the chain valid from boot)
  - `lio_sam/base_link → lio_sam/lidar_link` (extrinsic + the chicken-and-egg TF that LIO-SAM's IMU pre-integration looks up at startup before it can publish anything itself)
- `rviz/sim.rviz` — `LIO-SAM Odom` (purple Odometry, Best Effort QoS) and `LIO-SAM Path` (purple Path), both enabled by default.

### Known quirks

- **Static `lio_sam/map → lio_sam/odom` masks LIO-SAM's loop-closure-driven correction.** When LIO-SAM detects a loop and tries to publish that transform itself, tf2 will see two publishers on the same edge and emit warnings. If you want to actually *see* loop closures snap the trajectory, drop the bridge static and accept that the chain is disconnected until the first loop fires.
- **The `explorer_r2/odom → lio_sam/map` static is offset by +1.05 m in z**, not identity. LIO-SAM tracks the LiDAR pose internally and applies our configured extrinsic to derive base_link, but its internal origin is at the first LiDAR pose — so without the compensating shift, lio_sam/base_link ends up at z=-1.05 m (underground in RViz). The shift cancels that. Works for arbitrary rover motion because the IMU↔LiDAR extrinsic is rigid.
- **First build takes ~10 minutes.** GTSAM-from-source dominates. Cached after that.

## Loop closure in 3D LiDAR-SLAM

The three estimators above are pure **odometry frontends**: they
estimate motion scan-to-scan, build a running map, and drift over
time. None of them recognise when the rover comes back to a place
it's already been. **Loop closure** is the layer that adds that
recognition, and when verified, globally corrects the trajectory so
the loop joins up. A LIO with loop closure (e.g. LIO-SAM) is what
most people mean by "LiDAR-SLAM".

A 3D LiDAR loop-closure pipeline has three stages:

1. **Place recognition.** For every new keyframe, compute a compact
   descriptor that captures "what does the LiDAR scan look like from
   here". Look the descriptor up in a database of past keyframes;
   nearest neighbours are loop candidates. This step is cheap — it
   has to run on every frame in real time.
2. **Geometric verification.** Run ICP (or a robust variant) between
   the current scan and the candidate keyframe. Most candidates are
   false positives — they look similar but aren't the same place —
   so verification with a strict ICP fitness threshold is essential.
   A single false-positive loop corrupts the whole pose graph.
3. **Pose graph correction.** Every verified loop becomes an edge in
   a factor graph alongside the odometry edges. Run an optimisation
   step (Gauss-Newton / Levenberg-Marquardt) to find the trajectory
   consistent with both. Modern implementations use **iSAM2**
   (incremental smoothing) so the graph updates online instead of
   being re-solved from scratch each loop.

### Descriptors people actually use

- **Scan Context** (Kim & Kim, 2018) — by far the most common in
  modern LIO-SLAM. Divides the horizontal plane around the LiDAR
  into $N_\text{ring} \times N_\text{sector}$ polar bins and stores
  the max $z$ height per bin. The result is a 2D "image" that's
  invariant to translation within the scan; rotation is handled by
  row-shifting during matching. A 1D "ring key" gives an O(N) prefilter
  before the more expensive 2D comparison. **LIO-SAM uses this** by
  default.
- **Scan Context++** (2021) — augments the original with additional
  invariances; better robustness to lateral motion.
- **M2DP** (Multi-view 2D Projection) — older, projects the cloud
  onto multiple 2D planes and computes an SVD-based descriptor.
  Still appears as a baseline in papers.
- **STD** (Stable Triangle Descriptor, 2023) — uses triangles formed
  by stable feature points; strong recent benchmarks. Less common in
  production than Scan Context but worth knowing about.
- **Learned descriptors** — PointNetVLAD, LCDNet. Trained on a
  dataset, can outperform hand-crafted descriptors in cluttered
  urban scenes. Less common in production because of inference cost
  and brittleness to domain shift (a model trained on KITTI
  underperforms on caves).

### Pose-graph backends

- **GTSAM** with **iSAM2** — the dominant choice in modern LIO-SLAM.
  LIO-SAM, FAST-LIO-SLAM, hdl_graph_slam all use this combination.
  Incremental smoothing means new loops are integrated in
  milliseconds, not seconds.
- **g2o** — older, still used in the ORB-SLAM family and some LIO
  derivatives. Slightly less ergonomic for large factor graphs.
- **Ceres** — general-purpose nonlinear least-squares solver; can
  do pose graphs but less optimised than GTSAM/g2o for SLAM.

### 2D vs 3D loop closure

2D loop closure projects each LiDAR scan to a horizontal plane and
compares 2D occupancy / grid patches. It's what Cartographer and
slam_toolbox use, and it's fast and effective for indoor wheeled
robots on flat floors.

For a 3D LiDAR in caves, tunnels, or outdoor terrain with non-planar
floors and overhangs, 2D projection throws away most of the
discriminating geometry — long tunnel sections all look the same in
2D. **3D descriptors (Scan Context being the canonical choice) are
the de-facto standard for the kind of environment this benchmark
targets.** Vision-based loop closure (DBoW, NetVLAD on camera images)
is a different tradition that comes from visual SLAM; hybrid
LiDAR+vision exists in research (Kimera-Multi) but isn't standard in
LIO stacks.

## Why collisions wreck LIO (and how each estimator reacts)

Drive the rover into a wall and you'll see the trajectories deteriorate
sharply for a few seconds afterwards — sometimes recovering, sometimes
not. **A surprising observation in this sim: the LiDAR-only KISS-ICP
often recovers *faster* than the IMU-coupled FAST-LIO / DLIO.** That's
the opposite of how it would go on real hardware. The cause is
specific to the way Gazebo emits IMU data; understanding it explains
both what you're seeing in RViz and why this sim is not a fair test of
IMU fusion at impact moments.

### The root cause: an IMU with no bandwidth model

The rover's IMU (`model.sdf`, sensor `imu_sensor`) declares realistic
Gaussian noise — gyro stddev ≈ 0.115 °/s, accel stddev ≈ 3 mg, plus
slow bias drift — but **no analog low-pass, no `<bandwidth>` tag, no
oversample-and-decimate**. Gazebo hands the sensor whatever
instantaneous acceleration the physics engine computed for that tick.

Real IMUs are mechanically band-limited by their MEMS resonance to a
few hundred Hz. That mechanical filter is exactly the thing that
prevents a millisecond-long contact impulse from showing up as a 50 g
acceleration sample. The sim's IMU skips that step.

So when the wheels hit something rigid, the chain is:

1. A 1 ms contact delivers Δv ≈ 0.5 m/s → that's 500 m/s² (≈ 50 g)
   for one physics tick, then back to nominal.
2. A real IMU would smear this to maybe 10–20 m/s² peak. The sim's
   IMU publishes the raw 500.
3. Whichever estimator most trusts the IMU gets hit hardest.

This is the same artefact `cave.launch.py` documents for VIO ("VIO
consistently failed by 3–10× scale in our sim recordings while
producing clean numbers on real datasets") — LIO is less sensitive
than VIO because LiDAR dominates pose estimation, but collisions push
the system into the regime where the unfiltered IMU peak starts to
matter.

### How each estimator reacts in this sim

- **KISS-ICP doesn't read the IMU at all.** The LiDAR scan after the
  bump is geometrically fine (gz `gpu_lidar` is decoupled from contact
  dynamics). Its only failure mode is its constant-velocity motion
  prediction being wrong for that scan — ICP starts from a bad initial
  guess, but the robust Geman–McClure kernel plus the adaptive
  correspondence threshold usually pull it back within a frame or two.
- **FAST-LIO propagates the state through IMU integration between
  LiDAR scans.** One 500 m/s² sample over 1 ms = +0.5 m/s velocity
  instantly = roughly +5 cm of predicted position before the next
  scan. The iterated-Kalman update has to drag the pose back ~5 cm
  against a confidently propagated prior. The filter *can* disbelieve
  the IMU (covariance grows), but only if the LiDAR scan registers
  well from the wrong starting point — and ICP-style correspondence
  searches from a 5 cm offset can lock onto the wrong wall.
- **DLIO uses continuous-time IMU integration for per-point scan
  deskew.** A spike at time $t_i$ inside the 100 ms scan means
  point $i$ gets transformed by a wildly wrong $\mathbf{T}(t_i)$,
  while point $i+1$ sampled 50 µs later (after the spike has passed)
  doesn't. The deskewed cloud is now **non-uniformly distorted** —
  not just translated, *bent*. GICP doesn't converge cleanly on a
  bent cloud because no rigid transform makes the bent geometry line
  up with the map.

### Other effects that compound the above

1. **Constant-velocity assumption breaks.** FAST-LIO and KISS-ICP
   deskew assuming roughly constant velocity over the ~100 ms scan
   period. A collision is the textbook violation: velocity reverses
   sign within the scan window. The deskewed cloud has points
   spatially smeared along the wrong direction.
2. **ICP local minima after a big jump.** All three methods start
   registration from a predicted pose and search nearby. If the
   actual motion exceeds the search radius, the optimiser locks onto
   the wrong wall and reports a confident-but-wrong pose. After
   that, incorporated scans are misaligned and the error compounds.
3. **Sensor-frame vibration.** Even without large translation, the
   LiDAR mount oscillates briefly post-impact. The LiDAR–IMU
   extrinsic we hand-coded is no longer the true value — it's
   wobbling. Every estimator here assumes it's rigid.
4. **Loss of geometric structure close to the wall.** Hitting a wall
   means the LiDAR is now pointing at it from very close. Most
   returns come from a single surface, so the scan has poor
   structure: one plane in front, nothing else useful. ICP / GICP
   need geometry in multiple directions to constrain pose — one wall
   pins down forward translation but not lateral sliding or yaw.

### Real-world caveat

The KISS-ICP-beats-LIO-at-collisions effect is **largely a sim
artefact**. On a real robot:

- The MEMS lowpass physically caps acceleration peaks.
- Wheel and chassis suspension absorb part of the impulse before the
  IMU mount sees it.
- IMU fusion almost always *helps* through collisions on hardware,
  not hurts — that's why every production LIO stack uses it.

If you care about real-robot behaviour through impacts, don't read
this benchmark literally. It tells you about each algorithm's
*sensitivity to unfiltered IMU spikes*, which is one diagnostic axis
among many.

### Mitigations in this sim

- Drive gently — the cheapest fix is not crashing.
- Increase `min_range` in each estimator's config so the rover's own
  body and immediate-wall returns are filtered before registration.
- For FAST-LIO, setting `extrinsic_est_en: true` in `lio.yaml` lets it
  co-estimate the LiDAR–IMU extrinsic and absorb some post-impact
  vibration.
- For DLIO, larger keyframe thresholds (`threshD`, `threshR` in DLIO's
  `params.yaml`) make it less twitchy at the cost of reaction time.
- For KISS-ICP, the `initial_threshold` in `config/kiss_icp.yaml`
  controls how big a pose jump it tolerates — too small and a single
  collision wipes out the run.
- Root-cause fix: add a low-pass filter on `/imu` (e.g. a tiny ROS 2
  node averaging the last $N$ samples, or an `<update_rate>` cap in
  the sensor SDF combined with `<bandwidth>` if/when gz_sim supports
  it). The right cutoff depends on the rover's mechanical resonance —
  100–200 Hz is a reasonable starting point for a wheeled robot.

## Adding another LIO library

The sim exposes the inputs every LIO expects:

| topic              | type                       | rate    |
|--------------------|----------------------------|---------|
| `/lidar/points`    | `sensor_msgs/PointCloud2`  | ~10 Hz  |
| `/imu`             | `sensor_msgs/Imu`          | ~200 Hz |
| `/ground_truth/path` + `/ground_truth/odom` | `nav_msgs/Path`, `Odometry` | ~50 Hz |

There are two integration patterns in this repo, depending on how
willing the new estimator is to build in our Jazzy + GZ Harmonic image:

### Pattern A: build inside the `sim` container (FAST-LIO, DLIO, KISS-ICP)

Used when the estimator builds cleanly with whatever apt / ROS Jazzy
packages we already have. Three steps:

1. **Drop the package in.** Add it as a git submodule under
   `third_party/<your_lio>/`, or directly into `src/<your_lio>/`. The
   container entrypoint already symlinks anything under `third_party/`
   into `src/` and runs `colcon build`.
2. **Wire the topics.** In your launch file, remap whatever the estimator
   subscribes to so it reads `/lidar/points` and `/imu`. If it needs
   Livox `CustomMsg` instead of `PointCloud2`, use the existing
   `livox_ros_driver2` shim package in `src/` as a model.
3. **Run it.** Either rebuild with `BUILD=force docker compose up`, or
   start it in a second terminal:
   ```bash
   docker compose exec sim ros2 launch <your_lio> <your.launch.py>
   ```

Worked examples in the repo:

- **DLIO** (LiDAR + IMU): `launch/dlio.launch.py` + `config/dlio.yaml`
  + the DLIO block in `docker/sim-entrypoint.sh` + the auto-include in
  `launch/cave.launch.py`. ~90 lines of glue, no source patches.
- **KISS-ICP** (LiDAR only, no IMU): `launch/kiss_icp.launch.py` +
  `config/kiss_icp.yaml` + the KISS-ICP block in
  `docker/sim-entrypoint.sh` + auto-include in `cave.launch.py`. Even
  shorter because no extrinsic config is needed.

### Pattern B: sibling container with its own Dockerfile (LIO-SAM)

Used when the estimator pins a specific dependency version (LIO-SAM
needs GTSAM 4.1 + unstable; Noble's apt ships only GTSAM 4.2 with no
unstable variant) or otherwise fights the sim image's build. Instead
of trying to backport it, give it its own image but on the **same ROS
distribution** as `sim` so cross-container DDS delivers messages
natively (no Humble↔Jazzy type-hash mismatch — we hit this when we
tried upstream's reference Humble image, and the fix was building on
Jazzy ourselves). Four steps:

1. **Submodule the upstream repo** — `git submodule add` into
   `third_party/<your_lio>/`. Kept as a source reference; the
   Dockerfile clones a fresh copy at build time.
2. **Write a self-contained Dockerfile** that uses ROS Jazzy as the
   base, installs whatever deps are needed (build from source if apt
   doesn't have the right version), clones the estimator, applies
   any patches, and runs `colcon build`. See `docker/lio_sam.Dockerfile`
   (~50 lines including GTSAM-from-source).
3. **Write a sim-specific params yaml** at `config/<your_lio>.yaml`
   with topic names, frame names, and extrinsic for our sim. Mount it
   into the container over upstream's stock config.
4. **Write a minimal launch file** that runs only the estimator nodes
   (skip upstream's `robot_state_publisher`, RViz, or any other
   extras that would pollute our TF tree). Add any static TFs the
   estimator needs internally (chicken-and-egg lookups, glue into the
   sim's fixed frame). See `launch/lio_sam_container.launch.py`.
5. **Add a second compose service.** Build from your Dockerfile, set
   `network_mode: host` and the same `ROS_DOMAIN_ID` as `sim`, mount
   your params + launch, run the launch. See the `lio_sam:` service in
   `docker-compose.yml`.

For TF: the static transforms the estimator needs (extrinsic,
fixed-frame glue) live in the **estimator's own launch file**, not in
the sim's `world.launch.py` — that way they're only present when the
estimator is running, no orphan-frame noise in default `docker compose
up sim`.

For RViz overlay (either pattern), follow the `LIO Odom` / `LIO Path`
(blue, FAST-LIO), `DLIO Odom` / `DLIO Path` (green), `KISS-ICP Odom`
(orange), or `LIO-SAM Odom` / `Path` (purple) blocks in `rviz/sim.rviz`
— duplicate one with a different colour and topic name to add a fifth
estimator. Match the publisher's QoS (LIO-SAM publishes Best Effort,
the others Reliable).

## Running on public datasets

The four estimators are decoupled from any specific dataset — the
same launch runs against any rosbag2 (or ROS 1 bag, auto-detected),
skipping Gazebo entirely. The `bag` compose service does the wiring:
plays the bag with the right topic remap for the dataset, starts the
estimators + RViz, never starts Gazebo.

```bash
cd lio_benchmark/src/explorer_r2_sim
docker compose down                              # stop sim if running

# Default: NCD-stairs from data/slam/collection 1 - newer college/ros2/stairs
docker compose --profile bag up bag lio_sam

# Any other bag — pass BAG_PATH (in-container path) and DATASET:
BAG_PATH=/data/slam/collection\ 1\ -\ newer\ college/ros2/quad_easy \
  DATASET=newer_college \
  docker compose --profile bag up bag lio_sam

# Playback speed / loop:
RATE=0.5 docker compose --profile bag up bag lio_sam        # half-speed
LOOP=true docker compose --profile bag up bag lio_sam       # loop forever
```

**Path mounts.** Two host directories are bind-mounted into the `bag`
container (see `docker-compose.yml`):

| Host directory                                       | Container path |
|------------------------------------------------------|----------------|
| `~/workspace/robotic_notes/data/`                    | `/data`        |
| `~/datasets/`                                        | `/datasets`    |

Pick either; `BAG_PATH` just needs to resolve inside the container.

**What appears in RViz.** All Path / Cloud / Camera displays are
enabled by default; the redundant per-estimator Odometry arrow
markers are off by default (toggle them on if you want them).
Frames-list view in the TF display: only `lio_sam/map` and
`lio_sam/odom` are pre-hidden because they're identity-bridges to
`explorer_r2/odom`.

**Adding another dataset.** Add one entry per dataset to
`bag.launch.py`: a `--remap` line in the `if dataset == "..."`
block (bag's topic names → `/lidar/points`, `/imu`), and a
`bag_cloud_frame` entry mapping the dataset name to the cloud's
`header.frame_id`. The cloud frame gets statically tied under
`dlio/baselink` so RViz can render `/lidar/points` (which RViz
otherwise rejects with "Could not transform"). That's it — no
new code, no new launch file.

### Most relevant public LiDAR-IMU datasets

Ranked by fit with this benchmark's setup:

| Dataset | LiDAR | IMU | Ground truth | Why useful | Format |
|---|---|---|---|---|---|
| **[Newer College](https://ori-drs.github.io/newer-college-dataset/)** (Oxford) | Ouster OS1-64 (64-beam) | Built-in 100 Hz + Alphasense | Survey-grade total station (mm) | Gold standard for LiDAR-IMU benchmarks. Handheld in college courtyards. Smallest sequence (`quad_easy`) is ~700 MB. | ROS 1 bag + ROS 2 ports |
| **[HILTI SLAM Challenge](https://hilti-challenge.com/)** (2021–2023) | Hesai PandarXT-32 / Ouster OS0-128 | Built-in | Total-station GT (mm) | Indoor multi-floor with construction-site environments. Literature reference for tight LIO eval. | ROS 1 bag |
| **[MulRan](https://sites.google.com/view/mulran-pr/dataset)** (KAIST) | Ouster OS1-64 | Xsens MTI-300 | RTK GPS | Urban + dense park, KAIST campus loops. ~5–20 GB per sequence. | Custom (KAIST tools) |
| **[KITTI](https://www.cvlibs.net/datasets/kitti/raw_data.php)** | Velodyne HDL-64E (64-beam) | OXTS 100 Hz | RTK GPS | Outdoor driving. The classic. Use `kitti2bag` to convert. | raw .bin (needs conversion) |
| **[Newer College Multi-Camera](https://ori-drs.github.io/newer-college-dataset/multi-cam/)** | Ouster OS1-128 | Built-in | Total station | Same Oxford rig with newer sensors. Includes loops for testing loop closure (LIO-SAM benefits). | rosbag2 native |

### Download commands

Newer College and KITTI distributions are on Google Drive — install
[`gdown`](https://github.com/wkentaro/gdown) (`pip install gdown`) and
use the file IDs from each dataset's download page. Newer College
sample (smallest sequence, `quad_easy` ~700 MB):

```bash
mkdir -p ~/datasets/newer_college
cd ~/datasets/newer_college
# Replace <FILE_ID> with the Google Drive ID from
# https://ori-drs.github.io/newer-college-dataset/download/
gdown --id <FILE_ID> -O quad_easy.bag
# If needed, convert ROS 1 → ROS 2:
ros2 bag convert -i quad_easy.bag -o quad_easy --output-options '{"storage_id": "sqlite3"}'
```

KITTI sequence 00 (~6 GB):

```bash
mkdir -p ~/datasets/kitti
# raw_data download script from https://github.com/utiasSTARS/pykitti
# Then convert with kitti2bag:
pip install kitti2bag
kitti2bag -t 2011_10_03 -r 0027 raw_synced .  # writes a ROS 1 bag
# Convert ROS 1 → ROS 2 if needed.
```

### Per-dataset config

Each dataset has a different LiDAR (16-beam vs 64-beam), IMU
convention, topic names, and frame_ids. Per-dataset overrides go in
`config/datasets/<name>.yaml`. See `config/datasets/newer_college.yaml`
for the template — it covers what to remap and how to override each
estimator's sensor params (LIO-SAM's `N_SCAN`, FAST-LIO's `scan_line`,
DLIO's extrinsics, etc.).

Two universal gotchas to expect with any new dataset:

1. **LiDAR field layout.** FAST-LIO + LIO-SAM need per-point `t` and
   `ring` fields; some datasets don't have them. `lidar_field_adapter.py`
   synthesises them; in `bag.launch.py` you can pipe the cloud through
   it the same way the sim does.
2. **IMU coordinate convention.** Some IMUs are Z-up, some Z-down,
   some have the gyro flipped. Each estimator has an
   `extrinsicRot`/`extrinsicRPY` block — get the rotation right or
   the bias-init never converges and the pipeline stays silent.

### Newer College — full walkthrough

Collection 1 of the dataset (Quad / Stairs sequences, ~10 GB each,
ROS 1 `.bag` format) lives under `robotic_notes/data/slam/collection 1 - newer college/`.
That directory is mounted into the sim container at `/data` via a
bind mount in `docker-compose.yml`.

**1. Convert ROS 1 → ROS 2.** ROS 2 Jazzy can't play `.bag` files
directly; convert them to rosbag2 (sqlite3) on the host:

```bash
cd ~/workspace/robotic_notes/data/slam/collection\ 1\ -\ newer\ college
mkdir -p ros2
rosbags-convert --src 2021-07-01-10-40-50_0-stairs.bag --dst ros2/stairs
# Output: ros2/stairs/{metadata.yaml, stairs.db3}  (~9 GB)
```

Repeat for the other sequences (quad-easy, quad-medium, quad-hard) as
needed.

**2. Stop the Gazebo sim** (the bag publishes the same `/lidar/points`
and `/imu` topics, so running both at once would mix the streams):

```bash
cd ~/workspace/robotic_notes/lio_benchmark/src/explorer_r2_sim
docker compose down
```

**3. Run the bag through the estimators.** A dedicated `bag` service
in `docker-compose.yml` (behind the `bag` profile) does exactly this:
plays the rosbag2 + runs FAST-LIO/DLIO/KISS-ICP/RViz, never starts
Gazebo. Pair it with `lio_sam` so all four estimators receive the bag
data:

```bash
docker compose --profile bag up bag lio_sam
```

That single command:
- Starts a `bag` container (sim image, no Gazebo, runs `bag.launch.py`)
- Starts the `lio_sam` container (always needed if you want LIO-SAM in
  the comparison)
- Does NOT start the `sim` (Gazebo) service — `sim` has no profile, so
  it only auto-starts on the default `docker compose up`

The `dataset:=newer_college` arg (baked into `bag`'s default command)
makes `bag.launch.py` remap the bag's topics (`/os_cloud_node/points`,
`/os_cloud_node/imu`) onto our canonical names (`/lidar/points`,
`/imu`) at playback time, so the estimators see them transparently.

To play a different sequence, override `BAG_PATH` (and optionally
`RATE` for non-realtime playback or `LOOP=true` to loop):

```bash
BAG_PATH=/data/slam/collection\ 1\ -\ newer\ college/ros2/quad_easy \
  docker compose --profile bag up bag lio_sam

# half-speed:
RATE=0.5 docker compose --profile bag up bag lio_sam

# loop indefinitely:
LOOP=true docker compose --profile bag up bag lio_sam
```

**4. When you're done:**

```bash
docker compose down
```

**5. Evaluate against ground truth.** TUM-format CSVs live in
`ground_truth/tum_format/` (e.g. `gt-nc-stairs.csv`). The
end-to-end flow:

```bash
# (a) Start the bag flow in one terminal (plays the bag while the four
#     estimators publish odometry):
docker compose --profile bag up bag lio_sam

# (b) In another terminal, record the four estimator odom topics for
#     the bag duration (NCD-stairs = 119s; pad to ~125s):
docker exec bag bash -lc "source /opt/ros/jazzy/setup.bash && \
  cd /tmp && timeout 125 ros2 bag record -o run1 \
    /Odometry /dlio/odom_node/odom /kiss/odometry /lio_sam/mapping/odometry"

# (c) Pull the recording out + run the eval script:
docker cp bag:/tmp/run1 /tmp/lio_eval/run1
src/explorer_r2_sim/scripts/eval_bag_replay.sh \
  /tmp/lio_eval/run1 \
  "/home/$USER/workspace/robotic_notes/data/slam/collection 1 - newer college/ground_truth/tum_format/gt-nc-stairs.csv"
```

The script writes `eval/{fast_lio,dlio,kiss_icp,lio_sam}.tum`,
`eval/summary.txt` (APE per estimator), and `eval/trajectories_*.png`
(XY overlay + per-axis vs time + RPY + speeds) next to the recorded
bag. Reference numbers from a clean run on NCD-stairs (109 s of
overlap, default in-tree configs):

| estimator | APE rmse (m) | APE max (m) | APE mean (m) | notes                            |
|-----------|-------------:|------------:|-------------:|----------------------------------|
| DLIO      |        0.197 |       0.374 |        0.187 | best on this sequence            |
| FAST-LIO  |        0.236 |       0.602 |        0.204 | close second                     |
| KISS-ICP  |        3.441 |       6.792 |        3.189 | drifts: no IMU + 16-beam configs |
| LIO-SAM   |            — |           — |            — | NCD's IMU leaves `sensor_msgs/Imu.orientation` empty → `imuPreintegration` refuses; needs a Madgwick/Mahony filter in the pipeline |

**Caveats with NCD specifically:**

- The Ouster OS1-64 is 64-beam, but our default configs are 16-beam.
  Expect drift to be larger than what published LIO-SAM/DLIO papers
  report on NCD until the per-estimator sensor params are tuned for
  64 beams (see `config/datasets/newer_college.yaml` for the values
  to override).
- The IMU extrinsic isn't auto-detected. NCD's rig has the IMU
  approximately coincident with the LiDAR, but the exact calibration
  matters — fetch it from NCD's `extrinsics/` directory in the
  Collection 1 distribution and update `config/datasets/newer_college.yaml`.
- LIO-SAM expects a specific `pointCloudTopic` and Ouster field
  layout. If you see `[lio_sam_imageProjection] Waiting for IMU data`
  forever, check the IMU coordinate convention first (NCD's IMU
  axes may be flipped relative to our default `extrinsicRot`).

## Recording a run

```bash
# from a second terminal attached to the container
docker compose exec sim bash
ros2 run explorer_r2_sim gt_to_path.py &
src/explorer_r2_sim/scripts/record_run.sh tunnel_drive_1
# drive, then Ctrl-C the recorder. Bag lands under runs/run_<UTC>/.
```

## Analysis

```bash
python3 lio_benchmark/src/explorer_r2_sim/scripts/analyze_bag.py \
    ~/ros2_ws/runs/run_<stamp>
```

Output: `summary.md` with topic rates, path lengths, end-point
errors, IMU sanity, per-axis plots.

## Tuning reference

- LiDAR sensor + FAST-LIO `lio.yaml` knobs:
  [`../vio_benchmark/docs/PARAMETERS.md` §6](../vio_benchmark/docs/PARAMETERS.md#6-lio--fast-lio-with-the-rover-sim)
- DLIO sim overrides (frames + extrinsics): `src/explorer_r2_sim/config/dlio.yaml`.
  Algorithm gains live in DLIO's own `params.yaml` (keyframe thresholds,
  GICP iteration counts, observer $K_p / K_v / K_q$, etc.) — see
  [DLIO README](third_party/DLIO/README.md).
- KISS-ICP sim overrides (max/min range + topic): `src/explorer_r2_sim/config/kiss_icp.yaml`.
  The full parameter set is tiny by design — see
  [KISS-ICP README](third_party/kiss-icp/README.md).
- Simulator GPU / rendering performance:
  [`docs/PERFORMANCE.md`](docs/PERFORMANCE.md)

## Trajectory evaluation

The same metrics and methodology used for VIO benchmarking apply:

- APE / RPE / Umeyama: [`../docs/visual_odometry/trajectory_analysis.ipynb`](../docs/visual_odometry/trajectory_analysis.ipynb)
- Diagnostic procedure when results look off: [`../vio_benchmark/docs/VIO_DIAGNOSTIC_GUIDE.md`](../vio_benchmark/docs/VIO_DIAGNOSTIC_GUIDE.md)
