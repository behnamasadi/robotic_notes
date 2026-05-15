# Simulator performance — why RTF dies and what to do about it

This document explains why `gz sim` Real-Time-Factor (RTF) can collapse to
~0.2 on a perfectly capable workstation, what to measure to find the actual
bottleneck, and what to change to recover it. The triggering incident was
run `v7` (`runs/run_20260514T221547Z/`): `/rs_front/image` and
`/rs_front_right/image` both publishing at **6.22 Hz** instead of their
configured 30 Hz, which corresponds to an RTF of ~0.21 across the whole
recording.

A 12.5× scale blow-up on OpenVINS in the same run was a *symptom* of this
— stereo VIO without enough temporal coverage triangulates badly — not a
root cause in the estimator itself.

## Status — fixes landed 2026-05-15

Three changes shipped in sequence; each one measured against the previous.

| signal                       | v7 baseline | + EGL fix | + Cyclone DDS + strip 3 cams | + bridge depth/points trim |
|------------------------------|-------------|-----------|-------------------------------|----------------------------|
| `/rs_front/image` rate       | 6.22-6.84 Hz| 13.7 Hz   | 25.3 Hz                       | **28.4 Hz**                |
| `/rs_front_right/image` rate | 5.86 Hz     | 16.2 Hz   | 24.0 Hz                       | **27.8 Hz**                |
| gz-sim RTF (recent)          | ~0.1        | ~1.0      | 0.5-1.0                       | 0.4-1.0                    |
| `nvidia-smi` GPU util        | 0 %         | 40 %      | 29-35 %                       | ~30 %                      |
| GPU memory used              | 15 MiB      | 1358 MiB  | 837 MiB                       | 837 MiB                    |
| EGL renderer string          | Mesa llvmpipe | NVIDIA RTX 3090 | NVIDIA RTX 3090         | NVIDIA RTX 3090            |
| `ruby` (gz-sim) CPU          | 200 %       | 300 %     | 270 %                         | 280 %                      |
| `parameter_bridge` CPU       | 60 %        | 70 %      | 70 %                          | **30 %** (half the channels) |

Confirmed inside the container with `eglinfo`:

```
EGL vendor string: NVIDIA
OpenGL core profile renderer: NVIDIA GeForce RTX 3090/PCIe/SSE2
```

The root cause was that `gz sim` was software-rasterising every sensor
frame through Mesa's llvmpipe because GLVND had no NVIDIA EGL vendor ICD
loaded. The fix (see `docker-compose.yml` env block and
`docker/sim.Dockerfile`):

1. Install `libglvnd0 libegl1 libgl1 libgles2 libglx0 mesa-utils` in the
   image so GLVND can dispatch to a vendor ICD and so `eglinfo`/`glxinfo`
   are available for debugging.
2. Point GLVND at the NVIDIA EGL ICD with
   `__EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json`.
3. Force ogre2's render-to-texture path through framebuffer objects (EGL)
   with `OGRE_RTT_MODE=FBO` and `GZ_HEADLESS_RENDERING=1`.

Notably we do **not** set `__GLX_VENDOR_LIBRARY_NAME=nvidia` — an earlier
attempt did, and gz-sim crashed with `X Error of failed request: BadValue,
Major opcode 156 (NV-GLX)`. That env var forces gz-sim onto NVIDIA's GLX
extension which requires the NVIDIA display driver on whatever X server
`DISPLAY` points at; the container has no such X server. EGL-only is the
right path for headless containers — same path the nvidia-container
runtime is built around.

**RTF is now real-time but cameras are still under 30 Hz.** The next
bottleneck is most likely `parameter_bridge` (single-threaded per topic,
70 % of one core in this measurement) or gz-sim's lockstep sensor
scheduling, not rendering. Those are the items below in "Original
hypothesis taxonomy"; they're worth revisiting now that the rendering
floor has been lifted.

## TL;DR — measured root cause

Headless test on 2026-05-15 with `gui:=false rviz:=false joy:=false` on
this workstation (Ryzen-class CPU + RTX 3090) showed:

| signal                                        | value                                        |
|-----------------------------------------------|----------------------------------------------|
| `/rs_front/image` rate                        | **6.84 Hz** (nominal 30 Hz)                  |
| `/rs_front_right/image` rate                  | 5.86 Hz                                      |
| `/imu` rate                                   | 119.7 Hz (healthy)                           |
| gz-sim RTF                                    | 0.10 – 0.82, mostly **~0.1**                 |
| `nvidia-smi` GPU utilization                  | **0 %**, 15 MiB / 24576 MiB used             |
| Top CPU consumer: `ruby` (gz-sim main)        | **200 %** (two cores pegged)                 |
| `parameter_bridge`                            | 60 % CPU                                     |
| `python3` (launch helpers)                    | 40 % + 20 % CPU                              |
| Estimators (fastlio/ov_msckf/vins) each       | 10-20 % CPU                                  |

**Diagnosis: the 3090 is sitting idle while `ruby` burns 200 % CPU.**
Gazebo is software-rendering the sensor cameras (and the world) on the
CPU through llvmpipe / Mesa, not on the GPU. Camera FPS at ~6 Hz with
two 640×480 streams is exactly the throughput a CPU rasteriser reaches
on this hardware. With actual GPU acceleration, hundreds of FPS are
typical.

This was confirmed by `nvidia-smi` reporting 0 % GPU utilization with
only 15 MiB of VRAM in use throughout the run — gz-sim isn't even
loading textures onto the GPU, let alone rendering on it. **Turning off
the GUI changed almost nothing** (`v7` was 6.22 Hz with GUI on; the
headless retest was 6.84 Hz). The GUI hypothesis from the original
draft of this doc was wrong; the renderer was already idle on the GPU.

## What to change (next session)

The container has the NVIDIA driver libraries (`libnvidia-*` visible in
`ldconfig`) and `nvidia-smi` works inside it, so the driver is reachable.
What's missing is gz-rendering's path *to* the driver: ogre2's default
rendering backend is GLX (X11), and even with `DISPLAY` set the headless
sensor cameras render through an offscreen context that on NVIDIA needs
EGL, not GLX, to reach the GPU.

Things to try, in order:

1. **Force EGL backend for sensor cameras**, e.g. set in the compose env:
   ```
   __EGL_VENDOR_LIBRARY_FILENAMES: /usr/share/glvnd/egl_vendor.d/10_nvidia.json
   __GLX_VENDOR_LIBRARY_NAME: nvidia
   OGRE_RTT_MODE: FBO
   ```
   and pass `--render-engine ogre2 --render-engine-gui-api-backend opengl`
   to `gz sim` (or set `GZ_SIM_RENDER_ENGINE_API_BACKEND=opengl`).

2. **Verify GLVND vendor file is present in container**: it lives at
   `/usr/share/glvnd/egl_vendor.d/10_nvidia.json` — if it's missing the
   `libglvnd0` / `libnvidia-egl-wayland1` packages need to be installed
   in the Dockerfile.

3. **Install glxinfo + eglinfo in the container** so future probes can
   confirm which renderer/vendor is actually in use. Adding
   `mesa-utils` to the Dockerfile is enough.

4. **Run `vkcube` or `glmark2` inside the container** as a quick sanity
   check that hardware acceleration works at all — if those run at
   60+ FPS, the problem is gz-sim's renderer choice; if those also run
   software, the container's GL plumbing is broken.

5. **As a last resort**, run gz-sim with `--headless-rendering` (gz-sim
   command-line flag) which forces EGL-based offscreen rendering.

Once the GPU is actually being used, expect cameras to jump to their
nominal 30 Hz and RTF to recover to near 1.0 without changing
resolution, rate, or any algorithm config. The previous suggestions in
this doc (drop resolution, drop rate, switch DDS) are still valid but
should be tried **after** the GPU-rendering path is fixed — fixing the
real bottleneck first may make them unnecessary.

## Original hypothesis taxonomy (kept for reference)

These are the bottlenecks worth checking on a *different* setup, or in
case the GPU-rendering fix isn't sufficient. The headless retest above
ruled out #1 (GUI) but the others are still on the table for future
runs.

### What can slow gz-sim, in rough order of likelihood

1. **The Gazebo GUI is a second renderer.** With `gui:=true` gz-sim runs
   a full Ogre2 render pipeline for the GUI window *in addition to* the
   headless sensor cameras. They share one GL context, so GUI repaints
   compete directly with sensor frames. ~~First thing to try.~~ — **on
   this workstation the 2026-05-15 headless retest showed `gui:=false`
   gave essentially identical numbers (6.84 vs 6.22 Hz), so GUI is
   ruled out here. Still worth considering on other setups.**

2. **ros_gz_bridge is single-threaded per topic and memcpys every frame.**
   Each `parameter_bridge` process pegs one core. With two stereo cameras
   at 30 Hz × 640×480×3 bytes that's ~55 MB/s of per-frame copy + DDS
   publish work; if the bridge can't keep up the sim throttles. Visible
   in `htop` as `parameter_bridge` at 100 % on one core each.

3. **DDS over loopback without shared-memory transport.** Default
   Fast-DDS uses UDP loopback for large messages — works, but every byte
   goes through the kernel networking stack. Enabling shared-memory
   transport (point `FASTRTPS_DEFAULT_PROFILES_FILE` at a profile with
   `<transport_descriptors><transport_descriptor><kind>SHM</kind>`) drops
   per-frame latency considerably for image-sized payloads.

4. **gz-sim's lockstep sensor scheduling.** By default the physics step
   waits for sensor render completion before advancing sim time, so one
   slow consumer (an estimator that blocks in its callback, an RViz
   display that struggles to keep up) stalls the simulation clock. The
   sim never "gets ahead" of the slowest subscriber.

5. **GPU passthrough quality inside Docker.** Even with the nvidia
   runtime, EGL plumbing and X-forwarding inside the container can add
   overhead. If `nvidia-smi` (run on the host) shows the GPU sitting
   below ~30 % during the run, GPU is *not* the bottleneck and you can
   stop investigating that side.

## Diagnostic procedure (15 min)

Goal: identify which of the five culprits above is actually responsible
on this machine, instead of guessing.

```bash
# Terminal 1: launch the sim as usual
cd ~/ros2_ws/src/explorer_r2_sim
BUILD=force docker compose up
```

```bash
# Terminal 2: who's actually busy?
docker compose exec sim htop                        # which processes peg cores?
docker compose exec sim ros2 topic hz /rs_front/image   # measured camera rate
```

```bash
# Terminal 3 (host): GPU utilization
nvidia-smi -l 1
```

Then run again with the GUI off to see if that alone fixes things:

```bash
docker compose exec sim ros2 launch explorer_r2_sim cave.launch.py gui:=false rviz:=false
```

The single-command capture is at
`src/explorer_r2_sim/scripts/perf_probe.sh` — runs all three probes for
a 10-second window and prints a one-screen summary. Use it during the
next bag recording so we have ground-truth numbers next time RTF looks
suspect.

### What the results mean

| Symptom                                                                  | Likely cause                                | Fix                                                                       |
|--------------------------------------------------------------------------|---------------------------------------------|---------------------------------------------------------------------------|
| **`nvidia-smi` GPU at 0 % AND `ruby` (gz-sim) at 200 % CPU**             | **gz-sim falling back to software renderer** | **Force EGL backend / `--headless-rendering` (see "What to change")**     |
| RTF jumps when `gui:=false`                                              | GUI renderer                                | Headless during recordings; only open GUI for inspection                  |
| `parameter_bridge` pegged at 100 % on one core per image topic           | Bridge CPU-bound                            | Lower camera rate or resolution, enable image_transport compression       |
| `nvidia-smi` GPU at <30 % and CPU also idle, RTF still low               | DDS / lockstep                              | Try `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, or SHM Fast-DDS profile      |
| `nvidia-smi` GPU at ~100 %                                               | Actually rendering-bound (unusual)          | Drop camera resolution; check whether the GUI is also rendering          |

## Action items / quick wins, in order of expected impact

These are reordered after the 2026-05-15 finding. The original list led
with `gui:=false`; the new evidence puts it last.

1. ~~**Fix gz-sim's GPU rendering path**~~ — **done 2026-05-15** (EGL pivot).
2. ~~**Switch RMW to Cyclone DDS** + strip 3 unused RGBD cameras~~ — done
   2026-05-15. Near-zero-copy SHM loopback + 3× less rendering work brought
   cameras from 14 → 25 Hz.
3. ~~**Drop unused depth + points bridge channels**~~ — done 2026-05-15.
   parameter_bridge CPU halved (70 → 30 %); cameras to 28 Hz. Re-enable
   either channel in `config/bridge.yaml` (commented blocks) if a downstream
   consumer needs it.

The combined effect of items 1-3: cameras 6 → 28 Hz (4.1×), RTF 0.1 → ~0.9.
The remaining ~2 Hz gap to nominal 30 Hz is gz-sim's lockstep sensor
scheduling — physics waits on sensor render every tick.

Further items below are no longer urgent. Leaving them documented for the
day someone wants to chase the last 5 % or runs into a new bottleneck.
2. **Move LIO to a separate compose service** (or simply skip it during
   VIO-focused runs by not building `third_party/FAST_LIO`). LIO is the
   heaviest single subscriber after gz-sim itself.
3. **Drop nominal camera rate from 30 → 20 Hz** in `model.sdf`. Stereo
   VIO is happy with 20 Hz; the bridge is happier with 33 % less work.
   *Probably unnecessary once item 1 is fixed.*
4. **Drop camera resolution from 640×480 → 424×240** if (1)+(2)+(3)
   aren't enough. Requires updating the four calibration YAMLs:
   `kalibr_imucam_chain.yaml`, `vins/cam0.yaml`, `vins/cam1.yaml`, plus
   the SDF. New intrinsics: fx=fy=367.5, cx=212, cy=120.
5. **Try CycloneDDS.** `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` in the
   container env — often faster than Fast-DDS for image traffic without
   any profile tuning.
6. **Record with `gui:=false rviz:=false`.** Free, harmless, and good
   hygiene even though on this workstation it didn't change the rate.

## What we should NOT do

- **Switch to fisheye cameras.** Gazebo supports them via the
  `wideanglecamera` sensor type with `<lens><type>equidistant</type>`,
  but cube-map rendering uses six render targets per frame — more
  expensive than pinhole, not less. Won't help RTF; would only help if
  the algorithm needs FOV it doesn't currently have.
- **Add more cameras "to compensate".** Whatever is bottlenecking the
  pipeline scales with sensor count.
- **Tune VIO parameters to fight the symptom.** A 12× scale blow-up
  isn't a tracker-knob problem; it's the algorithm being given starved
  input. Fix the input first.

## Related

- [`ANALYSIS.md`](./ANALYSIS.md) — per-run VIO/LIO trajectory analysis,
  including the v7 case study that prompted this document.
- [`PARAMETERS.md`](./PARAMETERS.md) — what every knob in
  `estimator_config.yaml` and `vins.yaml` does, with safe ranges.
