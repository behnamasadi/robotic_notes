# Getting sensor rendering on the GPU

By default, `gz sim` inside a Docker container can silently fall back to
software rasterisation (Mesa llvmpipe). When that happens, every sensor
frame — camera images, lidar scans — is rendered on the CPU, and RTF
collapses no matter how fast your GPU is. This page explains how to make
sure rendering actually reaches the NVIDIA GPU through EGL.

## How it's wired

Three things have to line up so gz-rendering's Ogre2 backend reaches the
NVIDIA driver via EGL (not GLX — see pitfall below).

1. **Dockerfile** installs `libglvnd0 libegl1 libgl1 libgles2 libglx0
   mesa-utils` so GLVND can dispatch to a vendor ICD, and so `eglinfo` /
   `glxinfo` are available for debugging.
2. **`docker-compose.yml`** env vars:
   ```yaml
   __EGL_VENDOR_LIBRARY_FILENAMES: /usr/share/glvnd/egl_vendor.d/10_nvidia.json
   OGRE_RTT_MODE: FBO
   GZ_HEADLESS_RENDERING: "1"
   ```
   These point GLVND at NVIDIA's EGL ICD and force Ogre2's render-to-texture
   path through framebuffer objects — the path the nvidia-container runtime
   supports without an X server.
3. **NVIDIA Container Toolkit** on the host, and the compose file requests
   the GPU under `deploy.resources.reservations.devices` (`driver: nvidia`).

## Verify

Inside the container:

```bash
eglinfo | head      # EGL vendor string should say NVIDIA, not Mesa
nvidia-smi          # gz-sim process should appear, GPU mem nonzero
```

On the host while the sim runs:

```bash
nvidia-smi -l 1     # GPU utilisation should rise once sensors start streaming
```

If `nvidia-smi` shows the GPU idle while gz-sim is using a lot of CPU,
rendering has fallen back to software — re-check the three items above.

## Pitfall: don't set `__GLX_VENDOR_LIBRARY_NAME=nvidia`

That forces gz-sim onto NVIDIA's GLX extension, which requires the NVIDIA
display driver on the X server `DISPLAY` points at. Inside the container
there isn't one, and gz-sim crashes with:

```
X Error of failed request: BadValue, Major opcode 156 (NV-GLX)
```

For headless containerised rendering, the EGL path is the right one.

## Other knobs worth knowing

- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` — Cyclone defaults to
  shared-memory loopback for same-host comms, near-zero-copy for
  image-sized payloads. Fast-DDS otherwise sends image frames through UDP
  loopback.
- Trim bridge channels you don't consume (e.g. `/rs_*/depth`,
  `/rs_*/points`) in `config/bridge.yaml`. Each enabled channel costs one
  full memcpy per frame in `parameter_bridge`.
