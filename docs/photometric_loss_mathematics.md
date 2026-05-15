# Photometric Loss: Mathematical Formulation

## Overview

This document explains the mathematical equations used in the photometric loss calculation for self-supervised monocular depth estimation, following the approach used in Monodepth2 and similar works.

---

## 1. Dataset Structure

### KITTI Odometry Dataset

- **Images**: `N = 4541` images (sequences/00/image_2/XXXXXX.png)
- **Poses**: `N = 4541` camera poses (poses/00.txt)
- **Calibration**: Camera intrinsic matrix K (data_odometry_calib/calib.txt)

**Important**: Each pose `T_w_i` represents the transformation from camera frame `i` to the world frame `w`.

---

## 2. Camera Intrinsic Matrix

The camera intrinsic matrix K projects 3D camera coordinates to 2D pixel coordinates:

```
K = [fx   0   cx]
    [0   fy   cy]
    [0    0    1]
```

Where:
- `fx, fy`: focal lengths (in pixels)
- `cx, cy`: principal point (image center)

**From KITTI calibration file (P2 projection matrix)**:

```
K = [718.856    0.0    607.1928]
    [  0.0    718.856  185.2157]
    [  0.0      0.0      1.0   ]
```

**Inverse intrinsic matrix**:

```
K_inv = [1/fx    0    -cx/fx]
        [  0   1/fy   -cy/fy]
        [  0     0       1   ]
```

---

## 3. Camera Pose Representation

### 3.1 Absolute Poses

Each line in `poses/00.txt` contains 12 values representing a 3×4 transformation matrix:

```
r11 r12 r13 tx  r21 r22 r23 ty  r31 r32 r33 tz
```

This is converted to a 4×4 homogeneous transformation matrix:

```
T_w_i = [r11  r12  r13  tx]
        [r21  r22  r23  ty]
        [r31  r32  r33  tz]
        [ 0    0    0   1 ]
```

Where:
- **R** = [r11 r12 r13; r21 r22 r23; r31 r32 r33] is the rotation matrix (3×3)
- **t** = [tx; ty; tz] is the translation vector (3×1)
- **T_w_i**: Transforms from camera frame `i` to world frame `w`

### 3.2 Relative Pose Computation

For photometric loss, we need the **relative transformation** between two frames.

Given:
- `T_w_t`: World pose of target frame (e.g., frame 0)
- `T_w_s`: World pose of source frame (e.g., frame 1)

The relative transformation from target to source is:

```
T_t→s = T_w_s^(-1) · T_w_t
```

**Interpretation**: This transforms points from target camera frame to source camera frame.

**In code**:
```python
T_target_to_source = np.linalg.inv(T_w_source) @ T_w_target
```

---

## 4. Image Warping (Projective Transformation)

### 4.1 Goal

Warp source image `I_s` to the target camera viewpoint to create `I_warped`, which should match target image `I_t` if depth is correct.

### 4.2 Pipeline

```
Pixel (u,v) → Camera coords → 3D point → Transform → Project → Warped pixel (u',v')
```

### 4.3 Detailed Steps

#### Step 1: Create Pixel Coordinate Grid

For each pixel (u, v) in the target image, create homogeneous coordinates:

```
p = [u]
    [v]
    [1]
```

For entire image (H×W):

```
P_homo ∈ ℝ^(3×H×W)
```

#### Step 2: Back-project to 3D Camera Coordinates

Using inverse intrinsics and depth:

```
X_cam_t = D_t · (K^(-1) · p)
```

Where:
- `D_t(u,v)`: Depth at pixel (u,v) in target frame
- `X_cam_t`: 3D point in target camera coordinates

In homogeneous coordinates:

```
X_cam_t = [X]     D_t(u,v) · [1/fx    0    -cx/fx] [u]
          [Y]  =            [  0   1/fy   -cy/fy] [v]
          [Z]               [  0     0       1   ] [1]
          [1]
```

Explicitly:

```
X = D_t(u,v) · (u - cx) / fx
Y = D_t(u,v) · (v - cy) / fy
Z = D_t(u,v)
```

#### Step 3: Transform to Source Camera Frame

Apply relative transformation:

```
X_cam_s = T_t→s · X_cam_t
```

Where `T_t→s` is the 4×4 transformation matrix:

```
[X_s]   [R_t→s  t_t→s] [X_t]
[Y_s] = [             ] [Y_t]
[Z_s]   [ 0 0 0   1  ] [Z_t]
[1 ]                    [ 1 ]
```

Expanded:

```
X_s = R_11·X_t + R_12·Y_t + R_13·Z_t + tx
Y_s = R_21·X_t + R_22·Y_t + R_23·Z_t + ty
Z_s = R_31·X_t + R_32·Y_t + R_33·Z_t + tz
```

#### Step 4: Project to Source Image Plane

Project 3D point in source frame to source image pixels:

```
p_s = K · [X_s/Z_s]
          [Y_s/Z_s]
          [  1    ]
```

Which gives:

```
u' = fx · (X_s / Z_s) + cx
v' = fy · (Y_s / Z_s) + cy
```

#### Step 5: Sample Source Image

Use bilinear interpolation to sample source image at fractional coordinates (u', v'):

```
I_warped(u,v) = BilinearSample(I_s, u', v')
```

**Complete Warping Equation**:

```
I_warped(u,v) = I_s(π(K · T_t→s · D_t(u,v) · K^(-1) · [u,v,1]^T))
```

Where `π` is the perspective projection operator:

```
π([X, Y, Z]^T) = [X/Z, Y/Z]^T
```

---

## 5. Photometric Loss Calculation

### 5.1 SSIM (Structural Similarity Index)

For two images (or patches) `I_t` and `I_warped`:

```
SSIM(I_t, I_warped) = (2μ_t·μ_w + C₁)(2σ_tw + C₂) / ((μ_t² + μ_w² + C₁)(σ_t² + σ_w² + C₂))
```

Where:
- `μ_t, μ_w`: Mean intensities
- `σ_t², σ_w²`: Variances
- `σ_tw`: Covariance
- `C₁ = (0.01)² = 0.0001`: Stability constant
- `C₂ = (0.03)² = 0.0009`: Stability constant

**SSIM Loss** (normalized to [0,1], where 0 is perfect match):

```
L_SSIM = (1 - SSIM) / 2
```

### 5.2 L1 Photometric Error

```
L_L1 = |I_t - I_warped|
```

Per-pixel absolute difference.

### 5.3 Combined Photometric Loss

```
L_photo = α · L_SSIM + (1 - α) · L_L1
```

Where `α = 0.85` (standard in Monodepth2)

**Per-pixel loss map**:

```
L(u,v) = 0.85 · L_SSIM(u,v) + 0.15 · L_L1(u,v)
```

**Total loss** (averaged over all valid pixels):

```
L_total = (1 / N_valid) · Σ L(u,v)
```

---

## 6. Complete Pipeline Summary

### Input:
1. Target image: `I_t` (frame i)
2. Source image: `I_s` (frame j)
3. Predicted depth: `D_t` for target frame
4. Camera intrinsics: `K`
5. Poses: `T_w_i`, `T_w_j`

### Process:

**Step 1**: Compute relative pose
```
T_t→s = T_w_j^(-1) · T_w_i
```

**Step 2**: For each pixel (u,v) in target image:
```
a) Back-project: X_cam_t = D_t(u,v) · K^(-1) · [u,v,1]^T
b) Transform:    X_cam_s = T_t→s · [X_cam_t; 1]
c) Project:      [u',v']^T = π(K · X_cam_s)
d) Sample:       I_warped(u,v) = I_s(u',v')
```

**Step 3**: Compute photometric loss
```
L = 0.85 · L_SSIM(I_t, I_warped) + 0.15 · L_L1(I_t, I_warped)
```

---

## 7. Implementation Details

### 7.1 Vectorized Implementation

Instead of looping over pixels, the implementation uses PyTorch tensors:

```python
# Shape notations:
# B = batch size
# H = image height  
# W = image width

# Input shapes:
I_s:     [B, 3, H, W]  # Source RGB image
I_t:     [B, 3, H, W]  # Target RGB image
depth_t: [B, 1, H, W]  # Predicted depth
K:       [B, 3, 3]     # Intrinsics
T_t2s:   [B, 4, 4]     # Relative transform
```

### 7.2 Grid Sampling

PyTorch's `F.grid_sample` expects normalized coordinates in [-1, 1]:

```python
u_norm = 2 * (u / (W - 1)) - 1
v_norm = 2 * (v / (H - 1)) - 1
```

---

## 8. Coordinate Systems

### World Frame (w)
- Fixed reference frame
- Poses are given relative to this

### Camera Frame (c)
- Origin at camera optical center
- Z-axis pointing forward
- X-axis pointing right
- Y-axis pointing down

### Image Frame (pixels)
- Origin at top-left corner
- u-axis (horizontal) pointing right
- v-axis (vertical) pointing down

---

## 9. Key Assumptions

1. **Static Scene**: Objects don't move between frames
2. **Lambertian Surfaces**: Photometric consistency assumption
3. **Known Intrinsics**: Camera calibration is accurate
4. **Known Poses**: Ground truth poses from KITTI (in training, these would be from pose network)

---

## 10. References

1. Godard et al. "Digging Into Self-Supervised Monocular Depth Estimation" (Monodepth2), ICCV 2019
2. KITTI Vision Benchmark Suite: http://www.cvlibs.net/datasets/kitti/
3. Multiple View Geometry in Computer Vision (Hartley & Zisserman)

---

## Appendix: Matrix Dimensions

```
Symbol      Dimension    Description
------      ---------    -----------
I_t         [B,3,H,W]    Target RGB image
I_s         [B,3,H,W]    Source RGB image  
I_warped    [B,3,H,W]    Warped source image
D_t         [B,1,H,W]    Depth map (target)
K           [B,3,3]      Camera intrinsics
K_inv       [B,3,3]      Inverse intrinsics
T_w_i       [B,4,4]      World to camera i
T_w_j       [B,4,4]      World to camera j
T_t→s       [B,4,4]      Target to source transform
p           [3,1]        Pixel homogeneous coords
X_cam       [4,1]        3D point homogeneous coords
L_photo     [B,1,H,W]    Photometric loss map
```

---

*Generated for PyTorch self-supervised depth estimation implementation*

