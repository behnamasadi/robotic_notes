import rerun as rr
import numpy as np
import time
import cv2

# ============================================================
# 1. Init Rerun
# ============================================================
rr.init("opencv_to_rerun_pinhole", spawn=True)

# World coordinate system:
# X = right, Y = forward, Z = up
rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

# ============================================================
# 2. World axes (for orientation sanity check)
# ============================================================
rr.log(
    "world/axes",
    rr.Arrows3D(
        vectors=[[1, 0, 0], [0, 1, 0], [0, 0, 1]],
        colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
    ),
)

# ============================================================
# 3. 3D points in OpenCV camera frame
# OpenCV: X=right, Y=down, Z=forward
# ============================================================
points_cv = np.array([
    [0.0,  0.0, 2.0],   # straight ahead
    [0.5,  0.0, 2.0],   # right
    [-0.5, 0.0, 2.0],   # left
    [0.0,  0.5, 2.0],   # down
])

# ============================================================
# 4. Convert OpenCV → Rerun world
# World: X=right, Y=forward, Z=up
# ============================================================
points_world = np.column_stack([
    points_cv[:, 0],    # X stays X
    points_cv[:, 2],    # Z (forward) → Y
    -points_cv[:, 1],   # -Y (down) → Z (up)
])

print("OpenCV points:\n", points_cv)
print("World points:\n", points_world)

# ============================================================
# 5. Log 3D points
# ============================================================
rr.log(
    "world/points",
    rr.Points3D(points_world, radii=0.06),
)

# ============================================================
# 6. OpenCV pinhole intrinsics
# ============================================================
width, height = 640, 480
fx, fy = 500.0, 500.0
cx, cy = width / 2.0, height / 2.0

K = np.array([
    [fx,  0, cx],
    [0, fy, cy],
    [0,  0,  1],
])

# ============================================================
# 7. Project points with OpenCV (camera frame)
# ============================================================
rvec = np.zeros((3, 1))
tvec = np.zeros((3, 1))
dist = np.zeros(5)

image_points, _ = cv2.projectPoints(
    points_cv,
    rvec,
    tvec,
    K,
    dist,
)

image_points = image_points.reshape(-1, 2)

print("Projected image points:\n", image_points)

# ============================================================
# 8. Camera pose: FIX orientation so camera looks forward
#
# We want:
#   camera forward → +Y_world
#   camera right   → +X_world
#   camera up      → +Z_world
#
# This transform maps OpenCV camera axes into world axes.
# ============================================================
rr.log(
    "world/camera",
    rr.Transform3D(
        mat3x3=[
            [1,  0,  0],   # camera X → world X
            [0,  0,  1],   # camera Z → world Y
            [0, -1,  0],   # camera -Y → world Z
        ],
        translation=[0.0, 0.0, 0.0],
    ),
    static=True,
)

# ============================================================
# 9. Log pinhole camera (Rerun draws frustum automatically)
# ============================================================
rr.log(
    "world/camera",
    rr.Pinhole(
        resolution=[width, height],
        focal_length=fx,
        principal_point=[cx, cy]
    ),
)

# ============================================================
# 10. Create image with projected points and log to camera
# ============================================================
# Create blank image
image = np.zeros((height, width, 3), dtype=np.uint8)

# Draw projected points on image
for pt in image_points:
    x, y = int(pt[0]), int(pt[1])
    if 0 <= x < width and 0 <= y < height:
        cv2.circle(image, (x, y), 5, (0, 255, 0), -1)  # Green filled circle
        cv2.circle(image, (x, y), 8, (255, 255, 255), 2)  # White outline

# Convert BGR to RGB for Rerun
image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

# Log image to camera entity (this will display in camera view)
rr.log("world/camera", rr.Image(image_rgb))
