import rerun.blueprint as rrb
import rerun as rr
import cv2
import numpy as np
import time
from pathlib import Path


rr.init("rerun_demo", spawn=True)
rr.log("world/xyz", rr.Arrows3D(vectors=[[1, 0, 0], [0, 1, 0],
       [0, 0, 1]], colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],),)


# ============================================================================
# Setup Camera and Webcam
# ============================================================================
resolution = [1280, 720]
fx = 848.53117539872062
fy = 848.53117539872062
cx = 639.5
cy = 359.5

cap = cv2.VideoCapture(0)
# Set camera resolution to 1280x720
cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])

# Open a video file (alternative)
# cap = cv2.VideoCapture("video.mp4")

entity_path = "world/camera"

# Log camera intrinsics (static - only once)
rr.log(
    entity_path,
    rr.Pinhole(
        resolution=[resolution[0], resolution[1]],
        focal_length=[fx, fy],
        principal_point=[cx, cy],
        # Distance from camera origin to image plane for 3D visualization
        image_plane_distance=1.0,
        color=[255, 128, 0],  # Orange color for camera frustum
        line_width=0.003  # Width of camera frustum lines
    ),
    static=True  # Camera intrinsics are static
)

# ============================================================================
# Live webcam feed with animated camera position
# ============================================================================
num_frames = 90  # Number of frames to capture

for frame_idx in range(num_frames):
    # Set time for this frame
    rr.set_time("frame", sequence=frame_idx)

    # Read webcam frame
    ret, frame = cap.read()
    if not ret:
        print("Failed to read frame from webcam")
        break

    # Convert BGR to RGB for Rerun
    img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Log image to camera (this will display in camera view)
    rr.log(entity_path, rr.Image(img_rgb))

    # Update camera position (animate in world)
    # Move camera along X-axis over time
    translation = [0.02 * frame_idx, 0, 1]
    rr.log(
        entity_path,
        rr.Transform3D(
            translation=translation,
            rotation=rr.Quaternion(xyzw=[0, 0, 0, 1])
        )
    )

    # Small delay to allow Rerun to process and control frame rate
    time.sleep(0.1)

# Release webcam
cap.release()
print(f"Captured {num_frames} frames")
