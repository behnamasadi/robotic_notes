import rerun.blueprint as rrb
import rerun as rr
import cv2
import numpy as np
import time
from pathlib import Path

# ============================================================================
# Path Management
# ============================================================================


def get_script_dir() -> Path:
    """Get the directory where this script is located."""
    return Path(__file__).resolve().parent


def get_resource_path(*parts: str) -> Path:
    """
    Get path to resource relative to script directory.
    This ensures the script works regardless of where it's run from.
    """
    return (get_script_dir() / Path(*parts)).resolve()


# ============================================================================
# Initialize Rerun
# ============================================================================
rr.init("rerun_demo", spawn=True)

# Print default coordinate system
print("Default ViewCoordinates (when not explicitly set):")
print(f"  RIGHT_HAND_Y_DOWN: {rr.ViewCoordinates.RIGHT_HAND_Y_DOWN}")
print(f"  Description: X=Right, Y=Down, Z=Forward")
print(f"  This means Y-axis points DOWN (not up)")
print()
print("Other common coordinate systems:")
print(f"  RIGHT_HAND_Z_UP: {rr.ViewCoordinates.RIGHT_HAND_Z_UP}")
print(f"    Description: X=Right, Y=Forward, Z=Up")
print(f"  RIGHT_HAND_Y_UP: {rr.ViewCoordinates.RIGHT_HAND_Y_UP}")
print(f"    Description: X=Right, Y=Up, Z=Back")
print(f"  RUB: {rr.ViewCoordinates.RUB}")
print(f"    Description: X=Right, Y=Up, Z=Back")
print()

# ============================================================================
# How to get current coordinate system
# ============================================================================
# If you set a coordinate system, you can access its coordinates like this:


def get_coordinate_system_info(vc: rr.ViewCoordinates) -> dict:
    """Get human-readable info about a ViewCoordinates object."""
    coords = vc.coordinates  # This is a numpy array [x, y, z]
    dir_names = {1: "Up", 2: "Down", 3: "Right",
                 4: "Left", 5: "Forward", 6: "Back"}
    return {
        "coordinates": coords.tolist(),
        "X": dir_names[coords[0]],
        "Y": dir_names[coords[1]],
        "Z": dir_names[coords[2]],
    }


# Example: Get info from different ViewCoordinates
print("ViewCoordinates examples:")
for name, vc in [
    ("RIGHT_HAND_Y_DOWN (default)", rr.ViewCoordinates.RIGHT_HAND_Y_DOWN),
    ("RIGHT_HAND_Z_UP", rr.ViewCoordinates.RIGHT_HAND_Z_UP),
]:
    vc_info = get_coordinate_system_info(vc)
    print(f"  {name}:")
    print(f"    Coordinates array: {vc_info['coordinates']}")
    print(
        f"    X-axis: {vc_info['X']}, Y-axis: {vc_info['Y']}, Z-axis: {vc_info['Z']}")
print()
print("Note: Rerun doesn't provide an API to query the coordinate system")
print("      from a recording. You need to track it yourself in your code.")
print("      The viewer's camera orientation can make axes appear rotated!")
print()
print("IMPORTANT: According to Rerun docs, the default is RIGHT_HAND_Y_DOWN")
print("  (X=Right, Y=Down, Z=Forward). However, if you see Z pointing UP in the viewer,")
print("  this could be due to:")
print("  1. Viewer's default camera orientation making it appear that way")
print("  2. Viewer using a different default than the SDK")
print("  3. Previous recording data still cached")
print("  The axes are drawn as raw 3D coordinates [1,0,0], [0,1,0], [0,0,1]")
print("  and the coordinate system defines how these map to world directions.")
print()

# NOT setting any coordinate system - using the default
# If you want to verify, you can check the viewer's properties panel
# for the "world" entity to see if ViewCoordinates is logged

entity_path = "text"
rr.log(entity_path, rr.TextLog("Hello Rerun"))

# ============================================================================
# Log world/axes
# ============================================================================

rr.log(
    "world/axes",
    rr.LineStrips3D(
        [
            [[0, 0, 0], [1, 0, 0]],  # X
            [[0, 0, 0], [0, 1, 0]],  # Y
            [[0, 0, 0], [0, 0, 1]],  # Z
        ],
        colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]]
    )
)


# Use proper path management (works from any working directory)
image_path = get_resource_path("images", "P1180141.JPG")


img = cv2.imread(str(image_path))


# ============================================================================
# Log image to Rerun
# ============================================================================
img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
entity_path = "disk/image"
rr.log(entity_path, rr.Image(img_rgb))

# # Send blueprint to display the image in a 2D view
# rr.send_blueprint(
#     rrb.Spatial2DView(origin=entity_path, name="Image View")
# )

# ============================================================================
# Log video
# ============================================================================
# Camera parameters from laptop_calib_result_1280x720.xml
resolution = [1280, 720]
fx = 848.53117539872062
fy = 848.53117539872062
cx = 639.5
cy = 359.5

cap = cv2.VideoCapture(0)
# Set camera resolution to 1280x720
cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])


# Open a video file
# cap = cv2.VideoCapture("video.mp4")
frame_idx = 0
entity_path = "camera/image"

while cap.isOpened():
    _, frame = cap.read()
    img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # # Set time for each frame so they appear in timeline
    # rr.set_time("video_frame", sequence=frame_idx)

    rr.log(entity_path, rr.Image(img_rgb))

    # Small delay to allow Rerun to process
    time.sleep(0.01)

    frame_idx += 1
    if frame_idx > 30:
        break

exit()
# ============================================================================
# Log Points
# ============================================================================


poinst = np.array([[0, 0, 0], [0, 1, 2,], [2, 1, 2], [1, 0, 2]])
entity_path = "world/points"
rr.log(entity_path, rr.Points3D(poinst, radii=0.1))

# ============================================================================
# Log LineStrips3D
# ============================================================================
trajectory = poinst

entity_path = "world/trajectory"

rr.log(entity_path, rr.LineStrips3D([trajectory]))

# ============================================================================
# Log Camera - animate in loop
# ============================================================================
entity_path = "world/camera"


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
    rr.Image(img_rgb),  # Add image to camera
    static=True  # Camera intrinsics and image are static
)

# Animate camera position in loop
for frame in range(20):
    rr.set_time("frame", sequence=frame)

    rr.log(
        entity_path,
        rr.Transform3D(
            translation=[0.1 * frame, 0, 1],
            rotation=rr.Quaternion(xyzw=[0, 0, 0, 1])
        )
    )

    # Small delay to allow Rerun to process
    time.sleep(0.4)
