import rerun as rr
import time
import numpy as np
import cv2
from pathlib import Path

rr.init("image_demo", spawn=True)


rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

rr.log("world/points",
       rr.Points3D([[0, 0, 0], [1, 0, 0], [0, 0, 4]], radii=0.2))


rr.log(
    "world/axes",
    rr.Arrows3D(
        vectors=[[1, 0, 0], [0, 1, 0], [0, 0, 1]],
        colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
    ),
)


rr.log(
    "world/camera",
    rr.Transform3D(
        translation=[0, 0, 1],
        rotation=rr.Quaternion(xyzw=[0, 0, 0, 1])
    )
)


# Get path relative to script location
script_dir = Path(__file__).resolve().parent
image_path = (
    script_dir / "../../data/sfm/south-building/images/P1180141.JPG").resolve()
img = cv2.imread(str(image_path))
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

# Get actual image dimensions
img_height, img_width = img.shape[:2]
print(f"Image dimensions: {img_width} x {img_height}")

# Update camera with actual image resolution and larger frustum
rr.log(
    "world/camera",
    rr.Pinhole(
        resolution=[img_width, img_height],
        focal_length=[500, 500],
        principal_point=[img_width / 2.0, img_height / 2.0],
        # Increase image_plane_distance to make frustum larger
        image_plane_distance=0.2,
        color=[255, 128, 0],  # Orange color for camera frustum
        line_width=0.005  # Width of camera frustum lines
    )
)

rr.log("world/camera", rr.Image(img))
