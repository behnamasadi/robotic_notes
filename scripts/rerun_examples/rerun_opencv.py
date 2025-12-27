import rerun as rr
import numpy as np
import cv2 as cv
from math import pi
from rerun.datatypes import Angle, RotationAxisAngle

# Load and preprocess the image
img = cv.imread(
    "/home/behnam/workspace/robotic_notes/data/sfm/south-building/images/P1180141.JPG")
if img is None:
    raise FileNotFoundError("Image not found at the specified path.")
print("img.shape:", img.shape)
height, width, _ = img.shape  # Corrected the order of dimensions
img = cv.cvtColor(img, cv.COLOR_BGR2RGB)

# Initialize Rerun session
rr.init("opencv_images", spawn=True)

# Log points
points = np.array([[1, 1, 1], [2, 2, 1]])
rr.log("points", rr.Points3D(points))

# Set view coordinates
rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

# Log arrows representing XYZ axes
rr.log(
    "world/xyz",
    rr.Arrows3D(
        vectors=[[1, 0, 0], [0, 1, 0], [0, 0, 1]],
        colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
    ),
)

# Log the camera with pinhole model
rr.log("world/camera", rr.Pinhole(focal_length=500, width=width, height=height))

# Log the camera's transform
rr.log(
    "world/camera",
    rr.Transform3D(
        translation=[1, 0, 1],
        rotation=RotationAxisAngle(axis=[1, 0, 0], angle=Angle(rad=-pi / 4)),
    ),
)

# Log the image
rr.log("world/camera/image/rgb", rr.Image(img))

# rr.set_time_sequence

# https://rerun.io/docs/reference/types/archetypes/image
# https://rerun.io/docs/reference/types/archetypes/transform3d
# https://rerun.io/docs/concepts/timelines

# rr.log(
#     "world/cam",
#     rr.Pinhole(fov_y=0.7853982, aspect_ratio=1.7777778, camera_xyz=rr.ViewCoordinates.RUB, image_plane_distance=0.1),
# )
