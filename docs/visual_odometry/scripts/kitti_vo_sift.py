import cv2
import numpy as np
import glob
import os
from kitti_calibration import load_and_decompose_calibration
import rerun as rr
from rerun.datatypes import Quaternion


def rotation_matrix_to_quaternion(R):
    """Convert a 3x3 rotation matrix to a quaternion [x, y, z, w]."""
    w = np.sqrt(1.0 + R[0, 0] + R[1, 1] + R[2, 2]) / 2.0
    x = (R[2, 1] - R[1, 2]) / (4.0 * w)
    y = (R[0, 2] - R[2, 0]) / (4.0 * w)
    z = (R[1, 0] - R[0, 1]) / (4.0 * w)
    return [x, y, z, w]


# Use relative path based on the current script directory
script_dir = os.path.dirname(os.path.abspath(__file__))


# Path to KITTI dataset calibration file, Relative to script directory
calibration_file = "../../data/kitti/odometry/05/calib.txt"
calibration_file_abs_path = os.path.join(script_dir, calibration_file)
print("reading calibration file from: ", calibration_file_abs_path)
decomposed_data = load_and_decompose_calibration(calibration_file_abs_path)


# Print the decomposed results
for camera, data in decomposed_data.items():
    print(f"Camera: {camera}")
    print("Projection Matrix:")
    print(data["Projection Matrix"])
    print("Intrinsic Matrix:")
    print(data["Intrinsic Matrix"])
    print("Rotation Matrix:")
    print(data["Rotation Matrix"])
    print("Translation Vector:")
    print(data["Translation Vector"])
    print("-")


# Path to KITTI dataset images, Relative to script directory
image_path = "../../data/kitti/odometry/05/image_0/*.png"
image_path_abs_path = os.path.join(script_dir, image_path)
print("reading images from: ", image_path_abs_path)

calibration_file_abs_path = os.path.join(script_dir, calibration_file)
print(calibration_file_abs_path)
decomposed_data = load_and_decompose_calibration(calibration_file_abs_path)
image_files = sorted(glob.glob(image_path_abs_path))

# Camera intrinsic parameters since we are using left camera gray, it is P0
camera_name = "P0"
camera_P0 = decomposed_data[camera_name]
K = camera_P0["Intrinsic Matrix"]
fx = K[0, 0]
fy = K[1, 1]

cx = K[0, 2]
cy = K[1, 2]
print("Intrinsic Matrix P0:", K)

print(fx)
print(fy)
print(cx)
print(cy)

# Parameters
feature_params = dict(maxCorners=1000, qualityLevel=0.01,
                      minDistance=10, blockSize=7)
lk_params = dict(winSize=(21, 21), maxLevel=3, criteria=(
    cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

# Initialize variables
prev_img = None
prev_features = None
trajectory = np.zeros((600, 800, 3), dtype=np.uint8)  # For visualization
pose = np.eye(4)  # Initial pose
positions = []

# Initialize rerun once at the start
rr.init("KITTI", spawn=True)
rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Y_DOWN, static=True)
# Log arrows representing XYZ axes
rr.log(
    "world/xyz",
    rr.Arrows3D(
        vectors=[[1, 0, 0], [0, 1, 0], [0, 0, 1]],
        colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
    ),
)


# Initialize SIFT detector
sift = cv2.SIFT_create()

# Create a BFMatcher
bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)


for i, file in enumerate(image_files):
    # Read current image
    curr_img = cv2.imread(file, cv2.IMREAD_GRAYSCALE)
    if curr_img is None:
        print(f"Could not read image: {file}")
        continue

    if prev_img is None:
        # Detect keypoints and compute descriptors in the initial image
        prev_keypoints, prev_descriptors = sift.detectAndCompute(
            curr_img, None)
        prev_img = curr_img
        continue

    # Detect keypoints and compute descriptors in the current image
    curr_keypoints, curr_descriptors = sift.detectAndCompute(curr_img, None)

    if prev_descriptors is None or curr_descriptors is None:
        print("No descriptors found in one of the images.")
        prev_img = curr_img
        prev_keypoints, prev_descriptors = curr_keypoints, curr_descriptors
        continue

    # Perform KNN matching
    knn_matches = bf.knnMatch(prev_descriptors, curr_descriptors, k=2)

    # Apply Lowe's ratio test
    ratio_thresh = 0.75
    good_matches = []
    for m, n in knn_matches:
        if m.distance < ratio_thresh * n.distance:
            good_matches.append(m)

    if len(good_matches) < 8:
        print("Insufficient good matches for pose estimation.")
        prev_img = curr_img
        prev_keypoints, prev_descriptors = curr_keypoints, curr_descriptors
        continue

    # Extract matched points
    prev_pts = np.float32(
        [prev_keypoints[m.queryIdx].pt for m in good_matches])
    curr_pts = np.float32(
        [curr_keypoints[m.trainIdx].pt for m in good_matches])

    # Estimate Essential Matrix and recover pose
    E, mask = cv2.findEssentialMat(
        curr_pts, prev_pts, K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
    _, R, t, mask = cv2.recoverPose(E, curr_pts, prev_pts, K)

    # Update the pose
    transformation = np.eye(4)
    transformation[:3, :3] = R
    transformation[:3, 3] = t.squeeze()
    pose = pose @ transformation

    R_global = pose[:3, :3]
    t_global = pose[:3, 3]

    height, width = curr_img.shape
    if i % 20 == 0:
        rr.log("world/camera" + str(i),
               rr.Pinhole(focal_length=float(fx), width=width, height=height))

        # Compute quaternion from rotation matrix, [x, y, z, w]
        quaternion = rotation_matrix_to_quaternion(R_global)

        quaternion_rerun = [quaternion[0],
                            quaternion[1], quaternion[2], quaternion[3]]
        quaternion_rerun = rr.Quaternion(xyzw=quaternion_rerun)

        # Log the camera's global transform to Rerun
        rr.log(
            "world/camera" + str(i),
            rr.Transform3D(
                translation=list(t_global.flatten()),
                rotation=quaternion_rerun
            ),
        )

        # Log the current image to Rerun
        rr.log("world/camera" + str(i), rr.Image(curr_img))
        rr.log("world/camera" + str(i) + "/image/rgb", rr.Image(curr_img))

    # Update variables for next iteration
    prev_img = curr_img
    prev_keypoints, prev_descriptors = curr_keypoints, curr_descriptors

cv2.destroyAllWindows()
