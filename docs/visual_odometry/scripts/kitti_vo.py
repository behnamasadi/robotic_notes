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


# Process each image in the dataset
for i, file in enumerate(image_files):
    # Read current image
    curr_img = cv2.imread(file, cv2.IMREAD_GRAYSCALE)
    if curr_img is None:
        print(f"Could not read image: {file}")
        continue

    if prev_img is None:
        # Detect initial features
        prev_features = cv2.goodFeaturesToTrack(
            curr_img, mask=None, **feature_params)
        prev_img = curr_img
        continue

    # Track features using Lucas-Kanade Optical Flow
    curr_features, status, err = cv2.calcOpticalFlowPyrLK(
        prev_img, curr_img, prev_features, None, **lk_params)

    # Filter valid points
    status = status.reshape(-1)
    prev_features = prev_features[status == 1]
    curr_features = curr_features[status == 1]

    if len(curr_features) < 8:
        print("Insufficient points for pose estimation.")
        prev_img = curr_img
        prev_features = cv2.goodFeaturesToTrack(
            curr_img, mask=None, **feature_params)
        continue

    # Estimate Essential Matrix and recover pose
    E, mask = cv2.findEssentialMat(
        curr_features, prev_features, K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
    _, R, t, mask = cv2.recoverPose(E, curr_features, prev_features, K)

    # Update the pose
    transformation = np.eye(4)
    transformation[:3, :3] = R
    transformation[:3, 3] = t.squeeze()
    pose = pose @ transformation

    R_global = pose[:3, :3]
    t_global = pose[:3, 3]

    height, width = curr_img.shape
    if i % 20 == 0:
        rr.log("world/camera"+str(i),
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
                # Convert translation to list
                translation=list(t_global.flatten()),
                rotation=quaternion_rerun
            ),
        )

        # Log the current image to Rerun
        rr.log("world/camera" + str(i), rr.Image(curr_img))

        rr.log("world/camera"+str(i) + "/image/rgb",  rr.Image(curr_img))

    # Save position for visualization
    positions.append(pose[:3, 3])

    # Visualize trajectory
    x, y, z = pose[:3, 3]
    draw_x, draw_y = int(x) + 400, int(z) + 300  # Adjust for visualization
    cv2.circle(trajectory, (draw_x, draw_y), 1, (0, 255, 0), 1)
    cv2.putText(trajectory, f"Frame: {
        i}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.imshow("Trajectory", trajectory)

    # Update variables for next iteration
    prev_img = curr_img
    prev_features = curr_features.reshape(-1, 1, 2)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
