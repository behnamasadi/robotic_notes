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
image_left_path = "../../data/kitti/odometry/05/image_0/*.png"
image_left_path_abs_path = os.path.join(script_dir, image_left_path)
image_files_left = sorted(glob.glob(image_left_path_abs_path))


image_right_path = "../../data/kitti/odometry/05/image_1/*.png"
image_right_path_abs_path = os.path.join(script_dir, image_left_path)
image_files_right = sorted(glob.glob(image_right_path_abs_path))


print(image_right_path_abs_path)

calibration_file_abs_path = os.path.join(script_dir, calibration_file)
decomposed_data = load_and_decompose_calibration(calibration_file_abs_path)

# Camera intrinsic parameters since we are using left camera gray, it is P0
left_camera_name = "P0"
right_camera_name = "P1"
camera_P0 = decomposed_data[left_camera_name]
camera_P1 = decomposed_data[right_camera_name]
K0 = camera_P0["Intrinsic Matrix"]
K1 = camera_P1["Intrinsic Matrix"]

print("Intrinsic Matrix P0:", K0)
print("Intrinsic Matrix P1:", K1)


print("P0 Rotation Matrix:")
print(camera_P0["Rotation Matrix"])
print("P0 Translation Vector:")
print(camera_P0["Translation Vector"])


print("P1 Rotation Matrix:")
print(camera_P1["Rotation Matrix"])
print("P1 Translation Vector:")
print(camera_P1["Translation Vector"])


R = camera_P1["Rotation Matrix"]
T = camera_P1["Translation Vector"]


for file_left in image_files_left:
    # Read current left and right images
    left_img = cv2.imread(file_left, cv2.IMREAD_GRAYSCALE)
    height, width = left_img.shape

    break


# This provides rectified projection matrices (P1, P2) and a disparity-to-depth map transformation matrix (Q).
# R1	Output 3x3 rectification transform (rotation matrix) for the first camera. This matrix brings points given in the unrectified first camera's coordinate system to points in the rectified first camera's coordinate system. In more technical terms, it performs a change of basis from the unrectified first camera's coordinate system to the rectified first camera's coordinate system.
# R2	Output 3x3 rectification transform (rotation matrix) for the second camera. This matrix brings points given in the unrectified second camera's coordinate system to points in the rectified second camera's coordinate system. In more technical terms, it performs a change of basis from the unrectified second camera's coordinate system to the rectified second camera's coordinate system.
# P1	Output 3x4 projection matrix in the new (rectified) coordinate systems for the first camera, i.e. it projects points given in the rectified first camera coordinate system into the rectified first camera's image.
# P2	Output 3x4 projection matrix in the new (rectified) coordinate systems for the second camera, i.e. it projects points given in the rectified first camera coordinate system into the rectified second camera's image.
# Q	Output 4×4 disparity-to-depth mapping matrix (see reprojectImageTo3D).


R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
    cameraMatrix1=K0,
    distCoeffs1=None,  # No distortion coefficients provided
    cameraMatrix2=K1,
    distCoeffs2=None,
    imageSize=(width, height),
    R=R,
    T=T
)


print("R1:", R1, "R2:", R2, "P1:", P1, "P2:", P2, "Q:", Q)

for i, (file_left, file_right) in enumerate(zip(image_files_left, image_files_right)):
    # Read current left and right images
    left_img = cv2.imread(file_left, cv2.IMREAD_GRAYSCALE)
    right_img = cv2.imread(file_right, cv2.IMREAD_GRAYSCALE)

    # StereoBM or StereoSGBM for disparity
    stereo = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=16*5,  # Must be divisible by 16
        blockSize=11,
        P1=8*3*11**2,
        P2=32*3*11**2,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=32
    )

    disparity = stereo.compute(left_img, right_img)
    # print(disparity)

    # Normalize the disparity map for display
    disparity_normalized = cv2.normalize(
        disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

    # Display using OpenCV
    cv2.imshow("Disparity", disparity_normalized)

    points_3D = cv2.reprojectImageTo3D(disparity, Q)

    cv2.imshow("Left Image", left_img)
    cv2.imshow("Right Image", right_img)
    cv2.waitKey(0)  # Wait for key press to proceed to the next pair

cv2.destroyAllWindows()
