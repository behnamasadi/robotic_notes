import os
import cv2
import rerun as rr
import numpy as np
import cv2 as cv
from rerun.datatypes import Angle, RotationAxisAngle


def list_images(images_dir):
    """
    List all image files in the specified directory.
    :param images_dir: The directory containing image files (absolute or relative path).
    :return: A list of full paths to image files.
    """
    # Normalize the path to handle both relative and absolute paths
    images_dir_full_path = os.path.abspath(images_dir)

    # Supported image file extensions
    image_extensions = ('.jpg', '.jpeg', '.png', '.bmp',
                        '.gif', '.tiff', '.webp')

    # List to store image file paths
    image_files = []

    # Walk through the directory
    for root, dirs, files in os.walk(images_dir_full_path):
        for file in files:
            if file.lower().endswith(image_extensions):
                # Add the full path of the image file
                image_files.append(os.path.join(root, file))

    return image_files


# Initialize Rerun session
rr.init("opencv_images", spawn=True)


# Set view coordinates
rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

# Log arrows representing XYZ axes
rr.log("world/xyz",
       rr.Arrows3D(
           vectors=[[5, 0, 0], [0, 5, 0], [0, 0, 5]],
           colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
       ),
       )


# Use relative path based on the current script directory
script_dir = os.path.dirname(os.path.abspath(__file__))
dataset_relative_path = "dataset/laptop_webcam/"  # Relative to script directory
dataset_path = os.path.join(script_dir, dataset_relative_path)


images_dir = "images"
images_dir_full_path = dataset_path+images_dir
print(images_dir_full_path)


image_files = list_images(images_dir_full_path)
img = cv.imread(image_files[0])


print("img.shape:", img.shape)
height, width, _ = img.shape  # Corrected the order of dimensions


# Path to your XML file
calibration_file = "laptop_calib_result_1280x720.xml"

calibration_file_path = dataset_path+calibration_file
print(calibration_file_path)

# Open the XML file
fs = cv2.FileStorage(calibration_file_path, cv2.FILE_STORAGE_READ)


image_width = int(fs.getNode("image_width").real())
image_height = int(fs.getNode("image_height").real())

# Read the camera matrix
camera_matrix = fs.getNode("camera_matrix").mat()
print("Camera Matrix:")
print(camera_matrix)

fx = camera_matrix[0, 0]
fy = camera_matrix[1, 1]

cx = camera_matrix[0, 2]
cy = camera_matrix[1, 2]
print(fx)
print(fy)
print(cx)
print(cy)


# Read the distortion coefficients
dist_coeffs = fs.getNode("distortion_coefficients").mat()
print("\nDistortion Coefficients:")
print(dist_coeffs)

k1, k2, p1, p2, k3 = dist_coeffs.flatten()

print("k1, k2, p1, p2, k3", k1, k2, p1, p2, k3)


# Read extrinsic parameters
extrinsic_parameters = fs.getNode("extrinsic_parameters").mat()

# Print the parameters with interpretation
num_views = extrinsic_parameters.shape[0]
print(f"Number of views: {num_views}")

for i in range(num_views):
    # Extract the 6-tuple (rvec and tvec)
    # First 3 values are the rotation vector
    rvec = extrinsic_parameters[i, :3]
    # Last 3 values are the translation vector
    tvec = extrinsic_parameters[i, 3:]

    # Convert rotation vector to a rotation matrix
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    # print(f"\nView {i+1}:")
    # print(f"Rotation Vector: {rvec}")
    # print(f"Rotation Matrix:\n{rotation_matrix}")
    # print(f"Translation Vector: {tvec}")

    # Convert rotation matrix to a quaternion
    # r_quaternion = cv2.RQDecomp3x3(rotation_matrix)[0]
    # quaternion = [
    #     rotation_matrix[0, 0],
    #     rotation_matrix[1, 0],
    #     rotation_matrix[2, 0],
    # ]

    # Log the camera with pinhole model
    rr.log("world/camera"+str(i),
           rr.Pinhole(focal_length=float(fx), width=width, height=height))

    # Convert rotation vector to angle-axis representation
    # 1. Compute the angle (magnitude of the rotation vector)
    angle = np.linalg.norm(rvec)

    # 2. Compute the axis (normalized rotation vector)
    if angle != 0:
        axis = rvec / angle
    else:
        axis = np.zeros_like(rvec)  # Handle the zero rotation case

    print("Angle (in radians):", angle)
    print("Rotation Axis:", axis.flatten())

    # Optional: Convert to degrees for a human-readable format
    angle_degrees = np.degrees(angle)
    print("Angle (in degrees):", angle_degrees)

    # Log the camera's transform
    rr.log(
        "world/camera"+str(i),
        rr.Transform3D(
            translation=list(tvec.flatten()),
            rotation=RotationAxisAngle(
                axis=axis.flatten(), angle=Angle(rad=angle)),
        ),
    )

    # Log the image
    print(image_files[i])
    # Load and preprocess the image
    img = cv.imread(image_files[i])
    if img is None:
        raise FileNotFoundError("Image not found at the specified path.")
    img = cv.cvtColor(img, cv.COLOR_BGR2RGB)
    rr.log("world/camera"+str(i) + "/image/rgb",  rr.Image(img))


fs.release()


# Print the list of image files
for image in image_files:
    print(image)
