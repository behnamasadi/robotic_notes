import os
import cv2
import numpy as np
import re
import rerun as rr

from rerun.datatypes import Angle, RotationAxisAngle


class CameraPose:
    def __init__(self):
        self.R = np.eye(3)
        self.t = np.zeros((3, 1))


def extract_number(filename):
    match = re.search(r"(\d+)", filename)
    return int(match.group(0)) if match else 0


def load_image_paths(directory_path):
    valid_extensions = [".jpg", ".jpeg", ".png", ".bmp", ".tiff"]
    image_paths = [
        os.path.join(directory_path, f)
        for f in os.listdir(directory_path)
        if os.path.splitext(f)[1].lower() in valid_extensions
    ]
    image_paths.sort(key=lambda x: extract_number(
        os.path.splitext(os.path.basename(x))[0]))
    return image_paths


def load_camera_calibration(calibration_file):
    fs = cv2.FileStorage(calibration_file, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise IOError(f"Failed to open calibration file: {calibration_file}")

    calibration_data = {
        "image_width": int(fs.getNode("image_width").real()),
        "image_height": int(fs.getNode("image_height").real()),
        "board_width": int(fs.getNode("board_width").real()),
        "board_height": int(fs.getNode("board_height").real()),
        "square_size": fs.getNode("square_size").real(),
        "fix_aspect_ratio": fs.getNode("fix_aspect_ratio").real(),
        "fisheye_model": bool(fs.getNode("fisheye_model").real()),
        "camera_matrix": fs.getNode("camera_matrix").mat(),
        "distortion_coefficients": fs.getNode("distortion_coefficients").mat(),
        "extrinsic_parameters": fs.getNode("extrinsic_parameters").mat(),
    }
    fs.release()

    # Extract and print the calibration parameters
    camera_matrix = calibration_data["camera_matrix"]
    distortion_coefficients = calibration_data["distortion_coefficients"]

    # Extracting fx, fy, cx, cy from the camera matrix
    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]

    # Extracting k1, k2, p1, p2 from the distortion coefficients
    k1, k2, p1, p2 = distortion_coefficients[:4].flatten()

    print(f"fx: {fx}, fy: {fy}, cx: {cx}, cy: {cy}")
    print(f"k1: {k1}, k2: {k2}, p1: {p1}, p2: {p2}")
    return calibration_data


def detect_and_compute_features(images):
    detector = cv2.SIFT_create()
    keypoints = []
    descriptors = []
    for img in images:
        kp, des = detector.detectAndCompute(img, None)
        keypoints.append(kp)
        descriptors.append(des)
    return keypoints, descriptors


def match_descriptors(descriptors, k=10):
    matcher = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)
    all_matches = []
    for i in range(len(descriptors) - 1):
        matches = matcher.match(descriptors[i], descriptors[i + 1])
        distances = [m.distance for m in matches]
        # For k=3, divide into 1/3 smaller, 2/3 larger
        cutoff_index = len(distances) // k
        cutoff_distance = sorted(distances)[cutoff_index]
        filtered_matches = [
            m for m in matches if m.distance <= cutoff_distance]
        all_matches.append(filtered_matches)
    return all_matches


def process_images(directory_path, calibration_file):
    calibration_data = load_camera_calibration(calibration_file)
    K = calibration_data["camera_matrix"]

    image_paths = load_image_paths(directory_path)
    images = [cv2.imread(path, cv2.IMREAD_GRAYSCALE) for path in image_paths]

    for img, path in zip(images, image_paths):
        if img is None:
            raise IOError(f"Failed to load image: {path}")

    keypoints, descriptors = detect_and_compute_features(images)
    matches = match_descriptors(descriptors)

    camera_poses = [CameraPose() for _ in images]
    points_3d = []

    for i in range(len(matches)):
        points1 = np.array([keypoints[i][m.queryIdx].pt for m in matches[i]])
        points2 = np.array(
            [keypoints[i + 1][m.trainIdx].pt for m in matches[i]])

        normalized_points1 = cv2.undistortPoints(points1, K, None)
        normalized_points2 = cv2.undistortPoints(points2, K, None)

        # E, mask = cv2.findEssentialMat(
        #     normalized_points1, normalized_points2, K, method=cv2.RANSAC)

        E, mask = cv2.findEssentialMat(
            points1, points2, K, method=cv2.RANSAC)

        if E is None:
            continue

        _, R, t, _ = cv2.recoverPose(
            E, normalized_points1, normalized_points2, K)

        print("t:", t.flatten())
        # print(" R:",  R)

        # Check if the translation aligns with the previous one
        if i > 0:
            prev_t = camera_poses[i].t.flatten()
            if np.dot(prev_t, t.flatten()) < 0:  # Flip if opposite directions
                print("Flip opposite directions")
                t = -t

        camera_poses[i + 1].R = camera_poses[i].R @ R
        camera_poses[i + 1].t = camera_poses[i].R @ t + camera_poses[i].t

        proj1 = K @ np.hstack((camera_poses[i].R, camera_poses[i].t))
        proj2 = K @ np.hstack((camera_poses[i + 1].R, camera_poses[i + 1].t))

        points_4d_hom = cv2.triangulatePoints(
            proj1, proj2, points1.T, points2.T)
        points_4d = points_4d_hom[:3] / points_4d_hom[3]

        points_3d.extend(points_4d.T)

        print(f"Processed pair {i} -> {i +
              1}: Triangulated {len(points_4d.T)} points")

        # Optionally display matches
        match_img = cv2.drawMatches(
            images[i], keypoints[i], images[i + 1], keypoints[i + 1], matches[i], None)
        cv2.imshow(f"Matches {i}-{i + 1}", match_img)
        # cv2.waitKey(0)
        cv2.destroyAllWindows()

    num_views = len(images)
    rr.init("SfM Example", spawn=True)

    for i in range(num_views):
        width = calibration_data["image_width"]
        height = calibration_data["image_height"]
        fx = K[0, 0]

        rr.log(f"world/camera{i}",
               rr.Pinhole(focal_length=float(fx), width=width, height=height))

        rvec = cv2.Rodrigues(camera_poses[i].R)[0].flatten()
        tvec = camera_poses[i].t

        angle = np.linalg.norm(rvec)
        axis = rvec / angle if angle != 0 else np.zeros_like(rvec)

        print(f"translation: {tvec.flatten()}")
        # print(f"Angle (in radians): {angle}")
        # print(f"Rotation Axis: {axis.flatten()}")
        angle_degrees = np.degrees(angle)
        # print(f"Angle (in degrees): {angle_degrees}")

        rr.log(
            f"world/camera{i}",
            rr.Transform3D(
                translation=list(tvec.flatten()),
                rotation=RotationAxisAngle(
                    axis=axis.flatten(), angle=Angle(rad=angle)),
            ),
        )

        img = cv2.imread(image_paths[i])
        if img is None:
            raise FileNotFoundError("Image not found at the specified path.")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        rr.log(f"world/camera{i}/image/rgb", rr.Image(img))

    return points_3d


if __name__ == "__main__":

    images_directory = "images"

    # Use relative path based on the current script directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # Relative to script directory
    calibration_file = "data/laptop_calib_result_1280x720.xml"
    calibration_file_abs_path = os.path.join(script_dir, calibration_file)
    images_directory_abs_path = os.path.join(script_dir, images_directory)

    print(images_directory_abs_path)
    print(calibration_file_abs_path)

    points_3d = process_images(
        images_directory_abs_path, calibration_file_abs_path)
    print(f"Reconstructed 3D points: {len(points_3d)}")
