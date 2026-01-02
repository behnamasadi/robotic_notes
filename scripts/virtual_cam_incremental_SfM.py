"""
Incremental Structure from Motion (SfM) with Bundle Adjustment

This is a Python conversion of the C++ incremental SfM implementation.
The original C++ code uses Ceres Solver; this Python version uses pyceres
(Python bindings for Ceres Solver) to match the C++ implementation.

Key features:
- Track-based incremental SfM pipeline
- Bundle adjustment to optimize camera poses and 3D points
- Rerun visualization of ground truth and optimized reconstructions
- Handles partial visibility (points visible in subset of cameras)
"""

import numpy as np
import cv2
import rerun as rr
import pyceres
from typing import List, Tuple, Dict, Optional
import math

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================


def eulerAnglesToRotationMatrix(theta):
    """Convert Euler angles (roll, pitch, yaw) to rotation matrix.
    Uses ZYX order (R_z * R_y * R_x).
    """
    roll, pitch, yaw = theta[0], theta[1], theta[2]

    # Rotation about x axis
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    # Rotation about y axis
    R_y = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    # Rotation about z axis
    R_z = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    # Combined rotation matrix
    R = R_z @ R_y @ R_x
    return R


def convertHomogeneous(points4D):
    """Convert from homogeneous (4 x N) to 3D points (divide by w)"""
    points3D = []
    for i in range(points4D.shape[1]):
        w = points4D[3, i]
        point = np.array([
            points4D[0, i] / w,
            points4D[1, i] / w,
            points4D[2, i] / w
        ])
        points3D.append(point)
    return points3D


def createImage(focalLength, numberOfPixelInHeight, numberOfPixelInWidth,
                projectedPoints, fileName):
    """Create a synthetic image with projected points"""
    cameraImage = np.zeros(
        (numberOfPixelInHeight, numberOfPixelInWidth), dtype=np.uint8)

    # Draw crosshair
    cv2.line(cameraImage,
             (numberOfPixelInWidth // 2, 0),
             (numberOfPixelInWidth // 2, numberOfPixelInHeight),
             255)
    cv2.line(cameraImage,
             (0, numberOfPixelInHeight // 2),
             (numberOfPixelInWidth, numberOfPixelInHeight // 2),
             255)

    # Draw projected points
    for pt in projectedPoints:
        col = int(pt[0])
        row = int(pt[1])
        if 0 <= row < numberOfPixelInHeight and 0 <= col < numberOfPixelInWidth:
            cameraImage[row, col] = 255
        else:
            print(f"{row},{col} is out of image")

    cv2.imwrite(fileName, cameraImage)
    return cameraImage


def createEllipsoidInWorldCoordinate(centerX=0, centerY=0, centerZ=0):
    """Create points on an ellipsoid surface"""
    objectPointsInWorldCoordinate = []

    phiStepSize = 0.2
    thetaStepSize = 0.1
    a, b, c = 2, 3, 1.6

    phi = -np.pi
    while phi < np.pi:
        theta = -np.pi / 2
        while theta < np.pi / 2:
            X = a * np.cos(theta) * np.cos(phi) - centerX
            Y = b * np.cos(theta) * np.sin(phi) - centerY
            Z = c * np.sin(theta) - centerZ
            objectPointsInWorldCoordinate.append(np.array([X, Y, Z]))
            theta += thetaStepSize
        phi += phiStepSize

    return objectPointsInWorldCoordinate


def RtToAngleAxisAndTranslate(R, t, camera_params):
    """Convert (R,t) from OpenCV to angle-axis + translation format"""
    rodrigues, _ = cv2.Rodrigues(R)
    camera_params[0:3] = rodrigues.flatten()[:3]
    camera_params[3:6] = t.flatten()[:3]


def angleAxisToRotationMatrix(angle_axis):
    """Convert angle-axis to rotation matrix"""
    R, _ = cv2.Rodrigues(angle_axis)
    return R


# ============================================================================
# DATA STRUCTURES
# ============================================================================

class FeatureObservation:
    def __init__(self, camera_idx, feature_idx, pixel):
        self.camera_idx = camera_idx
        self.feature_idx = feature_idx
        self.pixel = pixel  # (x, y) as np.array or tuple


class Track:
    """A track represents a single physical 3D point observed across multiple cameras"""

    def __init__(self):
        # Index in globalPoints3D (-1 if not triangulated)
        self.point3D_idx = -1
        self.observations = []  # List of FeatureObservation

    def addObservation(self, camera_idx, feature_idx, pixel):
        self.observations.append(FeatureObservation(
            camera_idx, feature_idx, pixel))

    def isObservedBy(self, camera_idx):
        return any(obs.camera_idx == camera_idx for obs in self.observations)

    def getFeatureIdx(self, camera_idx):
        for obs in self.observations:
            if obs.camera_idx == camera_idx:
                return obs.feature_idx
        return -1

    def numObservations(self):
        return len(self.observations)


class CameraExtrinsics:
    def __init__(self):
        self.R = np.eye(3)  # 3x3 rotation matrix
        self.t = np.zeros((3, 1))  # 3x1 translation vector


# ============================================================================
# REPROJECTION ERROR FUNCTION (PyCeres Cost Function)
# ============================================================================

class SnavelyReprojectionErrorFixedCamera(pyceres.CostFunction):
    """
    Snavely reprojection error with fixed camera intrinsics for pyceres.

    This matches the C++ SnavelyReprojectionErrorFixedCamera struct.
    Camera parameters: [angle_axis (3), translation (3)] = 6 params
    Point parameters: [x, y, z] = 3 params
    Residuals: [res_x, res_y] = 2 residuals
    """

    def __init__(self, observed_x, observed_y, fixed_focal, fixed_l1, fixed_l2):
        super().__init__()
        self.m_observed_x = observed_x
        self.m_observed_y = observed_y
        self.m_focal = fixed_focal
        self.m_l1 = fixed_l1
        self.m_l2 = fixed_l2

        # 2 residuals, 6 camera parameters, 3 point parameters
        self.set_num_residuals(2)
        self.set_parameter_block_sizes([6, 3])

    def Evaluate(self, parameters, residuals, jacobians):
        """
        Evaluate the cost function.

        parameters[0]: camera parameters [angle_axis (3), translation (3)]
        parameters[1]: 3D point [x, y, z]
        residuals: output [res_x, res_y]
        jacobians: optional output jacobians
        """
        camera = parameters[0]
        point = parameters[1]

        # Extract angle-axis and translation
        angle_axis = np.array(
            [camera[0], camera[1], camera[2]], dtype=np.float64)
        translation = np.array(
            [camera[3], camera[4], camera[5]], dtype=np.float64)
        point_array = np.array(
            [point[0], point[1], point[2]], dtype=np.float64)

        # Convert angle-axis to rotation matrix
        R, _ = cv2.Rodrigues(angle_axis)

        # Rotate point: p = R @ point
        p = R @ point_array

        # Translate: p = p + translation
        p = p + translation

        # Compute normalized coordinates (Snavely convention: negative z)
        xp = -p[0] / p[2]
        yp = -p[1] / p[2]

        # Apply radial distortion
        r2 = xp * xp + yp * yp
        distortion = 1.0 + r2 * (self.m_l1 + self.m_l2 * r2)

        # Compute predicted projection
        predicted_x = self.m_focal * distortion * xp
        predicted_y = self.m_focal * distortion * yp

        # Residuals
        residuals[0] = predicted_x - self.m_observed_x
        residuals[1] = predicted_y - self.m_observed_y

        # Compute jacobians if requested (using finite differences)
        # Note: For better performance, consider implementing analytical jacobians
        if jacobians is not None:
            eps = 1e-8
            # 2x6 matrix in row-major: 12 elements
            jacobians_camera = jacobians[0]
            # 2x3 matrix in row-major: 6 elements
            jacobians_point = jacobians[1]

            # Store current predicted values
            pred_x_base = predicted_x
            pred_y_base = predicted_y

            # Jacobian w.r.t. camera parameters (2x6)
            for i in range(6):
                camera_pert = camera.copy()
                camera_pert[i] += eps

                angle_axis_pert = np.array(
                    [camera_pert[0], camera_pert[1], camera_pert[2]], dtype=np.float64)
                translation_pert = np.array(
                    [camera_pert[3], camera_pert[4], camera_pert[5]], dtype=np.float64)
                R_pert, _ = cv2.Rodrigues(angle_axis_pert)
                p_pert = R_pert @ point_array + translation_pert

                xp_pert = -p_pert[0] / p_pert[2]
                yp_pert = -p_pert[1] / p_pert[2]
                r2_pert = xp_pert * xp_pert + yp_pert * yp_pert
                distortion_pert = 1.0 + r2_pert * \
                    (self.m_l1 + self.m_l2 * r2_pert)
                pred_x_pert = self.m_focal * distortion_pert * xp_pert
                pred_y_pert = self.m_focal * distortion_pert * yp_pert

                # Store in row-major format: jacobians[residual_idx * num_params + param_idx]
                jacobians_camera[0 * 6 + i] = (pred_x_pert - pred_x_base) / eps
                jacobians_camera[1 * 6 + i] = (pred_y_pert - pred_y_base) / eps

            # Jacobian w.r.t. point parameters (2x3)
            for i in range(3):
                point_pert = point.copy()
                point_pert[i] += eps
                point_pert_array = np.array(
                    [point_pert[0], point_pert[1], point_pert[2]], dtype=np.float64)

                p_pert = R @ point_pert_array + translation
                xp_pert = -p_pert[0] / p_pert[2]
                yp_pert = -p_pert[1] / p_pert[2]
                r2_pert = xp_pert * xp_pert + yp_pert * yp_pert
                distortion_pert = 1.0 + r2_pert * \
                    (self.m_l1 + self.m_l2 * r2_pert)
                pred_x_pert = self.m_focal * distortion_pert * xp_pert
                pred_y_pert = self.m_focal * distortion_pert * yp_pert

                # Store in row-major format
                jacobians_point[0 * 3 + i] = (pred_x_pert - pred_x_base) / eps
                jacobians_point[1 * 3 + i] = (pred_y_pert - pred_y_base) / eps

        return True


# ============================================================================
# INCREMENTAL STRUCTURE FROM MOTION
# ============================================================================

def virtualCamIncrementalSfMFixedCam():
    """Main incremental SfM function"""

    # Camera extrinsics setup
    thetaCam0 = np.array([0, np.pi / 12, 0])
    thetaCam1 = np.array([0, np.pi / 18, 0])
    thetaCam2 = np.array([0, -np.pi / 24, 0])

    # Scaled by 1.5x to demonstrate scale ambiguity
    txCam0, tyCam0, tzCam0 = 0.0, 0.0, 0.0
    txCam1, tyCam1, tzCam1 = 1.35, 0.15, 0.45
    txCam2, tyCam2, tzCam2 = 2.4, -0.15, 0.6

    t_Cam0_in_world = np.array([[txCam0], [tyCam0], [tzCam0]])
    t_Cam1_in_world = np.array([[txCam1], [tyCam1], [tzCam1]])
    t_Cam2_in_world = np.array([[txCam2], [tyCam2], [tzCam2]])

    rotation_Cam0_in_world = eulerAnglesToRotationMatrix(thetaCam0)
    rotation_Cam1_in_world = eulerAnglesToRotationMatrix(thetaCam1)
    rotation_Cam2_in_world = eulerAnglesToRotationMatrix(thetaCam2)

    # Create ellipsoid in world coordinates
    ellipsoidCenterX = -2.25
    ellipsoidCenterY = 0
    ellipsoidCenterZ = -6
    objectPointsInWorldCoordinate = createEllipsoidInWorldCoordinate(
        ellipsoidCenterX, ellipsoidCenterY, ellipsoidCenterZ
    )

    # Camera intrinsics
    k1, k2 = 0.0, 0.0
    numberOfPixelInHeight = 600
    numberOfPixelInWidth = 600
    focalLength = 4.0
    heightOfSensor = 10
    widthOfSensor = 10

    my = numberOfPixelInHeight / heightOfSensor
    mx = numberOfPixelInWidth / widthOfSensor

    fx = focalLength * mx
    fy = focalLength * my
    cx = numberOfPixelInWidth / 2
    cy = numberOfPixelInHeight / 2

    K = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1]
    ])

    # Project points to each camera
    rotation_world_in_Cam0 = rotation_Cam0_in_world.T
    t_world_in_Cam0 = -rotation_Cam0_in_world.T @ t_Cam0_in_world

    rotation_world_in_Cam1 = rotation_Cam1_in_world.T
    t_world_in_Cam1 = -rotation_Cam1_in_world.T @ t_Cam1_in_world

    rotation_world_in_Cam2 = rotation_Cam2_in_world.T
    t_world_in_Cam2 = -rotation_Cam2_in_world.T @ t_Cam2_in_world

    objectPointsArray = np.array(
        objectPointsInWorldCoordinate, dtype=np.float32)

    imagePointsCam0, _ = cv2.projectPoints(
        objectPointsArray, rotation_world_in_Cam0, t_world_in_Cam0, K, None
    )
    imagePointsCam0 = imagePointsCam0.reshape(-1, 2)

    imagePointsCam1, _ = cv2.projectPoints(
        objectPointsArray, rotation_world_in_Cam1, t_world_in_Cam1, K, None
    )
    imagePointsCam1 = imagePointsCam1.reshape(-1, 2)

    imagePointsCam2, _ = cv2.projectPoints(
        objectPointsArray, rotation_world_in_Cam2, t_world_in_Cam2, K, None
    )
    imagePointsCam2 = imagePointsCam2.reshape(-1, 2)

    # Initialize Rerun
    rr.init("virtual_cam_incremental_SfM", spawn=True)
    rec = rr.get_global_data_recording()
    rec.log("world", rr.ViewCoordinates.RIGHT_HAND_Y_DOWN)

    # Log coordinate axes
    rec.log("world/xyz", rr.Arrows3D(
        vectors=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]]
    ))

    # Transform ground truth points to Camera 0 frame
    point3d_positions = []
    point_sizes = []

    for pt_world in objectPointsInWorldCoordinate:
        pt_world_mat = np.array(pt_world).reshape(3, 1)
        pt_cam0 = rotation_Cam0_in_world.T @ (pt_world_mat - t_Cam0_in_world)
        point3d_positions.append(pt_cam0.flatten())
        point_sizes.append(0.05)

    gt_colors = [(128, 128, 128)] * len(point3d_positions)
    rec.log("world/points_ground_truth", rr.Points3D(
        positions=point3d_positions,
        radii=point_sizes,
        colors=gt_colors
    ))

    # Transform ground truth cameras to Camera 0 reference frame
    gt_R_relative = [None] * 3
    gt_t_relative = [None] * 3

    gt_R_relative[0] = np.eye(3)
    gt_t_relative[0] = np.zeros((3, 1))

    gt_R_relative[1] = rotation_Cam0_in_world.T @ rotation_Cam1_in_world
    gt_t_relative[1] = rotation_Cam0_in_world.T @ (
        t_Cam1_in_world - t_Cam0_in_world)

    gt_R_relative[2] = rotation_Cam0_in_world.T @ rotation_Cam2_in_world
    gt_t_relative[2] = rotation_Cam0_in_world.T @ (
        t_Cam2_in_world - t_Cam0_in_world)

    # Log ground truth cameras
    for cam_id in range(3):
        cam_name = f"world/ground_truth/cam{cam_id}"

        # Convert to camera-in-world for visualization
        R_cam_in_world = gt_R_relative[cam_id].T
        t_cam_in_world = -gt_R_relative[cam_id].T @ gt_t_relative[cam_id]

        # Log transform (mat3x3 as flattened list)
        rec.log(cam_name, rr.Transform3D(
            translation=t_cam_in_world.flatten().tolist(),
            mat3x3=R_cam_in_world.flatten().tolist()
        ))

        # Log pinhole
        rec.log(cam_name + "/image", rr.Pinhole(
            resolution=[numberOfPixelInWidth, numberOfPixelInHeight],
            focal_length=[fx, fy],
            principal_point=[cx, cy]
        ))

    # Create and log images
    fileName = f"image_cam0{focalLength}_.png"
    img_cam0 = createImage(focalLength, numberOfPixelInHeight, numberOfPixelInWidth,
                           imagePointsCam0, fileName)
    rec.log("world/ground_truth/cam0/image", rr.Image(img_cam0))

    fileName = f"image_cam1{focalLength}_.png"
    img_cam1 = createImage(focalLength, numberOfPixelInHeight, numberOfPixelInWidth,
                           imagePointsCam1, fileName)
    rec.log("world/ground_truth/cam1/image", rr.Image(img_cam1))

    fileName = f"image_cam2{focalLength}_.png"
    img_cam2 = createImage(focalLength, numberOfPixelInHeight, numberOfPixelInWidth,
                           imagePointsCam2, fileName)
    rec.log("world/ground_truth/cam2/image", rr.Image(img_cam2))

    print("Press any key to continue...")
    input()

    # Setup for incremental SfM
    N_CAMERAS = 3

    # Create keypoints (using projected points)
    keypoints = []
    for points in [imagePointsCam0, imagePointsCam1, imagePointsCam2]:
        kp_list = []
        for pt in points:
            kp_list.append(cv2.KeyPoint(pt[0], pt[1], 1.0))
        keypoints.append(kp_list)

    images = [img_cam0, img_cam1, img_cam2]

    # Create matches (assuming same point indices match)
    all_matches = []
    for i in range(N_CAMERAS - 1):
        matches = []
        num_points = len(keypoints[i])
        for j in range(num_points):
            matches.append(cv2.DMatch(j, j, 0.0))
        all_matches.append(matches)

    # Initialize cameras (Camera 0 is world reference)
    cameras = [CameraExtrinsics() for _ in range(N_CAMERAS)]
    cameras[0].R = np.eye(3)
    cameras[0].t = np.zeros((3, 1))

    cameraParams = np.zeros(6 * N_CAMERAS)
    RtToAngleAxisAndTranslate(cameras[0].R, cameras[0].t, cameraParams[0:6])

    # Global 3D points and observations
    globalPoints3D = []

    # Track-based structure
    tracks = []
    featureToTrack = {}  # (camera_idx, feature_idx) -> track_id

    # Incremental SfM loop
    for i in range(1, N_CAMERAS):
        print(f"i is : {i}, processing camera: {i-1} and camera: {i}")

        # Extract matched points
        pts_im1 = np.array(
            [keypoints[i-1][match.queryIdx].pt for match in all_matches[i-1]])
        pts_i = np.array(
            [keypoints[i][match.trainIdx].pt for match in all_matches[i-1]])

        # Find essential matrix
        E, mask = cv2.findEssentialMat(pts_im1, pts_i, K, method=cv2.RANSAC,
                                       prob=0.999, threshold=1.0)

        # Recover pose
        _, R_im1_to_i, t_im1_to_i, mask = cv2.recoverPose(
            E, pts_im1, pts_i, K, mask=mask)
        t_im1_to_i = t_im1_to_i.flatten()

        # Chain transformations
        R_0_to_im1 = cameras[i-1].R
        t_0_to_im1 = cameras[i-1].t

        R_0_to_i = R_im1_to_i @ R_0_to_im1
        t_0_to_i = R_im1_to_i @ t_0_to_im1 + t_im1_to_i.reshape(3, 1)

        cameras[i].R = R_0_to_i
        cameras[i].t = t_0_to_i
        RtToAngleAxisAndTranslate(
            cameras[i].R, cameras[i].t, cameraParams[i*6:(i+1)*6])

        # Build projection matrices
        Rt_im1 = np.hstack([cameras[i-1].R, cameras[i-1].t])
        P_im1 = K @ Rt_im1

        Rt_i = np.hstack([cameras[i].R, cameras[i].t])
        P_i = K @ Rt_i

        # Triangulate
        points4D = cv2.triangulatePoints(P_im1, P_i, pts_im1.T, pts_i.T)
        newPoints_in_cam0 = convertHomogeneous(points4D)

        # Track management
        rr_triangulated_pointsInCam0 = []

        for k, match in enumerate(all_matches[i-1]):
            feature_im1 = match.queryIdx
            feature_i = match.trainIdx

            key_im1 = (i-1, feature_im1)

            track_id = -1
            is_new_track = False

            if key_im1 in featureToTrack:
                # Extend existing track
                track_id = featureToTrack[key_im1]
            else:
                # Create new track
                track_id = len(tracks)
                tracks.append(Track())
                is_new_track = True
                tracks[track_id].addObservation(i-1, feature_im1, pts_im1[k])
                featureToTrack[key_im1] = track_id

            # Add observation from camera i
            key_i = (i, feature_i)
            tracks[track_id].addObservation(i, feature_i, pts_i[k])
            featureToTrack[key_i] = track_id

            # Triangulate if new track
            if is_new_track:
                point3D_idx = len(globalPoints3D)
                globalPoints3D.append(newPoints_in_cam0[k])
                tracks[track_id].point3D_idx = point3D_idx
                rr_triangulated_pointsInCam0.append(newPoints_in_cam0[k])

        # Log triangulated points
        if len(rr_triangulated_pointsInCam0) > 0:
            colors = [(255, 0, 0) if (i % 2 == 0) else (0, 255, 0)
                      ] * len(rr_triangulated_pointsInCam0)
            rec.log(f"world/initial_triangulation_iteration_{i}", rr.Points3D(
                positions=rr_triangulated_pointsInCam0,
                radii=[0.05] * len(rr_triangulated_pointsInCam0),
                colors=colors
            ))

        print(
            f"Total tracks: {len(tracks)}, Total 3D points: {len(globalPoints3D)}")
        print("Press any key to continue...")
        input()

    # Build observations for bundle adjustment
    class Observation:
        def __init__(self, camera_idx, point_idx, x, y):
            self.camera_idx = camera_idx
            self.point_idx = point_idx
            self.x = x
            self.y = y

    observations = []
    for track in tracks:
        if track.point3D_idx < 0:
            continue
        for feat_obs in track.observations:
            # Convert pixel to Snavely normalized coordinates
            x_sn = -(feat_obs.pixel[0] - cx) / fx
            y_sn = -(feat_obs.pixel[1] - cy) / fy
            obs = Observation(feat_obs.camera_idx,
                              track.point3D_idx, x_sn, y_sn)
            observations.append(obs)

    print(f"Total observations: {len(observations)}")

    # Prepare parameters for optimization (as numpy arrays)
    pointParams = np.array(globalPoints3D, dtype=np.float64)

    # Convert camera parameters to list of numpy arrays (one per camera)
    camera_params_list = []
    for i in range(N_CAMERAS):
        cam_params = np.array(cameraParams[i*6:(i+1)*6], dtype=np.float64)
        camera_params_list.append(cam_params)

    # Bundle adjustment using pyceres (matching C++ implementation)
    problem = pyceres.Problem()

    # Add parameter blocks for cameras
    for i in range(N_CAMERAS):
        problem.add_parameter_block(camera_params_list[i], 6)

    # Add parameter blocks for 3D points
    point_params_list = []
    for i in range(len(globalPoints3D)):
        point_param = np.array(pointParams[i], dtype=np.float64)
        point_params_list.append(point_param)
        problem.add_parameter_block(point_param, 3)

    # Add residual blocks (matching C++ code)
    print("Building Ceres problem with {} residual blocks...".format(len(observations)))

    loss = pyceres.TrivialLoss()  # No robust loss function
    for obs in observations:
        cost_function = SnavelyReprojectionErrorFixedCamera(
            obs.x, obs.y, 1.0, k1, k2
        )

        camera_params_ptr = camera_params_list[obs.camera_idx]
        point_params_ptr = point_params_list[obs.point_idx]

        # Parameters must be passed as a list (matching pyceres API)
        problem.add_residual_block(
            cost_function, loss, [camera_params_ptr, point_params_ptr])

    # Fix camera 0 to eliminate gauge freedom (matching C++ code)
    problem.set_parameter_block_constant(camera_params_list[0])

    print("\nCamera 0 is FIXED (world reference frame):")
    print("   R = I (identity), t = (0,0,0)")
    print("   This eliminates gauge freedom in the optimization.")
    print("   Only Camera 1 and Camera 2 poses will be optimized.\n")

    # Configure solver options (matching C++ code)
    options = pyceres.SolverOptions()
    options.linear_solver_type = pyceres.LinearSolverType.DENSE_SCHUR
    options.minimizer_progress_to_stdout = True
    options.max_num_iterations = 100

    # Solve
    print("Running bundle adjustment...")
    summary = pyceres.SolverSummary()
    pyceres.solve(options, problem, summary)

    print("\nBundle Adjustment Summary:")
    print(summary.BriefReport())

    # Extract optimized parameters
    optimized_cameraParams = np.zeros(6 * N_CAMERAS)
    for i in range(N_CAMERAS):
        optimized_cameraParams[i*6:(i+1)*6] = camera_params_list[i]

    optimized_pointParams = np.array([p for p in point_params_list]).flatten()

    print("\nBundle adjustment completed!")

    # Visualize optimized points
    point3d_positions_after_BA = []
    for i in range(0, len(optimized_pointParams), 3):
        point3d_positions_after_BA.append(optimized_pointParams[i:i+3])

    rec.log("world/optimized_points_after_BA", rr.Points3D(
        positions=point3d_positions_after_BA,
        radii=[0.06] * len(point3d_positions_after_BA),
        colors=[(255, 255, 0)] * len(point3d_positions_after_BA)  # Yellow
    ))

    # Visualize optimized cameras
    for cam_id in range(N_CAMERAS):
        cam_params = optimized_cameraParams[cam_id*6:(cam_id+1)*6]
        angle_axis = cam_params[:3]
        translation = cam_params[3:6]

        R = angleAxisToRotationMatrix(angle_axis)
        t = translation.reshape(3, 1)

        # Convert to camera-in-world
        R_cam_in_world = R.T
        t_cam_in_world = -R.T @ t

        opt_camera_name = f"world/optimized/cam{cam_id}"
        rec.log(opt_camera_name, rr.Transform3D(
            translation=t_cam_in_world.flatten().tolist(),
            mat3x3=R_cam_in_world.flatten().tolist()
        ))

        rec.log(opt_camera_name + "/image", rr.Pinhole(
            resolution=[numberOfPixelInWidth, numberOfPixelInHeight],
            focal_length=[fx, fy],
            principal_point=[cx, cy]
        ))

        # Log images to the same path (enables camera rays)
        if cam_id == 0:
            rec.log(opt_camera_name + "/image", rr.Image(img_cam0))
        elif cam_id == 1:
            rec.log(opt_camera_name + "/image", rr.Image(img_cam1))
        elif cam_id == 2:
            rec.log(opt_camera_name + "/image", rr.Image(img_cam2))

    print("Press any key to exit...")
    input()


if __name__ == "__main__":
    virtualCamIncrementalSfMFixedCam()
