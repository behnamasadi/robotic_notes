#!/usr/bin/env python3
"""
EuRoC Dense Stereo Reconstruction

This script performs dense stereo reconstruction from EuRoC camera data:
1. Loads calibration from cam0 and cam1
2. Computes stereo rectification
3. Processes stereo image pairs
4. Computes dense disparity maps
5. Converts to depth and 3D point clouds
6. Visualizes results
"""

import numpy as np
import cv2
import yaml
import pandas as pd
import argparse
import rerun as rr
from pathlib import Path
from typing import Tuple, Dict, Optional


def load_euroc_camera_calibration(sensor_yaml_path: Path) -> Dict:
    """
    Load camera calibration from EuRoC sensor.yaml file.

    Returns dict with:
    - T_BS: 4x4 transformation matrix from body to camera
    - resolution: [width, height]
    - intrinsics: [fu, fv, cu, cv]
    - distortion_coefficients: [k1, k2, p1, p2]
    - camera_matrix: 3x3 K matrix
    - dist_coeffs: distortion coefficients array
    """
    with open(sensor_yaml_path, 'r') as f:
        sensor_data = yaml.safe_load(f)

    # Parse T_BS transformation matrix
    t_bs_data = sensor_data['T_BS']['data']
    T_BS = np.array(t_bs_data, dtype=np.float64).reshape(4, 4)

    # Parse intrinsics: [fu, fv, cu, cv]
    intrinsics = sensor_data['intrinsics']
    fu, fv, cu, cv = intrinsics

    # Build camera matrix K
    K = np.array([
        [fu, 0.0, cu],
        [0.0, fv, cv],
        [0.0, 0.0, 1.0]
    ], dtype=np.float64)

    # Parse distortion coefficients: [k1, k2, p1, p2]
    dist_coeffs = np.array(
        sensor_data['distortion_coefficients'], dtype=np.float64)

    resolution = sensor_data['resolution']  # [width, height]

    return {
        'T_BS': T_BS,
        'resolution': resolution,
        'intrinsics': intrinsics,
        'distortion_coefficients': dist_coeffs,
        'camera_matrix': K,
        'dist_coeffs': dist_coeffs,
        'rate_hz': sensor_data.get('rate_hz', 20)
    }


def compute_stereo_extrinsics(calib0: Dict, calib1: Dict) -> Tuple[np.ndarray, np.ndarray, float]:
    """
    Compute relative stereo transformation T_C1C0 from body-to-camera transforms.

    T_C1C0 = T_BC1^(-1) * T_BC0

    Returns:
    - R: 3x3 rotation matrix from cam0 to cam1
    - t: 3x1 translation vector from cam0 to cam1
    - baseline: baseline distance in meters
    """
    T_BC0 = calib0['T_BS']
    T_BC1 = calib1['T_BS']

    # Compute T_C1C0 = T_BC1^(-1) * T_BC0
    T_BC1_inv = np.linalg.inv(T_BC1)
    T_C1C0 = T_BC1_inv @ T_BC0

    # Extract rotation and translation
    R = T_C1C0[:3, :3]
    t = T_C1C0[:3, 3]

    # Compute baseline (magnitude of translation)
    baseline = np.linalg.norm(t)

    return R, t, baseline


def load_euroc_camera_data(cam_csv_path: Path, cam_data_dir: Path) -> pd.DataFrame:
    """
    Load camera image timestamps and paths from EuRoC CSV format.
    """
    df = pd.read_csv(cam_csv_path, comment='#')
    df.columns = ['timestamp', 'filename']

    # Convert timestamp from nanoseconds to seconds
    df['timestamp'] = df['timestamp'] * 1e-9

    # Build full image paths
    df['image_path'] = df['filename'].apply(lambda f: cam_data_dir / f)

    return df


def synchronize_stereo_pairs(cam0_data: pd.DataFrame, cam1_data: pd.DataFrame,
                             max_time_diff: float = 0.01) -> pd.DataFrame:
    """
    Synchronize stereo image pairs based on timestamps.
    """
    stereo_pairs = []

    for idx0, row0 in cam0_data.iterrows():
        t0 = row0['timestamp']

        # Find closest timestamp in cam1
        time_diffs = np.abs(cam1_data['timestamp'] - t0)
        idx1 = time_diffs.idxmin()

        if time_diffs.iloc[idx1] < max_time_diff:
            stereo_pairs.append({
                'timestamp': t0,
                'cam0_path': row0['image_path'],
                'cam1_path': cam1_data.iloc[idx1]['image_path'],
                'time_diff': time_diffs.iloc[idx1]
            })

    return pd.DataFrame(stereo_pairs)


def compute_stereo_rectification(calib0: Dict, calib1: Dict, R: np.ndarray, t: np.ndarray):
    """
    Compute stereo rectification using OpenCV.

    Returns rectification maps and new projection matrices.
    """
    K0 = calib0['camera_matrix']
    D0 = calib0['dist_coeffs']
    K1 = calib1['camera_matrix']
    D1 = calib1['dist_coeffs']

    image_size = tuple(calib0['resolution'])  # (width, height) -> (cols, rows)
    # OpenCV expects (height, width)
    image_size_swap = (image_size[1], image_size[0])

    # Compute rectification
    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        K0, D0, K1, D1, image_size_swap, R, t,
        flags=cv2.CALIB_ZERO_DISPARITY,
        alpha=0.9  # 0=no black borders, 1=all pixels valid
    )

    # Compute rectification maps
    map1x, map1y = cv2.initUndistortRectifyMap(
        K0, D0, R1, P1, image_size_swap, cv2.CV_32FC1)
    map2x, map2y = cv2.initUndistortRectifyMap(
        K1, D1, R2, P2, image_size_swap, cv2.CV_32FC1)

    return {
        'R1': R1, 'R2': R2, 'P1': P1, 'P2': P2, 'Q': Q,
        'roi1': roi1, 'roi2': roi2,
        'map1x': map1x, 'map1y': map1y,
        'map2x': map2x, 'map2y': map2y,
        'image_size': image_size_swap
    }


def create_stereo_matcher(method: str = 'sgbm', num_disparities: int = 96, block_size: int = 5):
    """
    Create stereo matcher (BM or SGBM).

    Args:
        method: 'bm' or 'sgbm'
        num_disparities: number of disparity levels (must be divisible by 16)
        block_size: block matching window size (must be odd)
    """
    if method.lower() == 'bm':
        matcher = cv2.StereoBM_create(
            numDisparities=num_disparities, blockSize=block_size)
    else:  # sgbm
        matcher = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=num_disparities,
            blockSize=block_size,
            P1=8 * 3 * block_size**2,  # Penalty for disparity change of 1
            P2=32 * 3 * block_size**2,  # Penalty for disparity change > 1
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32,
            preFilterCap=63,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )

    return matcher


def disparity_to_depth(disparity: np.ndarray, Q: np.ndarray) -> np.ndarray:
    """
    Convert disparity map to depth map using Q matrix.
    """
    # Reconstruct 3D points
    points_3d = cv2.reprojectImageTo3D(disparity, Q)

    # Extract depth (Z coordinate)
    depth = points_3d[:, :, 2]

    return depth, points_3d


def extract_colors_from_image(img_rect: np.ndarray, valid_mask: np.ndarray) -> np.ndarray:
    """
    Extract RGB colors from rectified image for valid pixels.

    Args:
        img_rect: Rectified BGR image
        valid_mask: Boolean mask indicating valid pixels

    Returns:
        colors: Nx3 array of RGB colors (one per valid pixel)
    """
    # Convert BGR to RGB
    img_rgb = cv2.cvtColor(img_rect, cv2.COLOR_BGR2RGB)

    # Extract colors for valid pixels
    colors = img_rgb[valid_mask]  # Shape: (N, 3)

    return colors


def create_colored_point_cloud(points_3d: np.ndarray, colors: np.ndarray,
                               valid_mask: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Create colored point cloud from 3D points and colors.

    Args:
        points_3d: 3D points array (H, W, 3)
        colors: RGB colors array (N, 3) for valid pixels
        valid_mask: Boolean mask indicating valid pixels

    Returns:
        points: Nx3 array of 3D points
        colors_out: Nx3 array of RGB colors (0-255)
    """
    # Extract valid 3D points
    points = points_3d[valid_mask]  # Shape: (N, 3)

    # Ensure colors match points
    if colors.shape[0] != points.shape[0]:
        # If colors not pre-extracted, extract now
        raise ValueError("Number of colors must match number of valid points")

    # Normalize colors to 0-1 range if needed (assuming 0-255 input)
    colors_out = colors.astype(np.uint8)

    return points, colors_out


def visualize_colored_point_cloud(points: np.ndarray, colors: np.ndarray,
                                  max_points: int = 50000):
    """
    Visualize colored point cloud using matplotlib 3D scatter.

    Args:
        points: Nx3 array of 3D points
        colors: Nx3 array of RGB colors (0-255)
        max_points: Maximum number of points to visualize (for performance)
    """
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    # Subsample if too many points
    if len(points) > max_points:
        indices = np.random.choice(len(points), max_points, replace=False)
        points = points[indices]
        colors = colors[indices]

    # Normalize colors to 0-1 range
    colors_normalized = colors.astype(np.float32) / 255.0

    # Create 3D plot
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Scatter plot with colors
    ax.scatter(points[:, 0], points[:, 1], points[:, 2],
               c=colors_normalized, s=0.1, alpha=0.6)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('Colored Point Cloud')

    # Set equal aspect ratio
    max_range = np.array([
        points[:, 0].max() - points[:, 0].min(),
        points[:, 1].max() - points[:, 1].min(),
        points[:, 2].max() - points[:, 2].min()
    ]).max() / 2.0
    mid_x = (points[:, 0].max() + points[:, 0].min()) * 0.5
    mid_y = (points[:, 1].max() + points[:, 1].min()) * 0.5
    mid_z = (points[:, 2].max() + points[:, 2].min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    plt.tight_layout()
    plt.show()


def visualize_stereo_results(img0: np.ndarray, img1: np.ndarray,
                             img0_rect: np.ndarray, img1_rect: np.ndarray,
                             disparity: np.ndarray, depth: np.ndarray):
    """
    Visualize stereo processing results.
    """
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 3, figsize=(18, 12))

    # Original images
    axes[0, 0].imshow(cv2.cvtColor(img0, cv2.COLOR_BGR2RGB))
    axes[0, 0].set_title('Original Left (cam0)')
    axes[0, 0].axis('off')

    axes[0, 1].imshow(cv2.cvtColor(img1, cv2.COLOR_BGR2RGB))
    axes[0, 1].set_title('Original Right (cam1)')
    axes[0, 1].axis('off')

    # Rectified images
    axes[0, 2].imshow(cv2.cvtColor(img0_rect, cv2.COLOR_BGR2RGB))
    axes[0, 2].set_title('Rectified Left')
    axes[0, 2].axis('off')

    axes[1, 0].imshow(cv2.cvtColor(img1_rect, cv2.COLOR_BGR2RGB))
    axes[1, 0].set_title('Rectified Right')
    axes[1, 0].axis('off')

    # Disparity map
    # SGBM returns fixed-point disparity
    disparity_viz = disparity.astype(np.float32) / 16.0
    valid_mask = disparity_viz > 0
    axes[1, 1].imshow(disparity_viz, cmap='jet')
    axes[1, 1].set_title('Disparity Map')
    axes[1, 1].axis('off')
    plt.colorbar(axes[1, 1].images[0], ax=axes[1, 1])

    # Depth map
    depth_viz = depth.copy()
    depth_viz[~valid_mask] = 0
    depth_viz[depth_viz > 20] = 0  # Clip far depths for visualization
    im = axes[1, 2].imshow(depth_viz, cmap='jet', vmin=0, vmax=20)
    axes[1, 2].set_title('Depth Map (meters)')
    axes[1, 2].axis('off')
    plt.colorbar(im, ax=axes[1, 2])

    plt.tight_layout()
    plt.show()


def log_cameras_to_rerun(calib0: Dict, calib1: Dict):
    """
    Log stereo camera setup to Rerun (static, called once).

    Args:
        calib0: Camera 0 calibration dict
        calib1: Camera 1 calibration dict
    """
    # Get camera intrinsics
    K0 = calib0['camera_matrix']
    K1 = calib1['camera_matrix']
    resolution0 = calib0['resolution']  # [width, height]
    resolution1 = calib1['resolution']

    # Extract focal lengths and principal points
    fx0, fy0 = K0[0, 0], K0[1, 1]
    fx1, fy1 = K1[0, 0], K1[1, 1]

    # Get camera extrinsics (T_BS: body to sensor)
    T_BC0 = calib0['T_BS']
    T_BC1 = calib1['T_BS']

    # Log camera 0 (left) with pinhole intrinsics
    # Use [fx, fy] for different focal lengths, or single value if fx == fy
    focal0 = [fx0, fy0] if abs(fx0 - fy0) > 1e-6 else fx0
    rr.log("world/body/cam0", rr.Pinhole(
        focal_length=focal0,
        width=resolution0[0],
        height=resolution0[1],
        principal_point=[K0[0, 2], K0[1, 2]],
        image_plane_distance=1.0  # Fixed distance for consistent frustum visualization
    ), static=True)

    # Log transform from body to cam0
    R_BC0 = T_BC0[:3, :3]
    t_BC0 = T_BC0[:3, 3]
    rr.log("world/body/cam0", rr.Transform3D(
        translation=t_BC0,
        mat3x3=R_BC0
    ), static=True)

    # Log camera 1 (right) with pinhole intrinsics
    focal1 = [fx1, fy1] if abs(fx1 - fy1) > 1e-6 else fx1
    rr.log("world/body/cam1", rr.Pinhole(
        focal_length=focal1,
        width=resolution1[0],
        height=resolution1[1],
        principal_point=[K1[0, 2], K1[1, 2]],
        image_plane_distance=1.0  # Fixed distance for consistent frustum visualization
    ), static=True)

    # Log transform from body to cam1
    R_BC1 = T_BC1[:3, :3]
    t_BC1 = T_BC1[:3, 3]
    rr.log("world/body/cam1", rr.Transform3D(
        translation=t_BC1,
        mat3x3=R_BC1
    ), static=True)


def process_stereo_frame(img0: np.ndarray, img1: np.ndarray, rectify_maps: Dict,
                         matcher, calib0: Dict) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Process a single stereo frame: rectify, compute disparity, convert to 3D.

    Returns:
        points_3d: 3D points in cam0 frame (H, W, 3)
        colors: RGB colors for valid pixels
        valid_mask: Boolean mask for valid pixels
        img0_rect: Rectified left image
        img1_rect: Rectified right image
    """
    # Rectify images
    img0_rect = cv2.remap(
        img0, rectify_maps['map1x'], rectify_maps['map1y'], cv2.INTER_LINEAR)
    img1_rect = cv2.remap(
        img1, rectify_maps['map2x'], rectify_maps['map2y'], cv2.INTER_LINEAR)

    # Convert to grayscale for stereo matching
    gray0 = cv2.cvtColor(img0_rect, cv2.COLOR_BGR2GRAY)
    gray1 = cv2.cvtColor(img1_rect, cv2.COLOR_BGR2GRAY)

    # Compute disparity
    disparity = matcher.compute(gray0, gray1)

    # Convert to depth
    depth, points_3d = disparity_to_depth(
        disparity.astype(np.float32) / 16.0, rectify_maps['Q'])

    # Create valid mask
    valid_mask = (disparity > 0) & (depth > 0) & (depth < 50)

    # Extract colors
    colors = extract_colors_from_image(img0_rect, valid_mask)

    return points_3d, colors, valid_mask, img0_rect, img1_rect


def log_frame_to_rerun(calib0: Dict, img0: np.ndarray, img1: np.ndarray,
                       img0_rect: np.ndarray, img1_rect: np.ndarray,
                       points_3d: np.ndarray, colors: np.ndarray, valid_mask: np.ndarray,
                       timestamp: float):
    """
    Log a single stereo frame (images and point cloud) to Rerun with timestamp.

    Args:
        calib0: Camera 0 calibration dict (for transform)
        img0: Original left image (BGR)
        img1: Original right image (BGR)
        img0_rect: Rectified left image (BGR)
        img1_rect: Rectified right image (BGR)
        points_3d: 3D points in cam0 frame (H, W, 3)
        colors: RGB colors for valid points (N, 3)
        valid_mask: Boolean mask for valid pixels
        timestamp: Timestamp in seconds
    """
    # Convert images to RGB
    img0_rgb = cv2.cvtColor(img0, cv2.COLOR_BGR2RGB)
    img1_rgb = cv2.cvtColor(img1, cv2.COLOR_BGR2RGB)
    img0_rect_rgb = cv2.cvtColor(img0_rect, cv2.COLOR_BGR2RGB)
    img1_rect_rgb = cv2.cvtColor(img1_rect, cv2.COLOR_BGR2RGB)

    # Set timestamp
    rr.set_time("timestamp", timestamp=timestamp)

    # Log camera images
    rr.log("world/body/cam0/image/original", rr.Image(img0_rgb))
    rr.log("world/body/cam0/image/rectified", rr.Image(img0_rect_rgb))
    rr.log("world/body/cam1/image/original", rr.Image(img1_rgb))
    rr.log("world/body/cam1/image/rectified", rr.Image(img1_rect_rgb))

    # Transform point cloud from cam0 frame to body frame
    # Points are currently in cam0 frame, transform to body: X_body = T_BC0^(-1) @ X_cam0
    T_BC0 = calib0['T_BS']
    T_BC0_inv = np.linalg.inv(T_BC0)

    # Extract valid points
    points_cam0 = points_3d[valid_mask]  # Shape: (N, 3)

    # Transform to homogeneous coordinates, transform, then back
    points_cam0_homo = np.hstack([points_cam0, np.ones((len(points_cam0), 1))])
    points_body_homo = (T_BC0_inv @ points_cam0_homo.T).T
    points_body = points_body_homo[:, :3]

    # Normalize colors to 0-1 range for Rerun (it expects 0-255 or 0-1)
    # Rerun expects colors in 0-255 range as uint8
    colors_normalized = colors.astype(np.uint8)

    # Log colored point cloud in body frame
    rr.log("world/body/point_cloud", rr.Points3D(
        positions=points_body,
        colors=colors_normalized,
        radii=0.01  # 1cm radius for points
    ))


def main():
    parser = argparse.ArgumentParser(
        description='EuRoC Dense Stereo Reconstruction')
    parser.add_argument(
        '--data-dir',
        type=str,
        default='../data/machine_hall/MH_01_easy/MH_01_easy/mav0',
        help='Path to EuRoC dataset directory'
    )
    parser.add_argument(
        '--method',
        type=str,
        default='sgbm',
        choices=['bm', 'sgbm'],
        help='Stereo matching method: bm (Block Matching) or sgbm (Semi-Global Block Matching)'
    )
    parser.add_argument(
        '--num-disparities',
        type=int,
        default=96,
        help='Number of disparity levels (must be divisible by 16)'
    )
    parser.add_argument(
        '--block-size',
        type=int,
        default=5,
        help='Block matching window size (must be odd)'
    )
    parser.add_argument(
        '--visualize',
        action='store_true',
        help='Visualize results'
    )
    args = parser.parse_args()

    # Get script directory and resolve data path
    script_dir = Path(__file__).resolve().parent
    data_dir = Path(args.data_dir)
    if not data_dir.is_absolute():
        data_dir = script_dir / data_dir

    # Load camera calibrations
    cam0_yaml = data_dir / 'cam0' / 'sensor.yaml'
    cam1_yaml = data_dir / 'cam1' / 'sensor.yaml'

    print("Loading camera calibrations...")
    calib0 = load_euroc_camera_calibration(cam0_yaml)
    calib1 = load_euroc_camera_calibration(cam1_yaml)

    print(f"cam0 resolution: {calib0['resolution']}")
    print(f"cam1 resolution: {calib1['resolution']}")
    print(f"cam0 intrinsics: {calib0['intrinsics']}")
    print(f"cam1 intrinsics: {calib1['intrinsics']}")

    # Compute stereo extrinsics
    print("\nComputing stereo extrinsics...")
    R, t, baseline = compute_stereo_extrinsics(calib0, calib1)
    print(f"Baseline: {baseline:.6f} m")
    print(f"Rotation R_C1C0:\n{R}")
    print(f"Translation t_C1C0: {t}")

    # Compute stereo rectification
    print("\nComputing stereo rectification...")
    rectify_maps = compute_stereo_rectification(calib0, calib1, R, t)
    print("Rectification computed successfully")

    # Load camera data
    cam0_csv = data_dir / 'cam0' / 'data.csv'
    cam0_data_dir = data_dir / 'cam0' / 'data'
    cam1_csv = data_dir / 'cam1' / 'data.csv'
    cam1_data_dir = data_dir / 'cam1' / 'data'

    print("\nLoading camera data...")
    cam0_data = load_euroc_camera_data(cam0_csv, cam0_data_dir)
    cam1_data = load_euroc_camera_data(cam1_csv, cam1_data_dir)
    print(f"cam0: {len(cam0_data)} images")
    print(f"cam1: {len(cam1_data)} images")

    # Synchronize stereo pairs
    print("\nSynchronizing stereo pairs...")
    stereo_pairs = synchronize_stereo_pairs(cam0_data, cam1_data)
    print(f"Found {len(stereo_pairs)} synchronized stereo pairs")

    if len(stereo_pairs) == 0:
        print("Error: No synchronized stereo pairs found!")
        return 1

    # Initialize Rerun
    print("\nInitializing Rerun...")
    rr.init("euroc_dense_stereo", spawn=True)
    # OpenCV camera frame: X=Right, Y=Down, Z=Forward
    # RFU: X=Right, Y=Forward, Z=Up (rotated 90° from RDF)
    rr.log("world", rr.ViewCoordinates.RFU, static=True)
    log_cameras_to_rerun(calib0, calib1)

    # Create stereo matcher (reuse for all frames)
    matcher = create_stereo_matcher(
        args.method, args.num_disparities, args.block_size)

    # Process all frames and log to Rerun
    print(f"\nProcessing {len(stereo_pairs)} frames...")
    for i, (idx, pair) in enumerate(stereo_pairs.iterrows()):
        if (i + 1) % 10 == 0 or i == 0:
            print(f"Processing frame {i+1}/{len(stereo_pairs)}...")

        # Load images
        img0 = cv2.imread(str(pair['cam0_path']))
        img1 = cv2.imread(str(pair['cam1_path']))

        if img0 is None or img1 is None:
            print(f"Warning: Could not load images for frame {i}, skipping...")
            continue

        # Process stereo frame
        points_3d, colors, valid_mask, img0_rect, img1_rect = process_stereo_frame(
            img0, img1, rectify_maps, matcher, calib0)

        # Log to Rerun
        log_frame_to_rerun(calib0, img0, img1, img0_rect, img1_rect,
                           points_3d, colors, valid_mask, pair['timestamp'])

    print(f"\nProcessed and logged {len(stereo_pairs)} frames to Rerun!")
    print("Done!")
    return 0


if __name__ == '__main__':
    exit(main())
