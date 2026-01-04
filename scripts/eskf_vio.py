#!/usr/bin/env python3
"""
EuRoC ESKF VIO

This script performs ESKF VIO from EuRoC camera data:
1. Loads calibration from cam0 and cam1
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


def main():
    parser = argparse.ArgumentParser(
        description='EuRoC Dense Stereo Reconstruction')
    parser.add_argument(
        '--data-dir',
        type=str,
        default='../data/machine_hall/MH_01_easy/MH_01_easy/mav0',
        help='Path to EuRoC dataset directory'
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
    rr.init("euroc_monocular_eskf_vio", spawn=True)

    rr.log("world", static=True)

    print("Done!")
    return 0


if __name__ == '__main__':
    exit(main())
