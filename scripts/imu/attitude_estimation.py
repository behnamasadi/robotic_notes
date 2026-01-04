#!/usr/bin/env python3
"""
EuRoC MAV Dataset IMU Pose Estimation

This script:
1. Reads IMU data from EuRoC MAV Dataset format
2. Performs dead reckoning to estimate position from IMU
3. Visualizes the estimated trajectory in Rerun
"""

import numpy as np
import pandas as pd
import rerun as rr
import yaml
from pathlib import Path
from scipy.spatial.transform import Rotation
import argparse


def load_euroc_imu_data(imu_csv_path: Path) -> pd.DataFrame:
    """
    Load IMU data from EuRoC CSV format.

    Format: timestamp [ns], w_RS_S_x [rad/s], w_RS_S_y [rad/s], w_RS_S_z [rad/s],
            a_RS_S_x [m/s²], a_RS_S_y [m/s²], a_RS_S_z [m/s²]

    Returns DataFrame with columns: timestamp, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z
    """
    df = pd.read_csv(imu_csv_path, comment='#')
    df.columns = ['timestamp', 'gyro_x', 'gyro_y',
                  'gyro_z', 'accel_x', 'accel_y', 'accel_z']

    # Convert timestamp from nanoseconds to seconds
    df['timestamp'] = df['timestamp'] * 1e-9

    # Compute time differences
    df['dt'] = df['timestamp'].diff().fillna(0.0)

    return df


def load_euroc_camera_data(cam_csv_path: Path, cam_data_dir: Path) -> pd.DataFrame:
    """
    Load camera image timestamps and paths from EuRoC CSV format.

    Format: timestamp [ns], filename

    Returns DataFrame with columns: timestamp, filename, image_path
    """
    df = pd.read_csv(cam_csv_path, comment='#')
    df.columns = ['timestamp', 'filename']

    # Convert timestamp from nanoseconds to seconds
    df['timestamp'] = df['timestamp'] * 1e-9

    # Build full image paths
    df['image_path'] = df['filename'].apply(lambda f: cam_data_dir / f)

    return df


def load_euroc_imu_sensor_config(sensor_yaml_path: Path) -> dict:
    """
    Load IMU sensor configuration from EuRoC sensor.yaml file.

    Returns dict with:
    - T_BS: 4x4 transformation matrix from body frame to sensor frame
    - rate_hz: Sampling rate in Hz
    - gyroscope_noise_density: Gyro white noise [rad/s/sqrt(Hz)]
    - gyroscope_random_walk: Gyro bias diffusion [rad/s^2/sqrt(Hz)]
    - accelerometer_noise_density: Accel white noise [m/s^2/sqrt(Hz)]
    - accelerometer_random_walk: Accel bias diffusion [m/s^3/sqrt(Hz)]
    """
    with open(sensor_yaml_path, 'r') as f:
        sensor_data = yaml.safe_load(f)

    # Parse T_BS transformation matrix
    t_bs_data = sensor_data['T_BS']['data']
    T_BS = np.array(t_bs_data).reshape(4, 4)

    return {
        'T_BS': T_BS,
        'rate_hz': sensor_data['rate_hz'],
        'gyroscope_noise_density': sensor_data['gyroscope_noise_density'],
        'gyroscope_random_walk': sensor_data['gyroscope_random_walk'],
        'accelerometer_noise_density': sensor_data['accelerometer_noise_density'],
        'accelerometer_random_walk': sensor_data['accelerometer_random_walk'],
    }


def load_euroc_camera_intrinsics(sensor_yaml_path: Path) -> dict:
    """
    Load camera intrinsics from EuRoC sensor.yaml file.

    Returns dict with: resolution, focal_length, principal_point, etc.
    """
    with open(sensor_yaml_path, 'r') as f:
        sensor_data = yaml.safe_load(f)

    intrinsics = sensor_data['intrinsics']  # [fu, fv, cu, cv]
    resolution = sensor_data['resolution']  # [width, height]

    return {
        'resolution': resolution,
        'focal_length': [intrinsics[0], intrinsics[1]],  # [fx, fy]
        'principal_point': [intrinsics[2], intrinsics[3]],  # [cx, cy]
    }


def main():
    """Main function to process IMU data and visualize in Rerun."""

    parser = argparse.ArgumentParser(
        description='EuRoC IMU Pose Estimation with AHRS and Rerun')
    parser.add_argument(
        '--data-dir',
        type=str,
        default='../data/machine_hall/MH_01_easy/MH_01_easy/mav0',
        help='Path to EuRoC dataset directory (relative to script directory or absolute)'
    )
    parser.add_argument(
        '--imu-sensor-yaml',
        type=str,
        default=None,
        help='Path to IMU sensor.yaml file (default: <data-dir>/imu0/sensor.yaml)'
    )

    args = parser.parse_args()

    # Get script directory and resolve data path
    script_dir = Path(__file__).resolve().parent.parent
    data_dir = Path(args.data_dir)
    if not data_dir.is_absolute():
        data_dir = script_dir / data_dir

    imu_csv_path = data_dir / 'imu0' / 'data.csv'

    if not imu_csv_path.exists():
        print(f"Error: IMU data file not found at {imu_csv_path}")
        print(f"Please check the --data-dir argument")
        return 1

    # Determine IMU sensor.yaml path
    if args.imu_sensor_yaml:
        imu_sensor_yaml_path = Path(args.imu_sensor_yaml)
        if not imu_sensor_yaml_path.is_absolute():
            imu_sensor_yaml_path = script_dir / imu_sensor_yaml_path
    else:
        imu_sensor_yaml_path = data_dir / 'imu0' / 'sensor.yaml'

    # Load IMU sensor configuration
    imu_sensor_config = None
    if imu_sensor_yaml_path.exists():
        print(f"Loading IMU sensor configuration from: {imu_sensor_yaml_path}")
        imu_sensor_config = load_euroc_imu_sensor_config(imu_sensor_yaml_path)

        print(f"\n=== IMU Sensor Configuration ===")
        print(f"Sampling rate: {imu_sensor_config['rate_hz']} Hz")

        # Print T_BS transformation matrix (sensor frame in body frame)
        T_BS = imu_sensor_config['T_BS']
        print(f"\nT_BS (sensor frame in body frame, 4x4 transformation matrix):")
        print(T_BS)

        # Extract rotation and translation components
        R_BS = T_BS[:3, :3]  # Rotation matrix
        t_BS = T_BS[:3, 3]   # Translation vector
        print(f"\nRotation R_BS (sensor to body):")
        print(R_BS)
        print(f"Translation t_BS (sensor origin in body frame): {t_BS}")

        # Print noise parameters
        print(f"\nNoise Parameters:")
        print(
            f"  Gyroscope noise density: {imu_sensor_config['gyroscope_noise_density']:.6e} rad/s/sqrt(Hz)")
        print(
            f"  Gyroscope random walk: {imu_sensor_config['gyroscope_random_walk']:.6e} rad/s²/sqrt(Hz)")
        print(
            f"  Accelerometer noise density: {imu_sensor_config['accelerometer_noise_density']:.6e} m/s²/sqrt(Hz)")
        print(
            f"  Accelerometer random walk: {imu_sensor_config['accelerometer_random_walk']:.6e} m/s³/sqrt(Hz)")
    else:
        print(
            f"Warning: IMU sensor.yaml not found at {imu_sensor_yaml_path}, continuing without sensor config")

    print(f"Loading IMU data from: {imu_csv_path}")
    imu_data = load_euroc_imu_data(imu_csv_path)

    print(f"Loaded {len(imu_data)} IMU samples")
    print(
        f"Time span: {imu_data['timestamp'].iloc[0]:.3f} to {imu_data['timestamp'].iloc[-1]:.3f} seconds")

    # Load camera data if available
    cam_csv_path = data_dir / 'cam0' / 'data.csv'
    cam_data_dir = data_dir / 'cam0' / 'data'
    cam_sensor_yaml = data_dir / 'cam0' / 'sensor.yaml'
    camera_data = None
    camera_intrinsics = None

    if cam_csv_path.exists() and cam_data_dir.exists() and cam_sensor_yaml.exists():
        print(f"Loading camera data from: {cam_csv_path}")
        camera_data = load_euroc_camera_data(cam_csv_path, cam_data_dir)
        camera_intrinsics = load_euroc_camera_intrinsics(cam_sensor_yaml)
        print(f"Loaded {len(camera_data)} camera images")
        print(
            f"Camera time span: {camera_data['timestamp'].iloc[0]:.3f} to {camera_data['timestamp'].iloc[-1]:.3f} seconds")
        print(f"Camera resolution: {camera_intrinsics['resolution']}")
        print(f"Camera focal length: {camera_intrinsics['focal_length']}")
    else:
        print("Warning: Camera data not found, skipping camera visualization")

    print(int(1/imu_data['dt'][5]))

    # Initialize orientation using accelerometer, Before integrating anything, you must initialize gravity.
    # Assumption: First ~0.5–1 second is static, sampling is 200Hz so for half second we have 100 sample
    N = 100

    a_x, a_y, a_z = 0, 0, 0
    accel = imu_data[['accel_x', 'accel_y', 'accel_z']].to_numpy()
    for i in range(N):
        a_x = a_x+accel[i][0]
        a_y = a_y+accel[i][1]
        a_z = a_z+accel[i][2]
    # Compute mean acceleration:
    gravity_vector = np.array([a_x/N, a_y/N, a_z/N])
    print(f"Gravity vector: {gravity_vector}")

    # Estimate gravity direction:
    gravity_magnitude = np.linalg.norm(gravity_vector)
    gravity_direction = gravity_vector / gravity_magnitude
    print(f"Gravity magnitude: {gravity_magnitude:.3f} m/s²")
    print(f"Gravity direction (unit vector): {gravity_direction}")

    # Find rotation Rwb such that: Rwb @ g_body = [0, 0, -1]^T
    # R_wb transforms vectors from body frame to world frame
    #
    # NOTE: This rotation is NOT unique! We have 1 degree of freedom remaining:
    # - Gravity alignment constrains 2 DOF (pitch and roll)
    # - Rotation about the vertical axis (yaw) is unconstrained
    # - There are infinitely many R_wb that satisfy R_wb @ g_body = [0, 0, -1]^T
    # - To make it unique, we need additional constraints (e.g., magnetometer for heading)
    g_body = gravity_direction  # Gravity direction in body frame
    # Target gravity direction in world frame (pointing down)
    g_world = np.array([0, 0, -1])

    # Use scipy to find the rotation that aligns g_body to g_world
    # align_vectors(a, b) finds R such that R @ a ≈ b
    # This returns ONE solution (arbitrary yaw), not the unique solution
    rotation, _ = Rotation.align_vectors([g_body], [g_world])
    R_wb = rotation.as_matrix()

    # Extract Euler angles to see the yaw ambiguity
    euler = rotation.as_euler('xyz', degrees=True)
    print(
        f"\nEuler angles (xyz, degrees): roll={euler[0]:.2f}, pitch={euler[1]:.2f}, yaw={euler[2]:.2f}")
    print("Note: Yaw is arbitrary - any rotation about the vertical axis would work")

    # Verify the rotation: R_wb @ g_body should equal [0, 0, -1]^T
    g_world_computed = R_wb @ g_body
    print(f"\nRotation matrix R_wb (body to world):")
    print(R_wb)
    print(f"\nVerification: R_wb @ g_body = {g_world_computed}")
    print(f"Target: [0, 0, -1]^T")
    print(f"Error: {np.linalg.norm(g_world_computed - g_world):.6f}")

    # ===================================================================
    # Step 1: Gyro bias initialization
    # ===================================================================
    print("\n=== Step 1: Gyro Bias Initialization ===")
    N = 100  # same window as gravity init
    gyro = imu_data[['gyro_x', 'gyro_y', 'gyro_z']].to_numpy()
    accel = imu_data[['accel_x', 'accel_y', 'accel_z']].to_numpy()

    gyro_bias = gyro[:N].mean(axis=0)
    print(f"Estimated gyro bias: {gyro_bias} rad/s")
    print(f"Gyro bias magnitude: {np.linalg.norm(gyro_bias):.6e} rad/s")

    # ===================================================================
    # Step 2 & 3: AHRS Loop - Gyro Integration + Accelerometer Correction
    # ===================================================================
    print("\n=== Step 2 & 3: AHRS Loop (Gyro Integration + Accel Correction) ===")

    # Initialize orientation
    R = Rotation.from_matrix(R_wb.copy())

    # AHRS parameters
    alpha = 0.01  # Accelerometer correction gain (try 0.005–0.02)

    # Storage for logging
    orientations = []
    euler_angles = []
    gravity_errors = []
    timestamps = []

    print(f"Starting AHRS loop with {len(imu_data)} samples...")
    print(f"Accelerometer correction gain: {alpha}")

    for k in range(1, len(imu_data)):
        dt = imu_data['dt'].iloc[k]
        if dt <= 0:
            continue

        # Step 2: Gyro integration
        omega = gyro[k] - gyro_bias  # rad/s, bias-corrected
        delta_rot = Rotation.from_rotvec(omega * dt)
        R = R * delta_rot  # right-multiplication: body-frame gyro

        # Step 3: Accelerometer correction
        acc = accel[k]
        acc_norm = np.linalg.norm(acc)

        if acc_norm > 1e-6:  # Avoid division by zero
            # Normalize measured acceleration (gravity direction in body frame)
            g_meas = acc / acc_norm

            # Predicted gravity direction (from current orientation)
            # R.inv() transforms from world to body, so R.inv().apply([0,0,-1]) gives gravity in body frame
            g_pred = R.inv().apply([0, 0, -1])

            # Compute error (cross product gives correction axis)
            e = np.cross(g_pred, g_meas)

            # Apply complementary correction
            delta_corr = Rotation.from_rotvec(alpha * e)
            R = delta_corr * R

        # Logging (every 100 samples for efficiency)
        if k % 100 == 0 or k == len(imu_data) - 1:
            orientations.append(R.as_matrix().copy())
            euler = R.as_euler('xyz', degrees=True)
            euler_angles.append(euler)
            timestamps.append(imu_data['timestamp'].iloc[k])

            # Gravity alignment error
            g_pred = R.inv().apply([0, 0, -1])
            if acc_norm > 1e-6:
                g_meas = acc / acc_norm
                gravity_error = np.linalg.norm(g_pred - g_meas)
            else:
                gravity_error = np.nan
            gravity_errors.append(gravity_error)

    print(f"AHRS loop completed. Processed {len(imu_data)} samples.")

    # ===================================================================
    # Step 4: Visualization & Sanity Checks
    # ===================================================================
    print("\n=== Step 4: Sanity Checks ===")

    # Final orientation
    final_euler = R.as_euler('xyz', degrees=True)
    print(f"\nFinal Euler angles (xyz, degrees):")
    print(f"  Roll:  {final_euler[0]:.2f}°")
    print(f"  Pitch: {final_euler[1]:.2f}°")
    print(f"  Yaw:   {final_euler[2]:.2f}°")

    # Orthonormality check
    R_matrix = R.as_matrix()
    ortho_error = np.linalg.norm(R_matrix.T @ R_matrix - np.eye(3))
    print(f"\nRotation matrix orthonormality error: {ortho_error:.2e}")
    print(f"  (Should be close to 0, < 1e-10 is good)")

    # Gravity alignment check
    g_pred_final = R.inv().apply([0, 0, -1])
    acc_final = accel[-1]
    acc_norm_final = np.linalg.norm(acc_final)
    if acc_norm_final > 1e-6:
        g_meas_final = acc_final / acc_norm_final
        gravity_error_final = np.linalg.norm(g_pred_final - g_meas_final)
        print(f"\nFinal gravity alignment error: {gravity_error_final:.6f}")
        print(f"  (Should stay small, < 0.1 is good)")

    # Statistics
    if gravity_errors:
        valid_errors = [e for e in gravity_errors if not np.isnan(e)]
        if valid_errors:
            print(f"\nGravity alignment error statistics:")
            print(f"  Mean: {np.mean(valid_errors):.6f}")
            print(f"  Std:  {np.std(valid_errors):.6f}")
            print(f"  Max:  {np.max(valid_errors):.6f}")

    # Euler angles evolution (first, middle, last)
    if len(euler_angles) >= 3:
        print(f"\nEuler angles evolution:")
        print(
            f"  Initial:  roll={euler_angles[0][0]:.2f}°, pitch={euler_angles[0][1]:.2f}°, yaw={euler_angles[0][2]:.2f}°")
        mid_idx = len(euler_angles) // 2
        print(
            f"  Middle:   roll={euler_angles[mid_idx][0]:.2f}°, pitch={euler_angles[mid_idx][1]:.2f}°, yaw={euler_angles[mid_idx][2]:.2f}°")
        print(
            f"  Final:    roll={euler_angles[-1][0]:.2f}°, pitch={euler_angles[-1][1]:.2f}°, yaw={euler_angles[-1][2]:.2f}°")

        # Check for drift
        roll_drift = abs(euler_angles[-1][0] - euler_angles[0][0])
        pitch_drift = abs(euler_angles[-1][1] - euler_angles[0][1])
        yaw_drift = abs(euler_angles[-1][2] - euler_angles[0][2])

        print(f"\nDrift analysis:")
        print(f"  Roll drift:  {roll_drift:.2f}° (should be small)")
        print(f"  Pitch drift: {pitch_drift:.2f}° (should be small)")
        print(f"  Yaw drift:   {yaw_drift:.2f}° (expected to drift)")

    exit()


#    # Log raw IMU signals as time series using send_columns(more efficient)
    # print("Logging IMU signals to Rerun...")
    # timestamps_ns = (imu_data['timestamp'].values *1e9).astype('datetime64[ns]')
    # times = rr.TimeColumn("timestamp", timestamp=timestamps_ns)

    # # Log accelerometer data
    # accel = imu_data[['accel_x', 'accel_y', 'accel_z']].to_numpy()
    # rr.send_columns("/imu/accel", indexes=[times], columns=rr.Scalars.columns(scalars=accel))

    # # Log gyroscope data
    # gyro = imu_data[['gyro_x', 'gyro_y', 'gyro_z']].to_numpy()
    # rr.send_columns("/imu/gyro", indexes=[times], columns=rr.Scalars.columns(scalars=gyro))

    return 0


if __name__ == '__main__':
    exit(main())
