#!/usr/bin/env python3
"""
EuRoC MAV Dataset IMU Pose Estimation using AHRS and Rerun Visualization

This script:
1. Reads IMU data from EuRoC MAV Dataset format
2. Uses AHRS (Attitude and Heading Reference System) to estimate orientation
3. Performs dead reckoning to estimate position from IMU
4. Visualizes the estimated trajectory in Rerun
"""

import numpy as np
import pandas as pd
import rerun as rr
import yaml
from pathlib import Path
from scipy.spatial.transform import Rotation
from ahrs.filters import Madgwick, Mahony


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


def estimate_pose_with_ahrs(imu_data: pd.DataFrame, filter_type: str = 'madgwick') -> np.ndarray:
    """
    Estimate pose (position and orientation) from IMU data using AHRS filtering.

    Args:
        imu_data: DataFrame with IMU measurements
        filter_type: 'madgwick' or 'mahony'

    Returns:
        Array of shape (N, 7) where each row is [x, y, z, qw, qx, qy, qz]
    """
    N = len(imu_data)

    # Initialize filter
    if filter_type.lower() == 'madgwick':
        ahrs_filter = Madgwick(frequency=200.0)  # EuRoC IMU is 200 Hz
    else:
        ahrs_filter = Mahony(frequency=200.0)

    # Initialize state
    # Quaternion [w, x, y, z] representing initial orientation (identity = no rotation)
    quaternion = np.array([1.0, 0.0, 0.0, 0.0])

    # Position and velocity (initialize at origin)
    position = np.array([0.0, 0.0, 0.0])
    velocity = np.array([0.0, 0.0, 0.0])

    # Gravity vector in world frame (assuming Z-up, gravity points down)
    gravity_world = np.array([0.0, 0.0, -9.81])

    poses = np.zeros((N, 7))  # [x, y, z, qw, qx, qy, qz]

    for i in range(N):
        row = imu_data.iloc[i]
        dt = row['dt']

        if dt <= 0 or dt > 0.1:  # Skip invalid or too large dt
            dt = 1.0 / 200.0  # Default to 200 Hz

        # IMU measurements in sensor frame
        gyro = np.array([row['gyro_x'], row['gyro_y'], row['gyro_z']])
        accel = np.array([row['accel_x'], row['accel_y'], row['accel_z']])

        # Update orientation using AHRS filter
        try:
            quaternion = ahrs_filter.updateIMU(quaternion, gyr=gyro, acc=accel)
        except:
            # Fallback: integrate gyro only if filter fails
            dq = 0.5 * np.array([
                -quaternion[1] * gyro[0] - quaternion[2] *
                gyro[1] - quaternion[3] * gyro[2],
                quaternion[0] * gyro[0] + quaternion[2] *
                gyro[2] - quaternion[3] * gyro[1],
                quaternion[0] * gyro[1] - quaternion[1] *
                gyro[2] + quaternion[3] * gyro[0],
                quaternion[0] * gyro[2] + quaternion[1] *
                gyro[1] - quaternion[2] * gyro[0]
            ])
            quaternion = quaternion + dq * dt
            quaternion = quaternion / np.linalg.norm(quaternion)

        # Convert quaternion to rotation matrix
        rot = Rotation.from_quat(
            [quaternion[1], quaternion[2], quaternion[3], quaternion[0]])
        R = rot.as_matrix()

        # Transform acceleration from sensor frame to world frame
        # Subtract gravity in world frame
        accel_world = R @ accel
        accel_world = accel_world - gravity_world

        # Integrate acceleration to get velocity
        velocity = velocity + accel_world * dt

        # Integrate velocity to get position
        position = position + velocity * dt

        # Store pose: [x, y, z, qw, qx, qy, qz]
        poses[i] = np.concatenate([position, quaternion])

    return poses


def main():
    """Main function to process IMU data and visualize in Rerun."""
    import argparse

    parser = argparse.ArgumentParser(
        description='EuRoC IMU Pose Estimation with AHRS and Rerun')
    parser.add_argument(
        '--data-dir',
        type=str,
        default='data/machine_hall/MH_01_easy/MH_01_easy/mav0',
        help='Path to EuRoC dataset directory (relative to script directory or absolute)'
    )
    parser.add_argument(
        '--filter',
        type=str,
        choices=['madgwick', 'mahony'],
        default='madgwick',
        help='AHRS filter type: madgwick or mahony'
    )
    parser.add_argument(
        '--max-samples',
        type=int,
        default=None,
        help='Maximum number of IMU samples to process (for testing)'
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

    print(f"Loading IMU data from: {imu_csv_path}")
    imu_data = load_euroc_imu_data(imu_csv_path)

    if args.max_samples:
        imu_data = imu_data.head(args.max_samples)

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

    # Estimate poses
    print(f"Estimating poses using {args.filter} filter...")
    poses = estimate_pose_with_ahrs(imu_data, filter_type=args.filter)

    # Initialize Rerun
    rr.init("euroc_imu_ahrs", spawn=True)

    # Set coordinate system (Z-up, right-handed)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    # Configure series lines for IMU signals (for time series plots)
    xyz_axis_names = ["x", "y", "z"]
    xyz_axis_colors = [[255, 0, 0], [0, 255, 0], [0, 0, 255]]
    rr.log("/imu/accel", rr.SeriesLines(names=xyz_axis_names,
           colors=xyz_axis_colors), static=True)
    rr.log("/imu/gyro", rr.SeriesLines(names=xyz_axis_names,
           colors=xyz_axis_colors), static=True)

    # Log axes
    rr.log(
        "world/axes",
        rr.LineStrips3D(
            [
                [[0, 0, 0], [1, 0, 0]],  # X axis (red)
                [[0, 0, 0], [0, 1, 0]],  # Y axis (green)
                [[0, 0, 0], [0, 0, 1]],  # Z axis (blue)
            ],
            colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]]
        ),
        static=True
    )

    # Extract trajectory (positions)
    trajectory = poses[:, :3]

    # Calculate trajectory scale for camera visualization
    trajectory_span = np.max(trajectory, axis=0) - np.min(trajectory, axis=0)
    trajectory_size = np.linalg.norm(trajectory_span)
    print(f"Trajectory span: {trajectory_span}")
    print(f"Trajectory size: {trajectory_size:.2f} meters")

    # Scale camera image_plane_distance based on trajectory size
    # Use a fraction of the trajectory size (e.g., 5-10%) to make camera visible
    # At least 5m, or 5% of trajectory
    camera_scale = max(5.0, trajectory_size * 0.05)
    print(f"Camera image_plane_distance set to: {camera_scale:.2f} meters")

    # Log trajectory as line strip
    rr.log(
        "world/trajectory",
        rr.LineStrips3D([trajectory], colors=[255, 255, 0])
    )

    # Log raw IMU signals as time series using send_columns (more efficient)
    print("Logging IMU signals to Rerun...")
    timestamps_ns = (imu_data['timestamp'].values *
                     1e9).astype('datetime64[ns]')
    times = rr.TimeColumn("timestamp", timestamp=timestamps_ns)

    # Log accelerometer data
    accel = imu_data[['accel_x', 'accel_y', 'accel_z']].to_numpy()
    rr.send_columns(
        "/imu/accel", indexes=[times], columns=rr.Scalars.columns(scalars=accel))

    # Log gyroscope data
    gyro = imu_data[['gyro_x', 'gyro_y', 'gyro_z']].to_numpy()
    rr.send_columns(
        "/imu/gyro", indexes=[times], columns=rr.Scalars.columns(scalars=gyro))

    # Log camera setup (static intrinsics)
    if camera_data is not None and camera_intrinsics is not None:
        print("Logging camera setup...")
        rr.log(
            "/world/cam0",
            rr.Pinhole(
                focal_length=camera_intrinsics['focal_length'],
                resolution=camera_intrinsics['resolution'],
                image_plane_distance=camera_scale
            ),
            static=True
        )

    # Log camera images and poses
    if camera_data is not None:
        print("Logging camera images to Rerun...")
        for _, row in camera_data.iterrows():
            timestamp = row['timestamp']
            image_path = row['image_path']

            if image_path.exists():
                rr.set_time("timestamp", timestamp=timestamp)
                rr.log("/world/cam0/image",
                       rr.EncodedImage(path=str(image_path)))

    # Log camera poses (transform) using send_columns for efficiency
    if camera_data is not None:
        print("Logging camera poses to Rerun...")
        # Match poses to camera timestamps
        cam_timestamps = camera_data['timestamp'].values
        cam_timestamps_ns = (cam_timestamps * 1e9).astype('datetime64[ns]')
        times = rr.TimeColumn("timestamp", timestamp=cam_timestamps_ns)

        # Find closest pose for each camera timestamp
        translations = []
        quaternions = []
        for cam_timestamp in cam_timestamps:
            pose_idx = np.argmin(
                np.abs(imu_data['timestamp'].values - cam_timestamp))
            pos = poses[pose_idx, :3]
            quat = poses[pose_idx, 3:7]  # [qw, qx, qy, qz]
            # Convert to [x, y, z, w] format for Rerun
            quat_rerun = [quat[1], quat[2], quat[3], quat[0]]
            translations.append(pos)
            quaternions.append(quat_rerun)

        translations_df = pd.DataFrame(translations, columns=['x', 'y', 'z'])
        quaternions_df = pd.DataFrame(
            quaternions, columns=['x', 'y', 'z', 'w'])

        rr.send_columns(
            "/world/cam0",
            indexes=[times],
            columns=rr.Transform3D.columns(
                translation=translations_df,
                quaternion=quaternions_df,
            )
        )

    print(f"Done! Visualized {len(poses)} poses in Rerun")
    final_pos = poses[-1, :3]
    print(
        f"Trajectory length: {np.linalg.norm(trajectory[-1] - trajectory[0]):.2f} meters")
    print(
        f"Final position: [{final_pos[0]:.2f}, {final_pos[1]:.2f}, {final_pos[2]:.2f}]")

    return 0


if __name__ == '__main__':
    exit(main())
