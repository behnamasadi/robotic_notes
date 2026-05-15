import numpy as np
import rerun as rr
import os


def parse_kitti_poses(file_path):
    """Read and parse KITTI poses from a text file."""
    poses = []
    with open(file_path, 'r') as f:
        for line in f:
            # Convert the line into a list of floats
            values = list(map(float, line.strip().split()))
            if len(values) == 12:
                # Reshape the list into a 3x4 matrix
                pose_matrix = np.array(values).reshape(3, 4)
                poses.append(pose_matrix)
    return poses


def opencv_to_opengl_coordinates(rotation, translation):
    """Convert OpenCV coordinates to OpenGL coordinates."""
    # Flip the Y and Z axes for OpenCV to OpenGL conversion
    flip_matrix = np.array([
        [1, 0, 0],
        [0, -1, 0],
        [0, 0, -1]
    ])
    new_rotation = flip_matrix @ rotation
    new_translation = flip_matrix @ translation
    return new_rotation, new_translation


def publish_poses_to_rerun(poses):
    """Publish poses to Rerun."""
    rr.init("kitti_poses", spawn=True)

    # Set view coordinates
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Y_DOWN, static=True)

    # Log arrows representing XYZ axes
    rr.log(
        "world/xyz",
        rr.Arrows3D(
            vectors=[[1, 0, 0], [0, 1, 0], [0, 0, 1]],
            colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
        ),
    )

    for i, pose in enumerate(poses):
        # Extract rotation (3x3) and translation (3x1)
        rotation = pose[:, :3]
        translation = pose[:, 3]

        # # Convert to OpenGL coordinates
        rotation, translation = opencv_to_opengl_coordinates(
            rotation, translation)

        # Define axes for visualization
        axes = np.eye(3)  # Identity matrix for X, Y, Z axes
        axes_transformed = np.dot(rotation, axes)

        # Log translation point
        rr.log(f"pose/{i}/translation",
               rr.Points3D(positions=[translation.tolist()]))

        # Log arrows representing XYZ axes
        rr.log(
            f"pose/{i}/axes",
            rr.Arrows3D(
                # All arrows start at the translation point
                origins=[translation.tolist()] * 3,
                vectors=axes_transformed.T.tolist(),  # Vectors for X, Y, Z axes
                # Red, Green, Blue for X, Y, Z
                colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
            ),
        )


if __name__ == "__main__":
    # Path to the KITTI poses fil, Relative to script directory
    kitti_poses_file = "../../data/kitti/odometry/05/poses/05.txt"

    # Use relative path based on the current script directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    kitti_poses_file_abs_path = os.path.join(script_dir, kitti_poses_file)
    print("reading poses from: ", kitti_poses_file_abs_path)

    # Parse poses
    poses = parse_kitti_poses(kitti_poses_file_abs_path)

    # Publish poses to Rerun
    publish_poses_to_rerun(poses)
