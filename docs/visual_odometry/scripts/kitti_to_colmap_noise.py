import numpy as np
import pandas as pd
import os.path
import argparse
import matplotlib.pyplot as plt


def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.

    Input
      :param roll: The roll (rotation around x-axis) angle in radians.
      :param pitch: The pitch (rotation around y-axis) angle in radians.
      :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
      :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - \
        np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + \
        np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - \
        np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + \
        np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return np.array([qw, qx, qy, qz])


def rotation_matrix_to_quaternion(matrix):
    """Convert a rotation matrix to quaternion."""
    q0 = np.sqrt(1 + matrix[0, 0] + matrix[1, 1] + matrix[2, 2]) / 2
    q1 = (matrix[2, 1] - matrix[1, 2]) / (4 * q0)
    q2 = (matrix[0, 2] - matrix[2, 0]) / (4 * q0)
    q3 = (matrix[1, 0] - matrix[0, 1]) / (4 * q0)
    return np.array([q0, q1, q2, q3])


def convertKITTIgtToColmap(KITTIfilepath):
    """Processes the file specified by filepath."""
    if not os.path.exists(KITTIfilepath):
        raise FileNotFoundError(f"File not found: {KITTIfilepath}")
        exit(1)

    print('result will be written into images.txt')
    poses = pd.read_csv(KITTIfilepath, sep=" ", header=None)

    # r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz

    # colmap format:
    # IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME

    CAMERA_ID = 1
    NAME = ""
    length = 6
    gt = np.zeros((len(poses), 3, 4))
    with open("images.txt", "w") as f:
        for i in range(len(poses)):
            gt[i] = np.array(poses.iloc[i]).reshape((3, 4))

            rotation_matrix = np.array(poses.iloc[i]).reshape((3, 4))[:, 0:3]
            translation_vector = np.array(poses.iloc[i]).reshape((3, 4))[:, 3]

            # -R^t * T
            translation_vector = np.dot(- np.transpose(rotation_matrix),
                                        translation_vector)

            rows = 3
            cols = 1
            rnd = 0.5*(np.random.random(size=[rows, cols])-0.5).squeeze()

            translation_vector = translation_vector+rnd

            rotation_matrix = np.transpose(rotation_matrix)

            quat = rotation_matrix_to_quaternion(rotation_matrix)

            rows = 4
            cols = 1
            # quat = quat+0.05*(np.random.random(size=[rows, cols])-0.5).squeeze()
            quat = quat/np.linalg.norm(quat)

            QW, QX, QY, QZ = quat

            TX, TY, TZ = translation_vector
            IMAGE_ID = i+1
            # IMAGE_ID = i

            # This creates a format string like "{:0>5}" for length = 5
            format_string = f"{{:0>{length}}}"
            NAME = format_string.format(i)+".png"

            str = f"{IMAGE_ID} {QW} {QX} {QY} {QZ} {TX} {TY} {TZ} {CAMERA_ID} {NAME}".format(
                IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME)
            # print(str)

            f.writelines(str)
            f.writelines('\n\n')

    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(gt[:, :, 3][:, 0], gt[:, :, 3][:, 1], gt[:, :, 3][:, 2])
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    # ax.view_init(elev=-40, azim=270)
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="convert a KITI ground truth file to colmap file.")
    parser.add_argument(
        "-f", "--file",
        type=str,
        help="Path to the input file. If not provided, a default file will be used.")

    args = parser.parse_args()

    # Define your default file name
    default_file = "../../data/kitti/odometry/05/poses/05.txt"

    filepath = args.file

    if filepath is None:
        print(f"No input file provided, Using default file at: {default_file}")
        filepath = default_file

    try:
        convertKITTIgtToColmap(filepath)
    except FileNotFoundError:
        print(f"Error: File '{filepath}' not found.  Exiting.")
        exit(1)  # Exit with error code indicating the file issue
    except Exception as e:  # Catch other potential errors
        print(f"An error occurred: {e}")
        exit(1)
