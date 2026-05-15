import numpy as np
import torch
import math
import matplotlib.pyplot as plt


def plot_camera_motion(T_rel):
    t = T_rel[:3, 3]
    plt.figure()
    plt.quiver(0, 0, t[0], t[2], angles='xy',
               scale_units='xy', scale=1, color='r')
    plt.xlabel("X (m)")
    plt.ylabel("Z (m)")
    plt.title("Relative camera motion (KITTI)")
    plt.axis("equal")
    plt.show()

# ---------- Function under test ----------


def se3_to_SE3(pose_6d):
    """
    Converts axis-angle (3) + translation (3) vector to SE(3) 4×4 matrix.
    pose_6d: [B,6] tensor -> [B,4,4] tensor
    """
    B = pose_6d.shape[0]
    r = pose_6d[:, :3]
    t = pose_6d[:, 3:].unsqueeze(-1)
    theta = torch.linalg.norm(r, dim=1, keepdim=True).clamp(min=1e-8)
    k = r / theta

    # Build skew-symmetric matrix
    K = torch.zeros(B, 3, 3, device=pose_6d.device)
    K[:, 0, 1] = -k[:, 2]
    K[:, 0, 2] = k[:, 1]
    K[:, 1, 0] = k[:, 2]
    K[:, 1, 2] = -k[:, 0]
    K[:, 2, 0] = -k[:, 1]
    K[:, 2, 1] = k[:, 0]

    I = torch.eye(3, device=pose_6d.device).unsqueeze(0).expand(B, -1, -1)
    sin = torch.sin(theta)[:, 0].unsqueeze(-1).unsqueeze(-1)
    cos = torch.cos(theta)[:, 0].unsqueeze(-1).unsqueeze(-1)
    kkt = k.unsqueeze(2) @ k.unsqueeze(1)
    R = I + sin * K + (1 - cos) * kkt

    T = torch.eye(4, device=pose_6d.device).unsqueeze(0).repeat(B, 1, 1)
    T[:, :3, :3] = R
    T[:, :3, 3:] = t
    return T


# ---------- Helper functions ----------
def rotation_matrix_to_axis_angle(R):
    """Convert a 3×3 rotation matrix to an axis–angle vector."""
    theta = np.arccos(np.clip((np.trace(R) - 1) / 2.0, -1.0, 1.0))
    if theta < 1e-8:
        return np.zeros(3, dtype=np.float32)
    axis = (1 / (2 * np.sin(theta))) * np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1],
    ])
    return axis * theta  # (axis * angle) vector


# ---------- KITTI test data ----------
def load_kitti_poses(seq_path="poses/00.txt", idx0=0, idx1=1):
    """Load two KITTI ground truth poses and compute relative motion."""
    poses = np.loadtxt(seq_path)
    T_w_0 = np.eye(4, dtype=np.float32)
    T_w_1 = np.eye(4, dtype=np.float32)
    T_w_0[:3, :] = poses[idx0].reshape(3, 4)
    T_w_1[:3, :] = poses[idx1].reshape(3, 4)
    T_rel = np.linalg.inv(T_w_1) @ T_w_0  # transform from frame 1 → 0
    return T_w_0, T_w_1, T_rel


# ---------- Test ----------
def main():
    # 1️⃣ Load two poses from KITTI
    seq_pose_file = "/path/to/KITTI/odometry/poses/00.txt"
    T_w_0, T_w_1, T_rel_gt = load_kitti_poses(seq_pose_file, 0, 1)
    R_gt = T_rel_gt[:3, :3]
    t_gt = T_rel_gt[:3, 3]

    print("Ground truth relative transform:")
    print(T_rel_gt)

    # 2️⃣ Convert GT rotation matrix to axis-angle representation
    r_axisangle = rotation_matrix_to_axis_angle(R_gt)
    print("Axis–angle (r):", r_axisangle)
    print("Translation (t):", t_gt)

    # 3️⃣ Create a torch tensor and run through se3_to_SE3()
    pose_6d = torch.from_numpy(np.concatenate(
        [r_axisangle, t_gt])).unsqueeze(0).float()
    T_recon = se3_to_SE3(pose_6d)[0].cpu().numpy()

    print("\nReconstructed transform:")
    print(T_recon)

    # 4️⃣ Compare with ground truth
    R_err = np.linalg.norm(R_gt - T_recon[:3, :3])
    t_err = np.linalg.norm(t_gt - T_recon[:3, 3])
    print(f"\nRotation matrix error: {R_err:.6f}")
    print(f"Translation error:     {t_err:.6f}")

    # Check small-angle consistency
    angle_gt = np.degrees(np.linalg.norm(r_axisangle))
    print(f"Rotation angle (deg):  {angle_gt:.4f}")


if __name__ == "__main__":
    main()
