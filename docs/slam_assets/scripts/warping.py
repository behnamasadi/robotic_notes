import torch
import torch.nn.functional as F
import numpy as np
import cv2
from pathlib import Path
import matplotlib.pyplot as plt

# ---------- Geometry utils ----------


def make_coords_grid(B, H, W, device, dtype):
    """Create homogeneous pixel coordinate grid [B,3,H,W] with [x, y, 1]."""
    y, x = torch.meshgrid(
        torch.arange(H, device=device, dtype=dtype),
        torch.arange(W, device=device, dtype=dtype),
        indexing="ij"
    )
    ones = torch.ones_like(x)
    pix = torch.stack([x, y, ones], dim=0).unsqueeze(0).repeat(B, 1, 1, 1)
    return pix  # [B,3,H,W]


def projective_warp(I_s, depth_t, T_t2s, K, K_inv):
    """
    Warp source image I_s into target view using depth_t and relative transform.
    I_s:    [B,3,H,W]
    depth_t:[B,1,H,W]
    T_t2s:  [B,4,4]
    K,K_inv:[B,3,3]
    """
    B, _, H, W = I_s.shape
    pix = make_coords_grid(B, H, W, I_s.device, I_s.dtype)
    cam_points = K_inv @ pix.reshape(B, 3, -1)
    cam_points = cam_points * depth_t.reshape(B, 1, -1)
    cam_h = torch.cat(
        [cam_points, torch.ones_like(cam_points[:, :1, :])], dim=1)

    P = K @ T_t2s[:, :3, :]
    proj = P @ cam_h
    z = proj[:, 2:3, :].clamp(min=1e-6)
    u = (proj[:, 0:1, :] / z).reshape(B, 1, H, W)
    v = (proj[:, 1:2, :] / z).reshape(B, 1, H, W)

    u_norm = 2 * (u / (W - 1)) - 1
    v_norm = 2 * (v / (H - 1)) - 1
    grid = torch.cat([u_norm, v_norm], dim=1).permute(0, 2, 3, 1)
    I_warp = F.grid_sample(I_s, grid, mode="bilinear",
                           padding_mode="border", align_corners=True)
    return I_warp

# ---------- KITTI sample loading ----------


def load_kitti_sample(data_root, seq="00", frame_idx=(0, 1)):
    """Load RGB images, depth (KITTI GT), intrinsics, and poses."""
    seq_path = Path(data_root) / "sequences" / seq
    pose_file = Path(data_root) / "poses" / f"{seq}.txt"
    K = np.array([[718.856, 0, 607.1928],
                  [0, 718.856, 185.2157],
                  [0, 0, 1]], dtype=np.float32)

    # Load images
    img_paths = [seq_path / "image_2" / f"{idx:06d}.png" for idx in frame_idx]
    imgs = [cv2.cvtColor(cv2.imread(str(p)), cv2.COLOR_BGR2RGB)
            for p in img_paths]
    H, W = imgs[0].shape[:2]
    imgs = [cv2.resize(im, (W//2, H//2)) for im in imgs]  # optional downsample
    H, W = imgs[0].shape[:2]

    # Load ground truth depth (if available, from KITTI depth completion dataset)
    depth_path = Path(data_root) / "depth" / f"{seq}_{frame_idx[0]:06d}.npy"
    if depth_path.exists():
        depth = np.load(depth_path)
        depth = cv2.resize(depth, (W, H))
    else:
        print("⚠️ No ground truth depth found. Generating dummy constant depth = 10m.")
        depth = np.ones((H, W), dtype=np.float32) * 10.0

    # Load ground truth poses (KITTI odometry format)
    poses = np.loadtxt(pose_file)
    T_w_0 = np.eye(4, dtype=np.float32)
    T_w_1 = np.eye(4, dtype=np.float32)
    T_w_0[:3, :] = poses[frame_idx[0]].reshape(3, 4)
    T_w_1[:3, :] = poses[frame_idx[1]].reshape(3, 4)
    T_t2s = np.linalg.inv(T_w_1) @ T_w_0  # transform target->source

    return imgs, depth, K, T_t2s

# ---------- Test ----------


def main():
    # 1️⃣ Load KITTI sample
    data_root = "/path/to/KITTI/odometry/"   # e.g. .../dataset/
    imgs, depth, K, T_t2s = load_kitti_sample(
        data_root, seq="00", frame_idx=(0, 1))

    # 2️⃣ Convert to torch tensors
    I_t = torch.from_numpy(imgs[0]).permute(
        2, 0, 1).unsqueeze(0).float() / 255.0
    I_s = torch.from_numpy(imgs[1]).permute(
        2, 0, 1).unsqueeze(0).float() / 255.0
    depth_t = torch.from_numpy(depth).unsqueeze(0).unsqueeze(0).float()
    K_t = torch.from_numpy(K).unsqueeze(0)
    K_inv = torch.inverse(K_t)
    T_t2s_t = torch.from_numpy(T_t2s).unsqueeze(0).float()

    # 3️⃣ Warp source into target frame
    with torch.no_grad():
        I_warp = projective_warp(I_s, depth_t, T_t2s_t, K_t, K_inv)

    # 4️⃣ Visualize
    I_t_np = I_t[0].permute(1, 2, 0).numpy()
    I_s_np = I_s[0].permute(1, 2, 0).numpy()
    I_warp_np = I_warp[0].permute(1, 2, 0).clamp(0, 1).numpy()

    fig, axs = plt.subplots(1, 3, figsize=(12, 4))
    axs[0].imshow(I_t_np)
    axs[0].set_title("Target frame (I_t)")
    axs[1].imshow(I_s_np)
    axs[1].set_title("Source frame (I_s)")
    axs[2].imshow(I_warp_np)
    axs[2].set_title("Warped source → target")
    for ax in axs:
        ax.axis("off")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
