from warping import projective_warp, make_coords_grid
from SSIM import SSIMLoss
import matplotlib.pyplot as plt
import cv2
import numpy as np
import torch.nn.functional as F
import torch.nn as nn
import torch
import sys
from pathlib import Path

# Add scripts directory to path for local imports
scripts_dir = Path(__file__).resolve().parent
if str(scripts_dir) not in sys.path:
    sys.path.insert(0, str(scripts_dir))


# Import SSIM and warping from local modules


# ---------- Photometric Loss Function ----------
class PhotometricLoss(nn.Module):
    """
    Photometric loss combining SSIM and L1 loss.
    """

    def __init__(self, alpha=0.85):
        super().__init__()
        self.alpha = alpha
        self.ssim = SSIMLoss()

    def forward(self, I_target, I_warped):
        """
        Args:
            I_target: Target image [B, 3, H, W] in range [0, 1]
            I_warped: Warped source image [B, 3, H, W] in range [0, 1]

        Returns:
            photometric_loss: Scalar loss value
        """
        # SSIM loss (already returns 1 - SSIM, where 0 is perfect match)
        ssim_loss = self.ssim(I_target, I_warped)

        # L1 loss
        l1_loss = torch.abs(I_target - I_warped).mean()

        # Combine losses
        photo_loss = self.alpha * ssim_loss + (1 - self.alpha) * l1_loss

        return photo_loss


# ---------- Data Loading Functions ----------
def load_calibration(calib_file):
    """
    Load camera intrinsics from KITTI calibration file.

    Returns:
        K: Camera intrinsic matrix [3, 3]
    """
    with open(calib_file, 'r') as f:
        lines = f.readlines()

    # Parse P2 (left color camera projection matrix)
    p2_line = [line for line in lines if line.startswith('P2:')][0]
    P2 = np.array([float(x) for x in p2_line.split()[1:]]).reshape(3, 4)

    # Extract intrinsic matrix K from P2
    K = P2[:3, :3]

    return K.astype(np.float32)


def load_poses(pose_file):
    """
    Load camera poses from KITTI poses file.

    Args:
        pose_file: Path to poses txt file

    Returns:
        poses: List of 4x4 transformation matrices
    """
    poses_data = np.loadtxt(pose_file)
    poses = []

    for pose_line in poses_data:
        T = np.eye(4, dtype=np.float32)
        T[:3, :] = pose_line.reshape(3, 4)
        poses.append(T)

    return poses


def load_images(images_dir, indices=(0, 1)):
    """
    Load images from directory.

    Args:
        images_dir: Directory containing images
        indices: Tuple of image indices to load

    Returns:
        images: List of loaded images (RGB, uint8)
    """
    images = []
    for idx in indices:
        img_path = Path(images_dir) / f"{idx:06d}.png"
        img = cv2.imread(str(img_path))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        images.append(img)

    return images


def compute_relative_transform(T_w_target, T_w_source):
    """
    Compute relative transform from target to source frame.

    Args:
        T_w_target: World to target transform [4, 4]
        T_w_source: World to source transform [4, 4]

    Returns:
        T_target_to_source: Relative transform [4, 4]
    """
    T_target_to_source = np.linalg.inv(T_w_source) @ T_w_target
    return T_target_to_source


# ---------- Main Demo ----------
def main():
    # Setup paths
    script_dir = Path(__file__).resolve().parent
    slam_dir = script_dir.parent

    images_dir = slam_dir / "images"
    calib_file = slam_dir / "data_odometry_calib" / "calib.txt"
    poses_file = slam_dir / "poses" / "00.txt"

    print("=" * 60)
    print("Photometric Loss Calculation Demo")
    print("=" * 60)

    # 1. Load calibration
    print("\n1. Loading calibration...")
    K = load_calibration(calib_file)
    print(f"Camera intrinsics K:\n{K}")

    # 2. Load poses
    print("\n2. Loading poses...")
    poses = load_poses(poses_file)
    print(f"Loaded {len(poses)} poses")

    # 3. Load images
    print("\n3. Loading images...")
    frame_indices = (0, 1)  # Consecutive frames
    images = load_images(images_dir, frame_indices)
    print(f"Image shapes: {[img.shape for img in images]}")

    H, W = images[0].shape[:2]

    # 4. Compute relative transform
    print("\n4. Computing relative transform...")
    T_w_0 = poses[frame_indices[0]]
    T_w_1 = poses[frame_indices[1]]
    T_0to1 = compute_relative_transform(T_w_0, T_w_1)
    print(f"T_0to1:\n{T_0to1}")

    # 5. Create dummy depth (since we don't have ground truth depth)
    print("\n5. Creating dummy depth map...")
    # In real scenario, this would come from a depth network
    depth_mean = 10.0  # Average depth in meters
    depth = np.ones((H, W), dtype=np.float32) * depth_mean
    print(f"Depth shape: {depth.shape}, mean: {depth_mean}m")

    # 6. Convert to PyTorch tensors
    print("\n6. Converting to PyTorch tensors...")
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"Using device: {device}")

    I_target = torch.from_numpy(images[0]).permute(
        2, 0, 1).unsqueeze(0).float() / 255.0
    I_source = torch.from_numpy(images[1]).permute(
        2, 0, 1).unsqueeze(0).float() / 255.0
    depth_t = torch.from_numpy(depth).unsqueeze(0).unsqueeze(0).float()
    K_t = torch.from_numpy(K).unsqueeze(0)
    K_inv = torch.inverse(K_t)
    T_t2s = torch.from_numpy(T_0to1).unsqueeze(0).float()

    # Move to device
    I_target = I_target.to(device)
    I_source = I_source.to(device)
    depth_t = depth_t.to(device)
    K_t = K_t.to(device)
    K_inv = K_inv.to(device)
    T_t2s = T_t2s.to(device)

    # 7. Warp source image to target frame
    print("\n7. Warping source image to target frame...")
    with torch.no_grad():
        I_warped = projective_warp(I_source, depth_t, T_t2s, K_t, K_inv)
    print(f"Warped image shape: {I_warped.shape}")

    # 8. Compute photometric loss
    print("\n8. Computing photometric loss...")
    photo_loss_fn = PhotometricLoss(alpha=0.85).to(device)

    with torch.no_grad():
        photo_loss = photo_loss_fn(I_target, I_warped)

    print(f"\n{'='*60}")
    print(f"PHOTOMETRIC LOSS: {photo_loss.item():.6f}")
    print(f"{'='*60}")

    # 9. Visualize results
    print("\n9. Visualizing results...")
    I_target_vis = I_target[0].permute(1, 2, 0).cpu().numpy()
    I_source_vis = I_source[0].permute(1, 2, 0).cpu().numpy()
    I_warped_vis = I_warped[0].permute(1, 2, 0).cpu().clamp(0, 1).numpy()

    # Compute error map
    error_map = torch.abs(I_target - I_warped).mean(dim=1, keepdim=True)
    error_map_vis = error_map[0, 0].cpu().numpy()

    fig, axs = plt.subplots(2, 2, figsize=(12, 8))

    axs[0, 0].imshow(I_target_vis)
    axs[0, 0].set_title(f"Target Frame (t={frame_indices[0]})")
    axs[0, 0].axis('off')

    axs[0, 1].imshow(I_source_vis)
    axs[0, 1].set_title(f"Source Frame (t={frame_indices[1]})")
    axs[0, 1].axis('off')

    axs[1, 0].imshow(I_warped_vis)
    axs[1, 0].set_title("Warped Source → Target")
    axs[1, 0].axis('off')

    im = axs[1, 1].imshow(error_map_vis, cmap='hot')
    axs[1, 1].set_title(
        f"Photometric Error Map\nLoss = {photo_loss.item():.6f}")
    axs[1, 1].axis('off')
    plt.colorbar(im, ax=axs[1, 1], fraction=0.046, pad=0.04)

    plt.tight_layout()
    plt.savefig(slam_dir / 'photometric_loss_result.png',
                dpi=150, bbox_inches='tight')
    print(
        f"\nSaved visualization to: {slam_dir / 'photometric_loss_result.png'}")
    plt.show()

    # 10. Additional statistics
    print("\n" + "="*60)
    print("Additional Statistics:")
    print("="*60)
    print(
        f"Mean pixel difference (L1): {torch.abs(I_target - I_warped).mean().item():.6f}")
    print(
        f"Max pixel difference: {torch.abs(I_target - I_warped).max().item():.6f}")
    print(
        f"PSNR: {-10 * torch.log10(torch.mean((I_target - I_warped)**2)).item():.2f} dB")


if __name__ == "__main__":
    main()
