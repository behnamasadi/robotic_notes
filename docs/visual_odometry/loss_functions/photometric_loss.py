import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
import cv2
import matplotlib.pyplot as plt
from pathlib import Path


# ---------- SSIM module ----------
class SSIM(nn.Module):
    """
    Compute per-pixel SSIM between two RGB images.
    Output is (1 - SSIM) / 2, so 0 = perfect match, 1 = worst.
    """

    def __init__(self):
        super().__init__()
        self.pool = nn.AvgPool2d(3, 1)  # local 3×3 average
        self.C1 = 0.01 ** 2
        self.C2 = 0.03 ** 2

    def forward(self, x, y):
        mu_x = self.pool(x)
        mu_y = self.pool(y)
        sigma_x = self.pool(x * x) - mu_x ** 2
        sigma_y = self.pool(y * y) - mu_y ** 2
        sigma_xy = self.pool(x * y) - mu_x * mu_y

        ssim_n = (2 * mu_x * mu_y + self.C1) * (2 * sigma_xy + self.C2)
        ssim_d = (mu_x ** 2 + mu_y ** 2 + self.C1) * \
            (sigma_x + sigma_y + self.C2)
        ssim = ssim_n / ssim_d
        return torch.clamp((1 - ssim) / 2, 0, 1)  # lower = better


# ---------- Photometric loss ----------
def photometric_error(I_t, I_warp, alpha=0.85):
    """
    Combines SSIM and L1 difference.
    Both inputs [B,3,H,W] normalized to [0,1].
    """
    ssim = SSIM()(I_t, I_warp)
    l1 = (I_t - I_warp).abs().mean(1, keepdim=True)
    if ssim.shape[1] != 1:
        ssim = ssim.mean(1, keepdim=True)
    return alpha * ssim + (1 - alpha) * l1


# ---------- KITTI loader ----------
def load_kitti_pair(data_root, seq="00", idx_pair=(0, 1)):
    """
    Load two consecutive KITTI odometry frames from sequence folder.
    Example structure:
        data_root/sequences/00/image_2/000000.png
    """
    seq_path = Path(data_root) / "sequences" / seq / "image_2"
    img_paths = [seq_path / f"{i:06d}.png" for i in idx_pair]
    imgs = [cv2.cvtColor(cv2.imread(str(p)), cv2.COLOR_BGR2RGB)
            for p in img_paths]

    # resize to half (faster testing)
    imgs = [cv2.resize(im, (im.shape[1]//2, im.shape[0]//2)) for im in imgs]
    H, W = imgs[0].shape[:2]

    I_t = torch.from_numpy(imgs[0]).permute(
        2, 0, 1).unsqueeze(0).float() / 255.0
    I_s = torch.from_numpy(imgs[1]).permute(
        2, 0, 1).unsqueeze(0).float() / 255.0
    return I_t, I_s, (H, W)


# ---------- Main test ----------
def main():
    data_root = "/path/to/KITTI/odometry/"
    I_t, I_s, (H, W) = load_kitti_pair(data_root, seq="00", idx_pair=(0, 1))

    # 1️⃣ Compute SSIM and photometric errors directly
    ssim_module = SSIM()
    ssim_map = ssim_module(I_t, I_s)           # [B,3,H,W] → [B,3,H,W]
    photo_map = photometric_error(I_t, I_s)    # combined loss map

    ssim_mean = ssim_map.mean().item()
    photo_mean = photo_map.mean().item()

    print(f"SSIM mean:          {ssim_mean:.4f}")
    print(f"Photometric mean:   {photo_mean:.4f}")

    # 2️⃣ Visualize
    ssim_vis = ssim_map[0].mean(0).numpy()
    photo_vis = photo_map[0, 0].numpy()

    I_t_vis = I_t[0].permute(1, 2, 0).numpy()
    I_s_vis = I_s[0].permute(1, 2, 0).numpy()

    fig, axs = plt.subplots(1, 4, figsize=(14, 4))
    axs[0].imshow(I_t_vis)
    axs[0].set_title("Target (I_t)")
    axs[1].imshow(I_s_vis)
    axs[1].set_title("Source (I_s)")
    axs[2].imshow(ssim_vis, cmap="inferno")
    axs[2].set_title("SSIM map (0=good)")
    axs[3].imshow(photo_vis, cmap="inferno")
    axs[3].set_title("Photometric error (0=good)")
    for a in axs:
        a.axis("off")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
