import torch
import torch.nn.functional as F


def gradient_x(img):
    """Compute gradient along x (width) axis."""
    return img[:, :, :, :-1] - img[:, :, :, 1:]


def gradient_y(img):
    """Compute gradient along y (height) axis."""
    return img[:, :, :-1, :] - img[:, :, 1:, :]


def smoothness_loss(disp, image):
    """
    Edge-aware smoothness loss for disparity map.
    disp:  [B, 1, H, W]
    image: [B, 3, H, W] (RGB normalized to [0,1])
    """
    # Normalize disparity: make mean=1 to avoid scale sensitivity
    mean_disp = disp.mean(dim=[2, 3], keepdim=True)
    disp_norm = disp / (mean_disp + 1e-7)

    # Compute disparity gradients
    disp_grad_x = gradient_x(disp_norm)
    disp_grad_y = gradient_y(disp_norm)

    # Compute image gradients (edge strength)
    img_grad_x = gradient_x(image)
    img_grad_y = gradient_y(image)

    # Weight disparity smoothness by image edges (stronger edges → less smoothing)
    weight_x = torch.exp(-torch.mean(torch.abs(img_grad_x),
                         dim=1, keepdim=True))
    weight_y = torch.exp(-torch.mean(torch.abs(img_grad_y),
                         dim=1, keepdim=True))

    smoothness_x = torch.abs(disp_grad_x) * weight_x
    smoothness_y = torch.abs(disp_grad_y) * weight_y

    # Take mean over all pixels and directions
    loss = smoothness_x.mean() + smoothness_y.mean()
    return loss


if __name__ == "__main__":
    B, C, H, W = 1, 3, 4, 4
    disp = torch.rand(B, 1, H, W)
    img = torch.rand(B, 3, H, W)
    loss = smoothness_loss(disp, img)
    print("Smoothness loss:", loss.item())


# Multi-scale setup
# If you have multiple disparity maps:

# for scale, disp in disp_outs.items():
#     smooth += weight[scale] * smoothness_loss(disp, I_t)
