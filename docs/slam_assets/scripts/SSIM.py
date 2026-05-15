import sys
from pathlib import Path
import torch
import torch.nn as nn
import torch.nn.functional as F

# Add src directory to path for importing utils
src_dir = Path(__file__).resolve().parent.parent.parent
if str(src_dir) not in sys.path:
    sys.path.insert(0, str(src_dir))


# SSIM loss function


class SSIMLoss(nn.Module):
    def __init__(self, window_size=11, size_average=True):
        super(SSIMLoss, self).__init__()
        self.window_size = window_size
        self.size_average = size_average

    def gaussian_window(self, window_size, sigma):
        x = torch.arange(window_size).float()
        gauss = torch.exp(-((x - window_size//2)**2) / (2 * sigma**2))
        return gauss / gauss.sum()

    def create_window(self, channel):
        _1D_window = self.gaussian_window(self.window_size, 1.5).unsqueeze(1)
        _2D_window = _1D_window.mm(
            _1D_window.t()).float().unsqueeze(0).unsqueeze(0)
        window = _2D_window.expand(
            channel, 1, self.window_size, self.window_size).contiguous()
        return window

    def forward(self, img1, img2):
        channel = img1.size(1)
        window = self.create_window(channel).to(img1.device)

        mu1 = F.conv2d(img1, window, padding=self.window_size //
                       2, groups=channel)
        mu2 = F.conv2d(img2, window, padding=self.window_size //
                       2, groups=channel)

        mu1_sq, mu2_sq, mu1_mu2 = mu1.pow(2), mu2.pow(2), mu1 * mu2
        sigma1_sq = F.conv2d(
            img1 * img1, window, padding=self.window_size//2, groups=channel) - mu1_sq
        sigma2_sq = F.conv2d(
            img2 * img2, window, padding=self.window_size//2, groups=channel) - mu2_sq
        sigma12 = F.conv2d(
            img1 * img2, window, padding=self.window_size//2, groups=channel) - mu1_mu2

        C1, C2 = 0.01**2, 0.03**2
        ssim_map = ((2*mu1_mu2 + C1) * (2*sigma12 + C2)) / \
                   ((mu1_sq + mu2_sq + C1) * (sigma1_sq + sigma2_sq + C2))

        if self.size_average:
            return 1 - ssim_map.mean()  # as loss
        else:
            return 1 - ssim_map.mean(1).mean(1).mean(1)


def main():
    # Example usage
    loss_fn = SSIMLoss()
    img1 = torch.rand((1, 3, 128, 128))
    img2 = torch.rand((1, 3, 128, 128))

    loss = loss_fn(img1, img2)
    print("SSIM Loss:", loss.item())


def SIIMImage():
    from PIL import Image
    import torchvision.transforms as T

    # Use Path to get images relative to this script
    script_dir = Path(__file__).resolve().parent
    images_dir = script_dir.parent / "images/00"

    img1_path = images_dir / "000000.png"
    img2_path = images_dir / "000001.png"

    img1 = Image.open(img1_path).convert('RGB')
    img2 = Image.open(img2_path).convert('RGB')

    print(f"Loaded image 1: {img1_path}")
    print(f"Loaded image 2: {img2_path}")
    print(f"Image 1 size: {img1.size}")
    print(f"Image 2 size: {img2.size}")

    transform = T.Compose([
        # Converts to torch.FloatTensor and scales [0,255] → [0,1]
        T.ToTensor()
    ])

    # .unsqueeze(0) adds a batch dimension(batch size=1).

    img1_tensor = transform(img1).unsqueeze(0)  # [1, 3, H, W]
    img2_tensor = transform(img2).unsqueeze(0)

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    img1_tensor = img1_tensor.to(device)
    img2_tensor = img2_tensor.to(device)

    import matplotlib.pyplot as plt

    plt.subplot(1, 2, 1)
    plt.title("Image 1")
    plt.imshow(img1_tensor[0].permute(1, 2, 0).cpu())  # [C,H,W] → [H,W,C]

    plt.subplot(1, 2, 2)
    plt.title("Image 2")
    plt.imshow(img2_tensor[0].permute(1, 2, 0).cpu())
    plt.show()

    loss_fn = SSIMLoss().to(device)

    loss = loss_fn(img1_tensor, img2_tensor)
    print("SSIM Loss:", loss.item())


if __name__ == "__main__":
    main()

    SIIMImage()
