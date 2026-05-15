# PoseNet (very small CNN)

#     Input: concat two RGB frames → 6 channels (or three frames → 9 channels).
#     Conv(7×7, s=2) → Conv(5×5, s=2) → Conv(3×3, s=2) × 3 → GAP → FC(6).
#     Last layer init near zero; multiply by 0.01 to keep poses small at start.

import torch
import torch.nn as nn
import torch.nn.functional as F
from torchvision.models import resnet18, ResNet18_Weights


class PoseNetSmall(nn.Module):
    """
    Tiny CNN PoseNet for self-supervised visual odometry.
    Input : [B, 6, H, W]  (pair of RGB frames concatenated along channels)
    Output: [B, 6]        (axis-angle [3] + translation [3])
    """

    def __init__(self, in_channels=6):
        super().__init__()
        # Simple CNN hierarchy
        self.conv1 = nn.Conv2d(
            in_channels, 16, kernel_size=7, stride=2, padding=3)
        self.conv2 = nn.Conv2d(16, 32, kernel_size=5, stride=2, padding=2)
        self.conv3 = nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1)
        self.conv4 = nn.Conv2d(64, 128, kernel_size=3, stride=2, padding=1)
        self.conv5 = nn.Conv2d(128, 256, kernel_size=3, stride=2, padding=1)

        self.relu = nn.ReLU(inplace=True)
        self.avgpool = nn.AdaptiveAvgPool2d(1)       # global feature
        self.pose_fc = nn.Conv2d(256, 6, kernel_size=1, stride=1, padding=0)

        # init small so poses start near zero
        nn.init.zeros_(self.pose_fc.weight)
        nn.init.zeros_(self.pose_fc.bias)

    def forward(self, x):
        x = self.relu(self.conv1(x))
        x = self.relu(self.conv2(x))
        x = self.relu(self.conv3(x))
        x = self.relu(self.conv4(x))
        x = self.relu(self.conv5(x))
        x = self.avgpool(x)          # [B,256,1,1]
        x = self.pose_fc(x)          # [B,6,1,1]
        x = x.view(-1, 6)
        return 0.01 * x              # scale for stability


class PoseNet3Frame(nn.Module):
    """
    3-frame PoseNet (as in Monodepth2)
    Input:  [B, 9, H, W]  → concatenated [I_{t-1}, I_t, I_{t+1}]
    Output: [B, 12]       → 2 relative SE(3) poses (t→t-1 and t→t+1)
    """

    def __init__(self, in_channels=9, num_frames_to_predict=2):
        super().__init__()
        self.num_frames = num_frames_to_predict

        self.conv1 = nn.Conv2d(in_channels, 16, 7, 2, 3)
        self.conv2 = nn.Conv2d(16, 32, 5, 2, 2)
        self.conv3 = nn.Conv2d(32, 64, 3, 2, 1)
        self.conv4 = nn.Conv2d(64, 128, 3, 2, 1)
        self.conv5 = nn.Conv2d(128, 256, 3, 2, 1)
        self.relu = nn.ReLU(inplace=True)

        self.avgpool = nn.AdaptiveAvgPool2d(1)
        self.pose_fc = nn.Conv2d(
            256, 6 * num_frames_to_predict, kernel_size=1, stride=1, padding=0)

        # Initialize small
        nn.init.zeros_(self.pose_fc.weight)
        nn.init.zeros_(self.pose_fc.bias)

    def forward(self, x):
        """
        Returns:
          poses: [B, num_frames, 6]
        """
        x = self.relu(self.conv1(x))
        x = self.relu(self.conv2(x))
        x = self.relu(self.conv3(x))
        x = self.relu(self.conv4(x))
        x = self.relu(self.conv5(x))

        x = self.avgpool(x)
        x = self.pose_fc(x)
        x = x.view(-1, self.num_frames, 6)
        return 0.01 * x


class PoseNetResNet18(nn.Module):
    """
    PoseNet with ResNet18 backbone.
    Input:  [B, 6, H, W]   (two RGB frames concatenated)
    Output: [B, 6]         (axis-angle + translation)
    """

    def __init__(self, pretrained=True):
        super().__init__()
        # Load pretrained ResNet18
        self.encoder = resnet18(
            weights=ResNet18_Weights.IMAGENET1K_V1 if pretrained else None)

        # Replace first conv layer to accept 6 channels (instead of 3)
        self.encoder.conv1 = nn.Conv2d(
            6, 64, kernel_size=7, stride=2, padding=3, bias=False
        )

        # Remove classification head
        self.encoder.fc = nn.Identity()

        # Small MLP head for pose regression
        self.fc1 = nn.Linear(512, 256)
        self.fc2 = nn.Linear(256, 6)

        self.relu = nn.ReLU(inplace=True)

        # Initialize last layer small
        nn.init.zeros_(self.fc2.weight)
        nn.init.zeros_(self.fc2.bias)

    def forward(self, x):
        feat = self.encoder(x)             # [B,512]
        feat = self.relu(self.fc1(feat))   # [B,256]
        pose = self.fc2(feat)              # [B,6]
        return 0.01 * pose                 # scale for stability


if __name__ == "__main__":
    # net = PoseNetSmall()
    # inp = torch.randn(4, 6, 192, 640)   # 4 pairs of frames
    # out = net(inp)
    # print("Output shape:", out.shape)
    # print("Sample pose vector:", out[0])

    # # -----------

    # model = PoseNet3Frame()
    # x = torch.randn(2, 9, 192, 640)   # batch of 2 triplets
    # out = model(x)
    # print("Output shape:", out.shape)
    # print("Pose for frame 0:", out[0])

    # # -----------

    model = PoseNetResNet18(pretrained=True)
    x = torch.randn(2, 6, 192, 640)
    out = model(x)
    print(out.shape)
    print(out[0])
