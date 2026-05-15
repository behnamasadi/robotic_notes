import torch
import torch.nn as nn
from torchvision.models import resnet18, ResNet18_Weights
import torch.nn.functional as F


class ResNet18Encoder(nn.Module):
    def __init__(self, weights=ResNet18_Weights.IMAGENET1K_V1):
        super().__init__()
        resnet = resnet18(weights=weights)

        self.conv1 = resnet.conv1
        self.bn1 = resnet.bn1
        self.relu = resnet.relu
        self.maxpool = resnet.maxpool
        self.layer1 = resnet.layer1
        self.layer2 = resnet.layer2
        self.layer3 = resnet.layer3
        self.layer4 = resnet.layer4

        # record output channels of each feature
        self.out_channels = [64, 64, 128, 256, 512]

    def forward(self, x):
        x0 = self.conv1(x)     # [B, 64, H/2, W/2]
        x0 = self.bn1(x0)
        x0 = self.relu(x0)     # first skip
        x1 = self.maxpool(x0)  # [B, 64, H/4, W/4]
        x2 = self.layer1(x1)   # [B, 64, H/4, W/4]
        x3 = self.layer2(x2)   # [B, 128, H/8, W/8]
        x4 = self.layer3(x3)   # [B, 256, H/16, W/16]
        x5 = self.layer4(x4)   # [B, 512, H/32, W/32]
        return [x0, x2, x3, x4, x5]  # skip connections


class ConvBlock(nn.Module):
    """(Conv→BN→ReLU)×2 used for center or decoder fusion."""

    def __init__(self, in_ch, out_ch):
        super().__init__()
        self.block = nn.Sequential(
            nn.Conv2d(in_ch, out_ch, 3, padding=1, bias=False),
            nn.BatchNorm2d(out_ch),
            nn.ReLU(inplace=True),
            nn.Conv2d(out_ch, out_ch, 3, padding=1, bias=False),
            nn.BatchNorm2d(out_ch),
            nn.ReLU(inplace=True),
        )

    def forward(self, x):
        return self.block(x)


class DecoderBlock(nn.Module):
    def __init__(self, in_ch, skip_ch, out_ch, up_mode="deconv"):
        super().__init__()
        if up_mode == "deconv":
            self.up = nn.ConvTranspose2d(
                in_ch, out_ch, kernel_size=2, stride=2)
        elif up_mode == "bilinear":
            self.up = nn.Identity()
            self.up_out_ch = in_ch  # keep channels, reduce after concat in ConvBlock
        else:
            raise ValueError("up_mode must be 'deconv' or 'bilinear'")

        # If using bilinear upsampling, we first upsample then reduce channels via 1x1
        self.reduce = nn.Identity() if up_mode == "deconv" else nn.Conv2d(
            in_ch, out_ch, 1, bias=False)
        self.fuse = ConvBlock(out_ch + skip_ch, out_ch)

        self.up_mode = up_mode

    def forward(self, x, skip):
        if self.up_mode == "deconv":
            x = self.up(x)
        else:
            x = F.interpolate(
                x, size=skip.shape[2:], mode="bilinear", align_corners=False)
            x = self.reduce(x)

        if x.shape[2:] != skip.shape[2:]:
            x = F.interpolate(
                x, size=skip.shape[2:], mode="bilinear", align_corners=False)
        x = torch.cat([x, skip], dim=1)
        return self.fuse(x)


class ResNetUNet(nn.Module):
    def __init__(self, num_classes=1, encoder_weights=ResNet18_Weights.IMAGENET1K_V1, center_mult=1.0, up_mode="deconv"
                 ):
        """
        center_mult lets you widen/narrow the center block:
          center_out_ch = int(enc_chs[-1] * center_mult)
        """
        super().__init__()

        self.encoder = ResNet18Encoder(weights=encoder_weights)

        enc_chs = self.encoder.out_channels  # [64, 64, 128, 256, 512]
        c0, c2, c3, c4, c5 = enc_chs

        # Optional explicit bottleneck (center) on top of x5
        center_out = int(c5 * center_mult)
        self.center = ConvBlock(
            c5, center_out) if center_mult != 1.0 else nn.Identity()
        bottom_ch = center_out if center_mult != 1.0 else c5

        # Decoder: parameterized, no hardcoded 512
        self.dec4 = DecoderBlock(bottom_ch, c4, c4, up_mode=up_mode)  # 32→16
        self.dec3 = DecoderBlock(c4,       c3, c3, up_mode=up_mode)   # 16→8
        self.dec2 = DecoderBlock(c3,       c2, c2, up_mode=up_mode)   # 8→4
        self.dec1 = DecoderBlock(c2,       c0, c0, up_mode=up_mode)   # 4→2

        # For DepthNet usage, just add a sigmoid activation at the end to bound disparity in (0, 1):
        # In ResNetUNet we have this instead:
        # self.final = nn.Sequential(
        #     nn.Upsample(scale_factor=2, mode="bilinear",
        #                 align_corners=False),  # 2→1
        #     nn.Conv2d(c0, num_classes, kernel_size=1),
        # )

        self.final = nn.Sequential(
            nn.Upsample(scale_factor=2, mode="bilinear", align_corners=False),
            nn.Conv2d(c0, num_classes, kernel_size=1),
            nn.Sigmoid(),   # normalize disparity output
        )

    def forward(self, x):
        x0, x2, x3, x4, x5 = self.encoder(x)
        x5 = self.center(x5)        # explicit U-Net bottleneck (optional)
        d4 = self.dec4(x5, x4)
        d3 = self.dec3(d4, x3)
        d2 = self.dec2(d3, x2)
        d1 = self.dec1(d2, x0)
        return self.final(d1)


class DepthNet(nn.Module):
    def __init__(self, encoder_weights=ResNet18_Weights.IMAGENET1K_V1):
        super().__init__()
        encoder_weights = encoder_weights
        self.net = ResNetUNet(
            num_classes=1, encoder_weights=encoder_weights, up_mode="bilinear")

    def forward(self, x):
        disp = self.net(x)
        return {"disp_0": disp}


class ResNetUNetMultiScale(nn.Module):
    """
    U-Net-style depth network with ResNet18 encoder.
    Outputs disparity maps at multiple scales for stable self-supervised training.
    """

    def __init__(self, num_classes=1, encoder_weights=ResNet18_Weights.IMAGENET1K_V1,
                 center_mult=1.0, up_mode="bilinear"):
        super().__init__()
        self.encoder = ResNet18Encoder(weights=encoder_weights)
        enc_chs = self.encoder.out_channels  # [64, 64, 128, 256, 512]
        c0, c1, c2, c3, c4 = enc_chs

        # Optional bottleneck widening
        center_out = int(c4 * center_mult)
        self.center = ConvBlock(
            c4, center_out) if center_mult != 1.0 else nn.Identity()
        bottom_ch = center_out if center_mult != 1.0 else c4

        # Decoder hierarchy (32→16→8→4→2)
        self.dec3 = DecoderBlock(bottom_ch, c3, c3, up_mode=up_mode)
        self.dec2 = DecoderBlock(c3, c2, c2, up_mode=up_mode)
        self.dec1 = DecoderBlock(c2, c1, c1, up_mode=up_mode)
        self.dec0 = DecoderBlock(c1, c0, c0, up_mode=up_mode)

        # Multi-scale disparity heads (predict 1-channel disparity)
        self.disp3 = nn.Conv2d(c3, num_classes, kernel_size=3, padding=1)
        self.disp2 = nn.Conv2d(c2, num_classes, kernel_size=3, padding=1)
        self.disp1 = nn.Conv2d(c1, num_classes, kernel_size=3, padding=1)
        self.disp0 = nn.Conv2d(c0, num_classes, kernel_size=3, padding=1)

    def forward(self, x):
        # ----- Encoder -----
        x0, x1, x2, x3, x4 = self.encoder(x)
        x4 = self.center(x4)  # optional bottleneck

        # ----- Decoder -----
        d3 = self.dec3(x4, x3)  # stride 16 → 8
        d2 = self.dec2(d3, x2)  # stride 8  → 4
        d1 = self.dec1(d2, x1)  # stride 4  → 2
        d0 = self.dec0(d1, x0)  # stride 2  → 1

        # ----- Multi-scale disparities -----
        disp_3 = torch.sigmoid(self.disp3(d3))
        disp_2 = torch.sigmoid(self.disp2(d2))
        disp_1 = torch.sigmoid(self.disp1(d1))
        disp_0 = torch.sigmoid(self.disp0(d0))

        # Upsample all disparities to full input size (for loss computation)
        disp_3 = F.interpolate(
            disp_3, size=x.shape[2:], mode="bilinear", align_corners=False)
        disp_2 = F.interpolate(
            disp_2, size=x.shape[2:], mode="bilinear", align_corners=False)
        disp_1 = F.interpolate(
            disp_1, size=x.shape[2:], mode="bilinear", align_corners=False)
        disp_0 = F.interpolate(
            disp_0, size=x.shape[2:], mode="bilinear", align_corners=False)

        return {
            "disp_0": disp_0,  # full resolution
            "disp_1": disp_1,  # 1/2
            "disp_2": disp_2,  # 1/4
            "disp_3": disp_3,  # 1/8
        }


def disp_to_depth(disp, min_depth=0.1, max_depth=100.0):
    min_disp = 1.0 / max_depth
    max_disp = 1.0 / min_depth
    scaled_disp = min_disp + (max_disp - min_disp) * disp
    depth = 1.0 / scaled_disp
    return depth


if __name__ == "__main__":
    model = ResNetUNet(num_classes=10)
    x = torch.randn(1, 3, 512, 512)
    y = model(x)
    print(y.shape)  # → [1, num_classes, 224, 224]

    # depth_net = DepthNet(encoder_weights=ResNet18_Weights.IMAGENET1K_V1)
    # x = torch.randn(1, 3, 512, 512)
    # y = depth_net(x)
    # print(y)  # → [1, num_classes, 224, 224]

    depth_net_multiscale = ResNetUNetMultiScale(
        encoder_weights=ResNet18_Weights.IMAGENET1K_V1)
    x = torch.randn(1, 3, 512, 512)
    disp_outs = depth_net_multiscale(x)

    for scale, disp in disp_outs.items():
        print("scale", scale)
        print("disp", disp.shape)


# disp_outs = depth_net(I_t)
# loss_total = 0
# for scale, disp in disp_outs.items():
#     depth = disp_to_depth(disp)
#     # downscale I_t, I_s, K accordingly
#     # compute min-reprojection + smoothness per scale
#     loss_total += scale_weight * loss_scale
