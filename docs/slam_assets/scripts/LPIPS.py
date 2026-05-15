#Install:
# pip install lpips

import torch
import lpips  # Learned Perceptual Image Patch Similarity

# Initialize with pretrained network (e.g., VGG)
loss_fn = lpips.LPIPS(net='vgg')

# Example inputs
img1 = torch.rand(1, 3, 256, 256) * 2 - 1  # range [-1,1]
img2 = torch.rand(1, 3, 256, 256) * 2 - 1

# Compute LPIPS score
d = loss_fn(img1, img2)
print("LPIPS distance:", d.item())



# Inputs must be normalized to **\[-1, 1]**.
# Smaller LPIPS = more perceptually similar.

