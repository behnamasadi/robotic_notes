import torch

def geodesic_loss_fn(q_pred, q_gt):
    # Normalize
    q_pred = q_pred / q_pred.norm(dim=-1, keepdim=True)
    q_gt = q_gt / q_gt.norm(dim=-1, keepdim=True)
    
    # Dot product (cosine of half-angle)
    dot = torch.sum(q_pred * q_gt, dim=-1).abs()
    dot = torch.clamp(dot, -1.0, 1.0)  # numerical stability
    
    return 2 * torch.acos(dot)  # rotation angle in radians

# Example quaternions (already normalized)
q_gt = torch.tensor([0.7071, 0, 0, 0.7071])
q_pred = torch.tensor([0.8660, 0, 0, 0.5])
geodesic_loss=geodesic_loss_fn(q_pred, q_gt)

print("Geodesic loss (radians):", geodesic_loss.item())
print("Geodesic loss (degrees):", geodesic_loss.item() * 180.0 / 3.14159)
