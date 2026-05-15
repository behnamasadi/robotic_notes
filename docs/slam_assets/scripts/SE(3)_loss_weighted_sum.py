import torch

def quat_geodesic_angle(q_pred, q_gt, eps=1e-8):
    q_pred = q_pred / (q_pred.norm(dim=-1, keepdim=True) + eps)
    q_gt   = q_gt   / (q_gt.norm(dim=-1, keepdim=True) + eps)
    dot = (q_pred * q_gt).sum(dim=-1).abs().clamp(-1.0, 1.0)  # handle double cover + stability
    return 2.0 * torch.acos(dot)  # radians

def huber(x, delta=0.05):
    absx = x.abs()
    quad = 0.5 * (absx**2) / delta
    lin  = absx - 0.5 * delta
    return torch.where(absx <= delta, quad, lin)

def se3_loss_weighted(q_pred, t_pred, q_gt, t_gt, lambda_R=1.0, lambda_t=5.0, robust=True):
    ang = quat_geodesic_angle(q_pred, q_gt)            # [B]
    trans_err = (t_pred - t_gt).norm(dim=-1)           # [B]
    if robust:
        trans_term = huber(trans_err).mean()
    else:
        trans_term = trans_err.mean()
    rot_term = ang.mean()
    return lambda_R * rot_term + lambda_t * trans_term
