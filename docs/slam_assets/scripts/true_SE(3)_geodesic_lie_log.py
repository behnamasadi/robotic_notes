import torch

def quat_to_R(q, eps=1e-8):
    # q = [w, x, y, z]
    q = q / (q.norm(dim=-1, keepdim=True) + eps)
    w, x, y, z = q.unbind(-1)
    B = q.shape[0]
    R = torch.empty(B, 3, 3, device=q.device, dtype=q.dtype)
    R[:,0,0] = 1 - 2*(y*y + z*z); R[:,0,1] = 2*(x*y - z*w);   R[:,0,2] = 2*(x*z + y*w)
    R[:,1,0] = 2*(x*y + z*w);     R[:,1,1] = 1 - 2*(x*x + z*z); R[:,1,2] = 2*(y*z - x*w)
    R[:,2,0] = 2*(x*z - y*w);     R[:,2,1] = 2*(y*z + x*w);     R[:,2,2] = 1 - 2*(x*x + y*y)
    return R

def so3_log(R, eps=1e-8):
    # returns omega in R^3 such that ||omega|| = rotation angle
    # theta = acos((trace(R) - 1)/2)
    tr = R.diagonal(dim1=-2, dim2=-1).sum(-1)
    cos_theta = ((tr - 1) * 0.5).clamp(-1+1e-7, 1-1e-7)
    theta = torch.acos(cos_theta)
    # For small angles, use first-order approx: log(R) ~ 0.5*(R - R^T)^\vee
    small = theta < 1e-3
    skew = 0.5 * (R - R.transpose(-1, -2))
    omega = torch.stack([skew[:,2,1], skew[:,0,2], skew[:,1,0]], dim=-1)
    # For not-small, use exact: omega = theta * axis
    # axis = (1/(2 sin theta)) * vee(R - R^T)
    sin_theta = torch.sin(theta).clamp_min(1e-7)
    axis = omega / sin_theta.unsqueeze(-1)
    omega_exact = axis * theta.unsqueeze(-1)
    return torch.where(small.unsqueeze(-1), omega, omega_exact)

def se3_log(q_pred, t_pred, q_gt, t_gt, eps=1e-8):
    # Build relative transform ΔT = T_gt^{-1} T_pred
    R_pred = quat_to_R(q_pred)
    R_gt   = quat_to_R(q_gt)
    t_pred = t_pred
    t_gt   = t_gt

    R_rel = torch.matmul(R_gt.transpose(1,2), R_pred)
    t_rel = torch.matmul(R_gt.transpose(1,2), (t_pred - t_gt).unsqueeze(-1)).squeeze(-1)

    # Rotation log
    omega = so3_log(R_rel)                  # [B,3]
    theta = omega.norm(dim=-1, keepdim=True)

    # V^{-1} for translation log: v = V^{-1} t_rel
    # V = I + (1-c)/θ^2 [ω]x + (θ-s)/θ^3 [ω]x^2, with c=cosθ, s=sinθ
    # Use series for small θ.
    I = torch.eye(3, device=R_rel.device, dtype=R_rel.dtype).unsqueeze(0).expand_as(R_rel)
    def hat(v):
        O = torch.zeros(v.shape[0], 3, 3, device=v.device, dtype=v.dtype)
        O[:,0,1], O[:,0,2] = -v[:,2],  v[:,1]
        O[:,1,0], O[:,1,2] =  v[:,2], -v[:,0]
        O[:,2,0], O[:,2,1] = -v[:,1],  v[:,0]
        return O

    Omega = hat(omega)
    theta2 = (theta**2).clamp_min(1e-12)
    theta3 = theta2 * theta.clamp_min(1e-12)

    c = torch.cos(theta)
    s = torch.sin(theta)

    A = (1 - c) / theta2                  # ~ 0.5 - θ^2/24 + ...
    B = (theta - s) / theta3              # ~ 1/6 - θ^2/120 + ...

    # Series fallback for very small theta
    small = (theta.squeeze(-1) < 1e-3).unsqueeze(-1).unsqueeze(-1)
    A_series = 0.5 - theta2/24 + theta2*theta2/720
    B_series = (1.0/6.0) - theta2/120 + theta2*theta2/5040
    A = torch.where(small, A_series, A)
    B = torch.where(small, B_series, B)

    V = I + A*Omega + B*(Omega @ Omega)

    # v = V^{-1} t_rel  (solve linear system for stability)
    v = torch.linalg.solve(V, t_rel.unsqueeze(-1)).squeeze(-1)  # [B,3]

    # ξ = [ω, v]
    xi = torch.cat([omega, v], dim=-1)     # [B,6]
    return xi

def se3_loss_logmap(q_pred, t_pred, q_gt, t_gt, w_rot=1.0, w_trans=1.0):
    xi = se3_log(q_pred, t_pred, q_gt, t_gt)   # [B,6] = [ω(3), v(3)]
    omega = xi[:, :3]
    v     = xi[:, 3:]
    return (w_rot * omega.norm(dim=-1) + w_trans * v.norm(dim=-1)).mean()

