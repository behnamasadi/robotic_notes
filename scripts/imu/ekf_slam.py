import numpy as np
import math
from dataclasses import dataclass

G2O_PATH = r"/home/behnam/workspace/robotic_notes/data/slam/input_INTEL_g2o.g2o"

# ----------------------------- utilities -----------------------------


def wrap_pi(a: float) -> float:
    return (a + np.pi) % (2*np.pi) - np.pi


def rot2(th: float) -> np.ndarray:
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s],
                     [s,  c]], dtype=float)


def parse_g2o_se2(path: str):
    vertices = {}        # id -> [x,y,theta]
    edges_se2 = []       # (i,j, dx,dy,dth, Omega(3x3))
    landmarks = {}       # id -> [x,y]  (if present)
    # (pose_id, lm_id, bearing, range, Omega(2x2)) (if present)
    edges_se2_xy = []

    with open(path, "r") as f:
        for line in f:
            if not line.strip():
                continue
            parts = line.split()
            tag = parts[0]

            if tag == "VERTEX_SE2":
                _, idx, x, y, th = parts
                vertices[int(idx)] = np.array(
                    [float(x), float(y), float(th)], dtype=float)

            elif tag == "EDGE_SE2":
                i = int(parts[1])
                j = int(parts[2])
                dx = float(parts[3])
                dy = float(parts[4])
                dth = float(parts[5])
                I11 = float(parts[6])
                I12 = float(parts[7])
                I13 = float(parts[8])
                I22 = float(parts[9])
                I23 = float(parts[10])
                I33 = float(parts[11])
                Omega = np.array([[I11, I12, I13],
                                  [I12, I22, I23],
                                  [I13, I23, I33]], dtype=float)
                edges_se2.append((i, j, dx, dy, dth, Omega))

            elif tag == "VERTEX_XY":
                _, idx, x, y = parts
                landmarks[int(idx)] = np.array(
                    [float(x), float(y)], dtype=float)

            elif tag == "EDGE_SE2_XY":
                pose_id = int(parts[1])
                lm_id = int(parts[2])
                bearing = float(parts[3])
                r = float(parts[4])
                I11 = float(parts[5])
                I12 = float(parts[6])
                I22 = float(parts[7])
                Omega = np.array([[I11, I12],
                                  [I12, I22]], dtype=float)
                edges_se2_xy.append((pose_id, lm_id, bearing, r, Omega))

    return vertices, edges_se2, landmarks, edges_se2_xy

# ------------------------ EKF-SLAM model -----------------------------


@dataclass
class SlamParams:
    # measurement noise for synthetic observations (if no EDGE_SE2_XY exists)
    sigma_bearing: float = 0.05  # rad
    sigma_range: float = 0.10    # m

    # gating threshold for data association (NIS gate)
    gate_chi2_2dof: float = 9.21  # ~99% for 2 DoF

    # synthetic sensor range (only used if generating observations)
    min_range: float = 0.5
    max_range: float = 10.0


def h_bearing_range(xr: np.ndarray, lm: np.ndarray) -> np.ndarray:
    """
    xr: [x,y,theta]
    lm: [lx,ly]
    returns z=[bearing, range]
    """
    dx = lm[0] - xr[0]
    dy = lm[1] - xr[1]
    q = dx*dx + dy*dy
    r = math.sqrt(q)
    b = wrap_pi(math.atan2(dy, dx) - xr[2])
    return np.array([b, r], dtype=float)


def H_bearing_range(x: np.ndarray, lm_index: int) -> np.ndarray:
    """
    Measurement Jacobian wrt full state x.
    lm_index points to landmark x coordinate in state vector.
    """
    xr = x[0:3]
    lm = x[lm_index:lm_index+2]

    dx = lm[0] - xr[0]
    dy = lm[1] - xr[1]
    q = dx*dx + dy*dy
    r = math.sqrt(q)

    n = len(x)
    H = np.zeros((2, n), dtype=float)

    if q < 1e-12:
        return H

    # dbearing/d(xr,yr,th)
    H[0, 0] = dy / q
    H[0, 1] = -dx / q
    H[0, 2] = -1.0

    # drange/d(xr,yr)
    H[1, 0] = -dx / r
    H[1, 1] = -dy / r
    H[1, 2] = 0.0

    # dbearing/d(lx,ly)
    H[0, lm_index] = -dy / q
    H[0, lm_index + 1] = dx / q

    # drange/d(lx,ly)
    H[1, lm_index] = dx / r
    H[1, lm_index + 1] = dy / r

    return H


def predict_se2(x: np.ndarray, P: np.ndarray, u: np.ndarray, Sigma_u: np.ndarray):
    """
    EKF-SLAM prediction:
    - robot pose updates using odometry in robot frame (dx,dy,dth)
    - landmarks stay unchanged
    """
    n = len(x)
    dx, dy, dth = u
    th = x[2]
    c, s = math.cos(th), math.sin(th)

    x_pred = x.copy()
    x_pred[0] = x[0] + c*dx - s*dy
    x_pred[1] = x[1] + s*dx + c*dy
    x_pred[2] = wrap_pi(x[2] + dth)

    # F = d f / d x  (full state)
    F = np.eye(n, dtype=float)
    F[0, 2] = -s*dx - c*dy
    F[1, 2] = c*dx - s*dy

    # G = d f / d u  affects only robot part
    G = np.zeros((n, 3), dtype=float)
    G[0, 0] = c
    G[0, 1] = -s
    G[1, 0] = s
    G[1, 1] = c
    G[2, 2] = 1.0

    Q = G @ Sigma_u @ G.T
    P_pred = F @ P @ F.T + Q
    return x_pred, P_pred


def ekf_update(x: np.ndarray, P: np.ndarray, z: np.ndarray, lm_index: int, R: np.ndarray):
    """
    Standard EKF update with bearing-range measurement to an existing landmark.
    """
    xr = x[0:3]
    lm = x[lm_index:lm_index+2]

    z_hat = h_bearing_range(xr, lm)
    y = z - z_hat
    y[0] = wrap_pi(y[0])

    H = H_bearing_range(x, lm_index)
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)

    x_new = x + K @ y
    x_new[2] = wrap_pi(x_new[2])

    P_new = (np.eye(len(x)) - K @ H) @ P
    return x_new, P_new, y, S


def augment_state_with_landmark(x: np.ndarray, P: np.ndarray, z: np.ndarray, R: np.ndarray):
    """
    Proper landmark initialization:
    Given robot pose xr and measurement z=[bearing, range],
    create landmark in world frame and properly augment covariance including cross terms.

    landmark position:
      l = [x + r cos(th+b), y + r sin(th+b)]

    Jacobians:
      J_xr = d l / d [x,y,th]
      J_z  = d l / d [b,r]

    Cov augmentation:
      P_ll = J_xr P_rr J_xr^T + J_z R J_z^T
      P_xl = P_xr * J_xr^T  (cross cov between all existing states and new landmark)
    """
    xr = x[0:3]
    x0, y0, th = xr
    b, r = z
    a = th + b
    ca, sa = math.cos(a), math.sin(a)

    l = np.array([x0 + r * ca,
                  y0 + r * sa], dtype=float)

    # J_xr: 2x3
    J_xr = np.array([
        [1.0, 0.0, -r * sa],
        [0.0, 1.0,  r * ca],
    ], dtype=float)

    # J_z: 2x2  (bearing, range)
    J_z = np.array([
        [-r * sa,  ca],
        [r * ca,  sa],
    ], dtype=float)

    n_old = len(x)
    x_new = np.concatenate([x, l])

    P_new = np.zeros((n_old + 2, n_old + 2), dtype=float)
    P_new[:n_old, :n_old] = P

    # Robot covariance block (first 3x3) is part of P
    P_rr = P[0:3, 0:3]

    # Landmark covariance
    P_ll = J_xr @ P_rr @ J_xr.T + J_z @ R @ J_z.T
    P_new[n_old:n_old+2, n_old:n_old+2] = P_ll

    # Cross covariance between existing states and new landmark:
    # For any existing state x_i, cov(x_i, l) = cov(x_i, xr) * J_xr^T
    P_xr = P[:, 0:3]                    # (n_old x 3)
    P_xl = P_xr @ J_xr.T                # (n_old x 2)

    P_new[:n_old, n_old:n_old+2] = P_xl
    P_new[n_old:n_old+2, :n_old] = P_xl.T

    return x_new, P_new


def nis(y: np.ndarray, S: np.ndarray) -> float:
    return float(y.T @ np.linalg.inv(S) @ y)

# ------------------------ main EKF-SLAM loop -------------------------


def main():
    params = SlamParams()

    vertices, edges_se2, lm_vertices, edges_se2_xy = parse_g2o_se2(G2O_PATH)

    # odometry chain
    chain = [e for e in edges_se2 if e[1] == e[0] + 1]
    chain.sort(key=lambda t: t[0])
    if not chain:
        raise RuntimeError("No consecutive (i->i+1) edges found.")

    # initial robot pose
    start_id = chain[0][0]
    xr0 = vertices.get(start_id, np.zeros(3, dtype=float))

    # EKF-SLAM state: [xr, yr, thr, l1x,l1y,l2x,l2y,...]
    x = xr0.copy()
    P = np.diag([0.5, 0.5, 0.2])**2  # initial robot uncertainty

    # landmark bookkeeping: lm_id -> index in state
    lm_index = {}

    # Prepare measurement noise for synthetic mode
    R_synth = np.diag([params.sigma_bearing, params.sigma_range])**2

    # If dataset has no real landmark obs, create a fixed set of synthetic landmarks in the world.
    use_real_obs = (len(edges_se2_xy) > 0)

    if not use_real_obs:
        # Build a set of fixed landmarks from a sparse subset of vertex positions (world-fixed).
        # This is only to emulate a sensor; EKF-SLAM logic remains correct.
        ids_sorted = sorted(vertices.keys())
        stride = max(1, len(ids_sorted) // 25)  # ~25 landmarks
        synthetic_landmarks = {}
        for k, vid in enumerate(ids_sorted[::stride]):
            if vid == start_id:
                continue
            synthetic_landmarks[100000 + k] = vertices[vid][:2].copy()
        print(
            f"[INFO] No EDGE_SE2_XY found. Using {len(synthetic_landmarks)} synthetic fixed landmarks.")
    else:
        synthetic_landmarks = {}

    xs = [x[:3].copy()]
    node_ids = [start_id]

    # index observations by pose_id for speed
    obs_by_pose = {}
    if use_real_obs:
        for (pid, lid, b, r, Omega) in edges_se2_xy:
            obs_by_pose.setdefault(pid, []).append(
                (lid, np.array([b, r], dtype=float), np.linalg.inv(Omega)))

    for (i, j, dx, dy, dth, Omega_u) in chain:
        # ---- predict ----
        u = np.array([dx, dy, dth], dtype=float)
        Sigma_u = np.linalg.inv(Omega_u)
        x, P = predict_se2(x, P, u, Sigma_u)

        # ---- collect observations at pose j ----
        if use_real_obs:
            obs_list = obs_by_pose.get(j, [])
            for (lid, z, R) in obs_list:
                if lid not in lm_index:
                    x, P = augment_state_with_landmark(x, P, z, R)
                    lm_index[lid] = len(x) - 2
                else:
                    idx = lm_index[lid]
                    x, P, innov, S = ekf_update(x, P, z, idx, R)

        else:
            # generate synthetic measurements from current pose to fixed landmarks
            xr = x[:3]
            for lid, lm_true in synthetic_landmarks.items():
                z_true = h_bearing_range(xr, lm_true)
                if not (params.min_range < z_true[1] < params.max_range):
                    continue

                # add noise
                z = z_true.copy()
                z[0] = wrap_pi(
                    z[0] + np.random.normal(0.0, params.sigma_bearing))
                z[1] = z[1] + np.random.normal(0.0, params.sigma_range)

                # initialize or update with gating
                if lid not in lm_index:
                    x, P = augment_state_with_landmark(x, P, z, R_synth)
                    lm_index[lid] = len(x) - 2
                else:
                    idx = lm_index[lid]
                    x_tmp, P_tmp, innov, S = ekf_update(x, P, z, idx, R_synth)
                    if nis(innov, S) < params.gate_chi2_2dof:
                        x, P = x_tmp, P_tmp
                    # else reject as outlier

        xs.append(x[:3].copy())
        node_ids.append(j)

    xs = np.array(xs)

    print("\n=== Proper EKF-SLAM finished ===")
    print(f"Robot poses: {len(xs)}")
    print(f"Landmarks in state: {len(lm_index)}  (state dim = {len(x)})")
    print(f"Final robot pose: {xs[-1]}")
    print(f"Final robot cov diag: {np.diag(P[:3, :3])}")

    # Plot
    try:
        import matplotlib.pyplot as plt
        plt.figure(figsize=(10, 7))
        plt.plot(xs[:, 0], xs[:, 1], label="EKF-SLAM trajectory")
        # plot landmark estimates
        if lm_index:
            L = np.array([x[idx:idx+2] for idx in lm_index.values()])
            plt.scatter(L[:, 0], L[:, 1], marker="x",
                        label="Landmark estimates")
        plt.axis("equal")
        plt.grid(True, alpha=0.3)
        plt.title("Proper EKF-SLAM (SE2 + point landmarks)")
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.legend()
        plt.show()
    except Exception as ex:
        print("Plot skipped:", ex)


if __name__ == "__main__":
    main()
