"""Small shared helpers for the Lie-series exercise notebooks (numpy only).

Notation is that of Sola, Deray & Atchuthan, "A micro Lie theory for state estimation in
robotics" (arXiv 1812.01537, ../../../pdf/A_micro_Lie_theory_for_state_estimation_in_robotics.pdf):
right-plus by default, and a pose tangent vector is tau = [rho; theta], translation first.
Equation numbers in the comments are the paper's.

    from lie_tools import SO2, SO3, SE2, SE3, skew, vee, expm, load_euroc

The exercises ask you to write most of these yourself first; the solutions use them to check.
"""
import os
import re
from pathlib import Path

import numpy as np

# ------------------------------------------------------------------ basics
def skew(v):
    """[v]x: the 3x3 skew-symmetric matrix with [v]x u = v x u (paper (9))."""
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0.0]])


def vee(S):
    """Inverse of skew: the three numbers out of a skew-symmetric matrix."""
    return np.array([S[2, 1], S[0, 2], S[1, 0]])


def expm(A, terms=30):
    """Matrix exponential by scaling and squaring a Taylor series. A reference that does not
    use any closed form, to check closed forms against."""
    nrm = np.linalg.norm(A, 2)
    n = 0 if nrm < 0.5 else int(np.ceil(np.log2(nrm))) + 2
    B = A / 2 ** n
    E, term = np.eye(len(A)), np.eye(len(A))
    for k in range(1, terms):
        term = term @ B / k
        E = E + term
    for _ in range(n):
        E = E @ E
    return E


ONEX = np.array([[0, -1.0], [1, 0]])   # [1]x of the paper, the generator of so(2)


def Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def wrap(a):
    """Angle to (-pi, pi]."""
    return np.pi - np.mod(np.pi - np.asarray(a), 2 * np.pi)


# ------------------------------------------------------------------ SO(2), App. A
class SO2:
    @staticmethod
    def Exp(th):                      # (116)
        c, s = np.cos(th), np.sin(th)
        return np.array([[c, -s], [s, c]])

    @staticmethod
    def Log(R):                       # (117)
        return float(np.arctan2(R[1, 0], R[0, 0]))


# ------------------------------------------------------------------ SO(3), App. B
class SO3:
    @staticmethod
    def Exp(th):                      # (134), Rodrigues
        th = np.asarray(th, float)
        a = np.linalg.norm(th)
        if a < 1e-10:
            return np.eye(3) + skew(th)
        K = skew(th / a)
        return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K

    @staticmethod
    def Log(R):                       # (135), with atan2 for the angle and the small-angle limit
        w = vee(R - R.T) / 2
        s, c = np.linalg.norm(w), (np.trace(R) - 1) / 2
        if s < 1e-10:
            return w
        return np.arctan2(s, c) * w / s

    @staticmethod
    def Jl(th):                       # (145); also V of SE(3), (174)
        th = np.asarray(th, float)
        a = np.linalg.norm(th)
        S = skew(th)
        if a < 1e-6:
            return np.eye(3) + 0.5 * S + S @ S / 6
        return np.eye(3) + (1 - np.cos(a)) / a ** 2 * S + (a - np.sin(a)) / a ** 3 * S @ S

    @staticmethod
    def Jr(th):                       # (143)
        return SO3.Jl(-np.asarray(th, float))

    @staticmethod
    def plus(R, th):                  # (25) right-plus: increment in the local (body) frame
        return R @ SO3.Exp(th)

    @staticmethod
    def minus(R2, R1):                # (26) right-minus
        return SO3.Log(R1.T @ R2)

    @staticmethod
    def lplus(th, R):                 # (27) left-plus: increment in the global (world) frame
        return SO3.Exp(th) @ R

    @staticmethod
    def lminus(R2, R1):               # (28) left-minus
        return SO3.Log(R2 @ R1.T)


def angle_deg(Ra, Rb):
    """Angle of the single turn that takes Ra to Rb, in degrees."""
    return float(np.degrees(np.linalg.norm(SO3.Log(Ra.T @ Rb))))


# ------------------------------------------------------------------ SE(2), App. C; tau = [rho; theta]
class SE2:
    @staticmethod
    def V(a):                         # (158)
        if abs(a) < 1e-10:
            return np.eye(2) + a / 2 * ONEX
        return np.sin(a) / a * np.eye(2) + (1 - np.cos(a)) / a * ONEX

    @staticmethod
    def hat(tau):                     # (153)
        H = np.zeros((3, 3))
        H[:2, :2] = tau[2] * ONEX
        H[:2, 2] = tau[:2]
        return H

    @staticmethod
    def Exp(tau):                     # (156)
        M = np.eye(3)
        M[:2, :2] = SO2.Exp(tau[2])
        M[:2, 2] = SE2.V(tau[2]) @ np.asarray(tau[:2], float)
        return M

    @staticmethod
    def Log(M):                       # (157)
        a = SO2.Log(M[:2, :2])
        return np.r_[np.linalg.solve(SE2.V(a), M[:2, 2]), a]

    @staticmethod
    def make(x, y, th):
        M = np.eye(3)
        M[:2, :2], M[:2, 2] = SO2.Exp(th), (x, y)
        return M


# ------------------------------------------------------------------ SE(3), App. D; tau = [rho; theta]
class SE3:
    V = staticmethod(SO3.Jl)          # (174): "matches (145) exactly"

    @staticmethod
    def hat(tau):                     # (169)
        H = np.zeros((4, 4))
        H[:3, :3] = skew(tau[3:])
        H[:3, 3] = tau[:3]
        return H

    @staticmethod
    def Exp(tau):                     # (172)
        tau = np.asarray(tau, float)
        M = np.eye(4)
        M[:3, :3] = SO3.Exp(tau[3:])
        M[:3, 3] = SE3.V(tau[3:]) @ tau[:3]
        return M

    @staticmethod
    def Log(M):                       # (173)
        th = SO3.Log(M[:3, :3])
        return np.r_[np.linalg.solve(SE3.V(th), M[:3, 3]), th]

    @staticmethod
    def make(R, t):
        M = np.eye(4)
        M[:3, :3], M[:3, 3] = R, t
        return M

    @staticmethod
    def inv(M):                       # [R^T, -R^T t]
        R, t = M[:3, :3], M[:3, 3]
        return SE3.make(R.T, -R.T @ t)


# ------------------------------------------------------------------ quaternions, Hamilton, q = [w, x, y, z]
def qmul(p, q):
    """Hamilton product p (x) q."""
    return np.r_[p[0] * q[0] - p[1:] @ q[1:], p[0] * q[1:] + q[0] * p[1:] + np.cross(p[1:], q[1:])]


def qconj(q):
    return np.r_[q[0], -np.asarray(q[1:])]


def qExp(th):                         # (132): cos(theta/2) + u sin(theta/2)
    th = np.asarray(th, float)
    a = np.linalg.norm(th)
    if a < 1e-12:
        return np.r_[1.0, th / 2]
    return np.r_[np.cos(a / 2), np.sin(a / 2) * th / a]


def qLog(q):                          # (133), with the paper's fix: make w >= 0 first
    q = np.asarray(q, float)
    if q[0] < 0:
        q = -q
    n = np.linalg.norm(q[1:])
    if n < 1e-12:
        return 2 * q[1:]
    return 2 * q[1:] * np.arctan2(n, q[0]) / n


def quat_to_R(q):
    """R from [w, x, y, z], (138); one quaternion or one per row."""
    qw, x, y, z = np.atleast_2d(q).T
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - qw * z), 2 * (x * z + qw * y)],
        [2 * (x * y + qw * z), 1 - 2 * (x * x + z * z), 2 * (y * z - qw * x)],
        [2 * (x * z - qw * y), 2 * (y * z + qw * x), 1 - 2 * (x * x + y * y)],
    ]).transpose(2, 0, 1)
    return R[0] if np.ndim(q) == 1 else R


# ------------------------------------------------------------------ EuRoC MH_01
DEFAULT_MAV0 = "/media/behnam/Extreme_SSD1/datasets/machine_hall/MH_01_easy/MH_01_easy/mav0"
DOWNLOAD = ("https://ethz-asl.github.io/datasets/euroc-mav/ (files on the ETH Research "
            "Collection, https://doi.org/10.3929/ethz-b-000690084)")


def mav0_path():
    """The mav0 folder of EuRoC MH_01_easy. Set EUROC_MAV0 to point at your copy."""
    p = Path(os.environ.get("EUROC_MAV0", DEFAULT_MAV0))
    if not (p / "imu0" / "data.csv").exists():
        raise FileNotFoundError(
            f"EuRoC MH_01_easy not found at {p}.\nDownload MH_01_easy (the ASL/zip "
            f"format) from {DOWNLOAD}, unzip it, and set EUROC_MAV0 to the 'mav0' folder inside.")
    return p


def load_euroc(mav0=None):
    """IMU (200 Hz) and ground truth of MH_01, on one clock: t = 0 at the first IMU sample.

    Loaded the way the IMU-series exercises load it. Returns a dict with
      t, w, a          IMU time [s], gyro [rad/s], accelerometer [m/s^2], body frame
      gt_t, P, V, Q    ground-truth time, position, velocity, quaternion [w, x, y, z] (normalised)
      R                R_WB at each gt_t: body (IMU) axes written in the world frame, z up
      BG               the ground truth's gyro bias; k the IMU sample at each gt_t
      at(sec)          index into gt_t of the sample at time sec
    The file stores the quaternions to six decimals; Q_file keeps them as stored.
    """
    mav0 = Path(mav0) if mav0 else mav0_path()
    imu_csv = mav0 / "imu0" / "data.csv"
    gt_csv = mav0 / "state_groundtruth_estimate0" / "data.csv"
    # timestamps as integers: nanoseconds since 1970 do not fit in a float
    stamp = np.loadtxt(imu_csv, delimiter=",", comments="#", usecols=0, dtype=np.int64)
    imu = np.loadtxt(imu_csv, delimiter=",", comments="#")
    gt_stamp = np.loadtxt(gt_csv, delimiter=",", comments="#", usecols=0, dtype=np.int64)
    gt = np.loadtxt(gt_csv, delimiter=",", comments="#")
    t = (stamp - stamp[0]) * 1e-9
    gt_t = (gt_stamp - stamp[0]) * 1e-9
    Q_file = gt[:, 4:8]
    Q = Q_file / np.linalg.norm(Q_file, axis=1, keepdims=True)
    return dict(t=t, w=imu[:, 1:4], a=imu[:, 4:7], gt_t=gt_t, P=gt[:, 1:4], Q=Q, Q_file=Q_file,
                V=gt[:, 8:11], BG=gt[:, 11:14], R=quat_to_R(Q), k=np.searchsorted(t, gt_t),
                at=lambda sec: int(np.searchsorted(gt_t, sec)))


def load_T_BS(mav0=None, cam="cam0"):
    """The camera-to-body extrinsic T_BS from <cam>/sensor.yaml (4x4), as stored."""
    mav0 = Path(mav0) if mav0 else mav0_path()
    txt = (mav0 / cam / "sensor.yaml").read_text()
    block = re.search(r"T_BS:.*?data:\s*\[(.*?)\]", txt, re.S).group(1)
    return np.array([float(v) for v in block.replace("\n", " ").split(",")]).reshape(4, 4)
