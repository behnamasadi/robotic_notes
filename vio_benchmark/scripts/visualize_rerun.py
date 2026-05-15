#!/usr/bin/env python3
"""visualize_rerun.py — Rerun visualization for a VIO comparison run.

Loads a ROS 2 mcap recording (containing estimator output + ground truth)
and logs it to a rerun viewer / .rrd file. Lets you scrub through time and
see the estimated trajectory next to the ground-truth trajectory, plus
the running drift error, plus any IMU stream in the bag.

Usage:
    # Visualize an OpenVINS run against Leica GT
    scripts/visualize_rerun.py runs/euroc_mh01_ov_solo \\
        --estimator-topic /ov_msckf/odomimu \\
        --gt-topic /leica/position \\
        --label OpenVINS \\
        [--output euroc_mh01_ov.rrd]      # if omitted, opens live viewer

    # Compare two estimators in one scene
    scripts/visualize_rerun.py runs/euroc_mh01_ov_solo runs/euroc_mh01_vins_solo \\
        --estimator-topic /ov_msckf/odomimu /odometry \\
        --label OpenVINS VINS-Fusion \\
        --gt-topic /leica/position

Output:
  - 3D trajectories overlaid in the same world
  - Estimator vs GT side-by-side
  - Running APE (distance to nearest GT point) as a time series
  - IMU accel components as time series
"""

import argparse
from pathlib import Path

import numpy as np
import rerun as rr
from rosbags.highlevel import AnyReader


def load_bag(bag_path: Path, est_topic: str, gt_topic: str, imu_topic: str | None):
    """Extract estimator poses, GT positions, and (optionally) IMU samples."""
    est, gt, imu = [], [], []
    with AnyReader([bag_path]) as reader:
        for conn, t, raw in reader.messages():
            t_s = t * 1e-9
            if conn.topic == est_topic:
                m = reader.deserialize(raw, conn.msgtype)
                if hasattr(m, "pose"):
                    p = m.pose.pose.position
                    q = m.pose.pose.orientation
                    est.append((t_s, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
            elif conn.topic == gt_topic:
                m = reader.deserialize(raw, conn.msgtype)
                if hasattr(m, "point"):
                    gt.append((t_s, m.point.x, m.point.y, m.point.z))
                elif hasattr(m, "pose"):
                    p = m.pose.pose.position
                    gt.append((t_s, p.x, p.y, p.z))
                elif hasattr(m, "transform"):
                    tr = m.transform.translation
                    gt.append((t_s, tr.x, tr.y, tr.z))
            elif imu_topic and conn.topic == imu_topic:
                m = reader.deserialize(raw, conn.msgtype)
                a = m.linear_acceleration
                imu.append((t_s, a.x, a.y, a.z))
    return np.array(est), np.array(gt), np.array(imu)


def nearest_gt(t_est, gt_t, gt_xyz):
    """For each estimator time, return the index of the nearest GT sample."""
    idx = np.searchsorted(gt_t, t_est).clip(0, len(gt_t) - 1)
    # Compare both neighbors
    prev = (idx - 1).clip(0)
    left_dist = np.abs(gt_t[prev] - t_est)
    right_dist = np.abs(gt_t[idx] - t_est)
    return np.where(left_dist < right_dist, prev, idx)


def umeyama(src, tgt, with_scale=False):
    mu_s, mu_t = src.mean(0), tgt.mean(0)
    sc, tc = src - mu_s, tgt - mu_t
    H = sc.T @ tc / len(src)
    U, D, Vt = np.linalg.svd(H)
    S = np.eye(3)
    if np.linalg.det(U @ Vt) < 0:
        S[-1, -1] = -1
    R = (U @ S @ Vt).T
    s = (D * np.diag(S)).sum() / np.mean(np.sum(sc**2, axis=1)) if with_scale else 1.0
    t = mu_t - s * R @ mu_s
    return R, t, s


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("bag", type=Path, nargs="+", help="ROS 2 mcap directory")
    p.add_argument("--estimator-topic", nargs="+", required=True,
                   help="Topic with estimator pose for each bag")
    p.add_argument("--label", nargs="+", required=True,
                   help="Display label for each estimator")
    p.add_argument("--gt-topic", default="/leica/position",
                   help="Ground-truth topic (PointStamped or Odometry)")
    p.add_argument("--imu-topic", default="/imu0",
                   help="IMU topic for time-series view (set to '' to disable)")
    p.add_argument("--source-bag", type=Path, default=None,
                   help="Optional EuRoC source bag (with /cam0 + /cam1 image_raw). "
                        "When given, camera frames are streamed into the viz on the "
                        "same timeline as the trajectories.")
    p.add_argument("--cam-stride", type=int, default=1,
                   help="Decimation for camera frames (1=every frame, 5=every 5th)")
    p.add_argument("--output", type=Path, default=None,
                   help="Output .rrd file. If omitted, opens live viewer.")
    args = p.parse_args()

    assert len(args.bag) == len(args.estimator_topic) == len(args.label), \
        "Need same number of --bag / --estimator-topic / --label"

    rec_name = "vio_benchmark/" + "_vs_".join(args.label)
    if args.output:
        rr.init(rec_name)
        rr.save(str(args.output))
    else:
        rr.init(rec_name, spawn=True)

    palette = [(0.10, 0.40, 0.90), (0.10, 0.75, 0.40), (0.85, 0.55, 0.10),
               (0.70, 0.20, 0.60), (0.50, 0.50, 0.50)]
    gt_color = (0.90, 0.50, 0.10)

    # EuRoC stereo intrinsics — used to draw a camera frustum at each
    # estimator's current pose. Numbers are from EuRoC's official
    # T_BS_cam0/sensor.yaml (the same OpenVINS upstream uses for MH_01).
    EUROC_FX, EUROC_FY = 458.654, 457.296
    EUROC_CX, EUROC_CY = 367.215, 248.375
    IMG_W, IMG_H = 752, 480

    gt_logged = False
    for bag, est_topic, label, color in zip(args.bag, args.estimator_topic, args.label,
                                             palette):
        print(f"loading {bag} ({label} via {est_topic}) ...")
        imu_topic = args.imu_topic if args.imu_topic else None
        est, gt, imu = load_bag(bag, est_topic, args.gt_topic, imu_topic)
        print(f"  est: {len(est)} poses, gt: {len(gt)} points, imu: {len(imu)} samples")
        if len(est) == 0:
            print(f"  WARN: no estimator data on {est_topic}, skipping")
            continue

        # Align estimator to GT with rotation+translation only (no scale).
        # R will also be applied to orientation quaternions below so the
        # body-frame coordinate triads point the right way after alignment.
        R = np.eye(3); t = np.zeros(3)
        if len(gt) > 0:
            gt_t = gt[:, 0]
            est_t = est[:, 0]
            mask = (est_t >= gt_t[0]) & (est_t <= gt_t[-1])
            if mask.sum() > 10:
                idx = nearest_gt(est_t[mask], gt_t, gt[:, 1:4])
                R, t, _ = umeyama(est[mask, 1:4], gt[idx, 1:4], with_scale=False)
                aligned = (R @ est[:, 1:4].T).T + t
            else:
                aligned = est[:, 1:4]
        else:
            aligned = est[:, 1:4]

        # Log GT once (it's the same for both estimators if both bags have it).
        # Use a child path /world/gt/path so we can add more children later
        # (e.g. a GT pose marker) without conflicting with the line strip.
        if not gt_logged and len(gt) > 0:
            rr.log("/world/gt/path", rr.LineStrips3D(
                [gt[:, 1:4]], colors=[gt_color], radii=0.02
            ), static=True)
            gt_logged = True

        # Full aligned trajectory as a static line under /world/<label>/path.
        # NB: deliberately a *child* path, NOT /world/<label>, because the
        # per-timestep Transform3D + Pinhole below need a separate entity
        # path or they'll clobber this line.
        rr.log(f"/world/{label}/path", rr.LineStrips3D(
            [aligned], colors=[color], radii=0.015
        ), static=True)

        # Per-timestep: log a Transform3D + Pinhole frustum at the current
        # pose so the viewer shows orientation and field of view, not just
        # a position dot. The quaternion is rotated by the alignment R so
        # orientations match the GT frame.
        if len(gt) > 0 and gt_logged:
            for i in range(0, len(est), max(1, len(est) // 2000)):
                t_s = est[i, 0]
                rr.set_time("time", duration=t_s - est[0, 0])

                # Aligned position + aligned orientation
                pos = aligned[i]
                # est columns 4..8 are q_xyzw from the estimator
                q_xyzw = est[i, 4:8]
                # Convert quaternion to rotation matrix, apply R alignment,
                # then convert back to quaternion for rerun. Hamilton convention.
                w, x, y, z = q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]
                R_body = np.array([
                    [1-2*(y*y+z*z),   2*(x*y-z*w),   2*(x*z+y*w)],
                    [  2*(x*y+z*w), 1-2*(x*x+z*z),   2*(y*z-x*w)],
                    [  2*(x*z-y*w),   2*(y*z+x*w), 1-2*(x*x+y*y)],
                ])
                R_aligned = R @ R_body
                # Matrix → quaternion (XYZW)
                tr = R_aligned.trace()
                if tr > 0:
                    s = (tr + 1.0) ** 0.5 * 2
                    qw = 0.25 * s
                    qx = (R_aligned[2,1] - R_aligned[1,2]) / s
                    qy = (R_aligned[0,2] - R_aligned[2,0]) / s
                    qz = (R_aligned[1,0] - R_aligned[0,1]) / s
                else:
                    # Fallback for negative-trace cases; cheap and correct enough
                    qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0

                # Use a separate /pose child entity for the moving frustum so
                # we don't clobber the static /path line strip above.
                rr.log(f"/world/{label}/pose",
                       rr.Transform3D(translation=pos.tolist(),
                                      rotation=rr.Quaternion(xyzw=[qx, qy, qz, qw])))
                rr.log(f"/world/{label}/pose",
                       rr.Pinhole(focal_length=[EUROC_FX, EUROC_FY],
                                  width=IMG_W, height=IMG_H,
                                  principal_point=[EUROC_CX, EUROC_CY]))

                # Drift scalar to nearest GT
                j = nearest_gt(np.array([t_s]), gt[:, 0], gt[:, 1:4])[0]
                drift = float(np.linalg.norm(aligned[i] - gt[j, 1:4]))
                rr.log(f"plots/{label}_drift", rr.Scalars(drift))

        # IMU stream
        if len(imu) > 0:
            for s in imu[::max(1, len(imu) // 5000)]:
                rr.set_time("time", duration=s[0] - est[0, 0])
                rr.log("imu/accel/x", rr.Scalars(s[1]))
                rr.log("imu/accel/y", rr.Scalars(s[2]))
                rr.log("imu/accel/z", rr.Scalars(s[3]))

    # Stream camera frames from the EuRoC source bag, on the same timeline.
    # Done last so the time-zero offset (est[0,0]) is settled by an estimator
    # bag first. If you only have the source bag, that's also fine — the
    # estimator section is a no-op then.
    if args.source_bag is not None and args.source_bag.exists():
        print(f"loading camera frames from {args.source_bag} ...")
        # The estimator bags use sim-current-time stamps; the source EuRoC bag
        # uses the original 2014 stamps. For visualization we align the *first*
        # camera frame to t=0 on the same timeline.
        #
        # For each frame we also log under each estimator's pose path so the
        # image projects through that estimator's frustum at the matching
        # timestamp. Rerun applies the latest Transform3D + Pinhole logged
        # at <=t, so a single Image log per estimator-pose-path is enough.
        # Left camera (cam0) goes under each estimator (they all use cam0);
        # right camera (cam1) goes under a single /world/camera_right entity
        # since none of our estimators publishes a right-eye pose directly.
        n_cam0, n_cam1 = 0, 0
        cam0_t0 = None
        with AnyReader([args.source_bag]) as reader:
            for conn, t, raw in reader.messages():
                if conn.topic not in ("/cam0/image_raw", "/cam1/image_raw"):
                    continue
                if cam0_t0 is None:
                    cam0_t0 = t * 1e-9
                t_rel = t * 1e-9 - cam0_t0
                # Decimate
                if conn.topic == "/cam0/image_raw":
                    n_cam0 += 1
                    if (n_cam0 - 1) % args.cam_stride != 0:
                        continue
                else:
                    n_cam1 += 1
                    if (n_cam1 - 1) % args.cam_stride != 0:
                        continue
                msg = reader.deserialize(raw, conn.msgtype)
                # EuRoC frames are mono8, 752x480
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
                rr.set_time("time", duration=t_rel)

                # Standalone view (always logged)
                entity = "camera/left" if conn.topic == "/cam0/image_raw" else "camera/right"
                rr.log(entity, rr.Image(img))

                # Project into each estimator's frustum — log the image under
                # /world/<label>/pose/image so it shows up inside the wedge.
                # The Pinhole + Transform3D at /world/<label>/pose was logged
                # in the earlier estimator loop; the image is a child of that
                # pinhole entity so rerun renders it inside the frustum.
                if conn.topic == "/cam0/image_raw":
                    for label in args.label:
                        rr.log(f"/world/{label}/pose/image", rr.Image(img))
        print(f"  logged {n_cam0 // args.cam_stride} left + {n_cam1 // args.cam_stride} right frames")

    print("done.")
    if args.output:
        print(f"  saved to {args.output} — open with: rerun {args.output}")


if __name__ == "__main__":
    main()
