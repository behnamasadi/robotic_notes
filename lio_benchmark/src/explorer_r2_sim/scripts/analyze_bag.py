#!/usr/bin/env python3
"""
Walk a rosbag recorded by `ros2 bag record` (mcap or sqlite) and produce
a full analysis report under <output-dir>:

  trajectory_xy.png      GT vs VIO vs LIO, top-down
  trajectory_xyz.png     Per-axis X / Y / Z over normalized time
  imu_accel.png          |accel| + accel.{x,y,z} over time
  imu_gyro.png           gyro.{x,y,z} over time
  camera_frames/         A few sample frames from /rs_front/image
  summary.md             Per-estimator path length, end-point error vs
                         GT, IMU sanity, message rates, and a one-line
                         interpretation. Aimed at "show this to someone
                         who didn't drive the robot and they can tell
                         what happened."

The script doesn't need ROS or rclpy installed on the host — it uses the
`rosbags` Python library (which is already an evo dependency). For
images you'll also want opencv-python; if not installed the camera
section is skipped.

Usage:
    python3 scripts/analyze_bag.py <bag-dir> [-o <output-dir>]
    python3 scripts/analyze_bag.py ~/ros2_ws/runs/run_20260514T154145Z
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# rosbags ships with evo. Don't need ROS itself.
from rosbags.highlevel import AnyReader  # type: ignore

try:
    import cv2  # type: ignore
    _HAVE_CV2 = True
except ImportError:
    _HAVE_CV2 = False


# Topics we care about
ODOM_TOPICS = {
    "GT":   "/ground_truth/odom",
    "VIO":  "/ov_msckf/odomimu",          # OpenVINS
    "VINS": "/vins_estimator/odometry",   # VINS-Fusion (zinuok/VINS-Fusion-ROS2)
    "LIO":  "/Odometry",                  # FAST_LIO
}
IMU_TOPIC   = "/imu"
IMAGE_TOPIC = "/rs_front/image"
CMD_TOPIC   = "/cmd_vel"

# Colors aligned with the rviz/sim.rviz palette
COLORS = {"GT": "#e6c100", "VIO": "#39c139", "VINS": "#ff6e3a", "LIO": "#3399ff"}


# ──────────────────────────────────────────────────────────────────────
# Data containers
# ──────────────────────────────────────────────────────────────────────
@dataclass
class OdomTrace:
    t:  list[float] = field(default_factory=list)   # seconds, monotonic
    xs: list[float] = field(default_factory=list)
    ys: list[float] = field(default_factory=list)
    zs: list[float] = field(default_factory=list)

    def array(self) -> np.ndarray:
        return np.array([self.t, self.xs, self.ys, self.zs]).T

    def __len__(self) -> int:
        return len(self.t)

    def path_length(self) -> float:
        a = self.array()
        if a.shape[0] < 2:
            return 0.0
        return float(np.sum(np.linalg.norm(np.diff(a[:, 1:4], axis=0), axis=1)))


@dataclass
class ImuTrace:
    t:    list[float] = field(default_factory=list)
    ax:   list[float] = field(default_factory=list)
    ay:   list[float] = field(default_factory=list)
    az:   list[float] = field(default_factory=list)
    gx:   list[float] = field(default_factory=list)
    gy:   list[float] = field(default_factory=list)
    gz:   list[float] = field(default_factory=list)

    def __len__(self) -> int:
        return len(self.t)


@dataclass
class TopicStats:
    msg_count: int = 0
    t_first:   float | None = None
    t_last:    float | None = None

    def rate(self) -> float:
        if self.msg_count < 2 or self.t_first is None or self.t_last is None:
            return 0.0
        dur = self.t_last - self.t_first
        return self.msg_count / dur if dur > 0 else 0.0


# ──────────────────────────────────────────────────────────────────────
# Bag reading
# ──────────────────────────────────────────────────────────────────────
def stamp_secs(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def read_bag(bag_dir: Path) -> tuple[dict[str, OdomTrace], ImuTrace, dict[str, TopicStats], list[bytes]]:
    """Read odometry + IMU + image-bytes (a few) + per-topic stats from a bag."""
    odoms: dict[str, OdomTrace] = {k: OdomTrace() for k in ODOM_TOPICS}
    imu = ImuTrace()
    stats: dict[str, TopicStats] = {}
    image_msgs: list[bytes] = []

    topics_of_interest = set(ODOM_TOPICS.values()) | {IMU_TOPIC, IMAGE_TOPIC, CMD_TOPIC}

    with AnyReader([bag_dir]) as reader:
        # All-topics rate stats (cheap)
        conns_all = list(reader.connections)
        for c in conns_all:
            stats[c.topic] = TopicStats()

        # Targeted decode for the topics we'll plot
        conns_decode = [c for c in conns_all if c.topic in topics_of_interest]
        for conn, t_ns, raw in reader.messages(connections=conns_decode):
            t = t_ns * 1e-9
            s = stats[conn.topic]
            s.msg_count += 1
            if s.t_first is None:
                s.t_first = t
            s.t_last = t

            if conn.topic in ODOM_TOPICS.values():
                key = next(k for k, v in ODOM_TOPICS.items() if v == conn.topic)
                msg = reader.deserialize(raw, conn.msgtype)
                p = msg.pose.pose.position
                odoms[key].t.append(stamp_secs(msg.header.stamp) or t)
                odoms[key].xs.append(p.x)
                odoms[key].ys.append(p.y)
                odoms[key].zs.append(p.z)

            elif conn.topic == IMU_TOPIC:
                msg = reader.deserialize(raw, conn.msgtype)
                imu.t.append(stamp_secs(msg.header.stamp) or t)
                imu.ax.append(msg.linear_acceleration.x)
                imu.ay.append(msg.linear_acceleration.y)
                imu.az.append(msg.linear_acceleration.z)
                imu.gx.append(msg.angular_velocity.x)
                imu.gy.append(msg.angular_velocity.y)
                imu.gz.append(msg.angular_velocity.z)

            elif conn.topic == IMAGE_TOPIC and len(image_msgs) < 8:
                # Sample 8 frames evenly across the bag — we'll select
                # the actual sample indices after we know the count.
                # For now just keep every Nth.
                if stats[IMAGE_TOPIC].msg_count % 100 == 1:
                    image_msgs.append(raw)

        # Count-only pass for the remaining topics we didn't decode
        for conn, t_ns, _ in reader.messages(connections=[c for c in conns_all if c.topic not in topics_of_interest]):
            t = t_ns * 1e-9
            s = stats[conn.topic]
            s.msg_count += 1
            if s.t_first is None:
                s.t_first = t
            s.t_last = t

    return odoms, imu, stats, image_msgs


# ──────────────────────────────────────────────────────────────────────
# Plots
# ──────────────────────────────────────────────────────────────────────
def plot_xy(odoms: dict[str, OdomTrace], out: Path) -> None:
    fig, ax = plt.subplots(figsize=(10, 10))
    for name in ("GT", "VIO", "VINS", "LIO"):
        d = odoms[name].array()
        if len(d) == 0:
            continue
        ax.plot(d[:, 1], d[:, 2], color=COLORS[name], lw=2,
                label=f"{name}  ({len(d)} poses, end=({d[-1,1]:+.2f}, {d[-1,2]:+.2f}))")
        ax.plot(d[0, 1], d[0, 2], "o", color=COLORS[name], ms=10, mec="black", mew=0.5)
        ax.plot(d[-1, 1], d[-1, 2], "s", color=COLORS[name], ms=10, mec="black", mew=0.5)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title("Trajectories — top-down (circle = start, square = end)")
    ax.legend()
    ax.grid(True)
    ax.set_aspect("equal", adjustable="datalim")
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


def plot_3d(odoms: dict[str, OdomTrace], out: Path) -> None:
    fig = plt.figure(figsize=(11, 9))
    ax = fig.add_subplot(111, projection="3d")
    for name in ("GT", "VIO", "VINS", "LIO"):
        d = odoms[name].array()
        if len(d) == 0:
            continue
        ax.plot(d[:, 1], d[:, 2], d[:, 3], color=COLORS[name], lw=2,
                label=f"{name}  end=({d[-1,1]:+.2f}, {d[-1,2]:+.2f}, {d[-1,3]:+.2f})")
        ax.scatter(d[0, 1], d[0, 2], d[0, 3], color=COLORS[name], s=80,
                   marker="o", edgecolor="black", linewidth=0.5)
        ax.scatter(d[-1, 1], d[-1, 2], d[-1, 3], color=COLORS[name], s=80,
                   marker="s", edgecolor="black", linewidth=0.5)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_title("Trajectories in 3D (circle = start, square = end)")
    ax.legend(loc="upper left")
    # Equalise aspect — matplotlib's 3D axes don't have set_aspect('equal')
    # robustly across versions, so do the math by hand.
    all_pts = np.concatenate(
        [odoms[n].array()[:, 1:4] for n in odoms if len(odoms[n]) > 0])
    if all_pts.size:
        mins = all_pts.min(axis=0)
        maxs = all_pts.max(axis=0)
        ctr = (mins + maxs) / 2
        rng = (maxs - mins).max() / 2 * 1.1
        ax.set_xlim(ctr[0] - rng, ctr[0] + rng)
        ax.set_ylim(ctr[1] - rng, ctr[1] + rng)
        ax.set_zlim(ctr[2] - rng, ctr[2] + rng)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


def plot_xyz(odoms: dict[str, OdomTrace], out: Path) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    labels = ("X [m]", "Y [m]", "Z [m]")
    for i, label in enumerate(labels):
        for name in ("GT", "VIO", "VINS", "LIO"):
            d = odoms[name].array()
            if len(d) == 0:
                continue
            # Use normalized index since GT stamps may be zero or unreliable
            xs = np.linspace(0, 1, len(d))
            axes[i].plot(xs, d[:, i + 1], color=COLORS[name], lw=1.5, label=name)
        axes[i].set_ylabel(label)
        axes[i].grid(True)
        if i == 0:
            axes[i].legend(loc="upper left")
    axes[-1].set_xlabel("Normalised time (0 = start, 1 = end of each trajectory)")
    fig.suptitle("Position components over time")
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


def plot_imu(imu: ImuTrace, out_accel: Path, out_gyro: Path) -> None:
    if len(imu) == 0:
        return
    t = np.array(imu.t)
    t -= t[0]
    ax = np.array(imu.ax)
    ay = np.array(imu.ay)
    az = np.array(imu.az)
    mag = np.sqrt(ax ** 2 + ay ** 2 + az ** 2)

    fig, axes = plt.subplots(4, 1, figsize=(12, 9), sharex=True)
    axes[0].plot(t, mag, color="#444", lw=1)
    axes[0].axhline(9.81, color="red", ls="--", lw=0.8, label="9.81 m/s² (g)")
    axes[0].set_ylabel("|accel| [m/s²]")
    axes[0].legend(loc="upper right")
    axes[0].grid(True)

    for i, (arr, name, color) in enumerate(zip(
            (ax, ay, az), ("accel.x", "accel.y", "accel.z"),
            ("#e07a00", "#0b8a3a", "#0066d6"))):
        axes[i + 1].plot(t, arr, color=color, lw=0.8)
        axes[i + 1].set_ylabel(name + " [m/s²]")
        axes[i + 1].grid(True)
    axes[-1].set_xlabel("time [s] (bag-relative)")
    fig.suptitle("IMU linear acceleration — components + magnitude")
    fig.tight_layout()
    fig.savefig(out_accel, dpi=110)
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(12, 7), sharex=True)
    for i, (arr_l, name, color) in enumerate(zip(
            (imu.gx, imu.gy, imu.gz),
            ("gyro.x", "gyro.y", "gyro.z"),
            ("#e07a00", "#0b8a3a", "#0066d6"))):
        axes[i].plot(t, np.array(arr_l), color=color, lw=0.8)
        axes[i].set_ylabel(name + " [rad/s]")
        axes[i].grid(True)
    axes[-1].set_xlabel("time [s] (bag-relative)")
    fig.suptitle("IMU angular velocity")
    fig.tight_layout()
    fig.savefig(out_gyro, dpi=110)
    plt.close(fig)


def save_sample_frames(bag_dir: Path, out_dir: Path, n_samples: int = 6) -> int:
    """Pick n_samples /rs_front/image messages evenly spread, save as PNG."""
    if not _HAVE_CV2:
        return 0
    out_dir.mkdir(parents=True, exist_ok=True)
    saved = 0
    with AnyReader([bag_dir]) as reader:
        img_conns = [c for c in reader.connections if c.topic == IMAGE_TOPIC]
        if not img_conns:
            return 0
        n_msgs = sum(c.msgcount for c in img_conns)
        if n_msgs == 0:
            return 0
        pick = set(int(i) for i in np.linspace(0, n_msgs - 1, n_samples))
        i = 0
        for conn, _t, raw in reader.messages(connections=img_conns):
            if i in pick:
                msg = reader.deserialize(raw, conn.msgtype)
                w, h = msg.width, msg.height
                # rs_front is rgb8 — buffer is HxWx3 packed bytes
                if msg.encoding in ("rgb8", "RGB8", "8UC3"):
                    img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                elif msg.encoding in ("bgr8", "BGR8"):
                    img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
                elif msg.encoding == "mono8":
                    img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
                else:
                    i += 1
                    continue
                cv2.imwrite(str(out_dir / f"frame_{saved:02d}.png"), img)
                saved += 1
            i += 1
    return saved


# ──────────────────────────────────────────────────────────────────────
# Summary stats
# ──────────────────────────────────────────────────────────────────────
def imu_sanity(imu: ImuTrace) -> dict[str, float]:
    if len(imu) == 0:
        return {}
    ax = np.array(imu.ax); ay = np.array(imu.ay); az = np.array(imu.az)
    mag = np.sqrt(ax ** 2 + ay ** 2 + az ** 2)
    return {
        "n":          float(len(imu)),
        "|a|_mean":   float(np.mean(mag)),
        "|a|_std":    float(np.std(mag)),
        "ax_mean":    float(np.mean(ax)),
        "ay_mean":    float(np.mean(ay)),
        "az_mean":    float(np.mean(az)),
    }


def write_summary(out: Path, bag_dir: Path, odoms: dict[str, OdomTrace],
                  imu: ImuTrace, stats: dict[str, TopicStats], n_frames: int) -> None:
    lines = []
    lines.append(f"# Analysis of `{bag_dir.name}`")
    lines.append("")
    lines.append(f"Bag dir: `{bag_dir}`")
    lines.append("")

    # Topic rates
    lines.append("## Topic message rates")
    lines.append("")
    lines.append("| Topic | Messages | Duration [s] | Rate [Hz] |")
    lines.append("|---|---:|---:|---:|")
    interesting = list(ODOM_TOPICS.values()) + [IMU_TOPIC, IMAGE_TOPIC, CMD_TOPIC,
                                                "/lidar/points", "/lidar/points_lio",
                                                "/path", "/ov_msckf/pathimu",
                                                "/ground_truth/path", "/ground_truth/pose",
                                                "/tf", "/tf_static"]
    for t in interesting:
        s = stats.get(t)
        if s is None or s.msg_count == 0:
            continue
        if s.t_first is not None and s.t_last is not None:
            dur = s.t_last - s.t_first
        else:
            dur = 0.0
        lines.append(f"| `{t}` | {s.msg_count} | {dur:.2f} | {s.rate():.2f} |")
    lines.append("")

    # Trajectory summary
    lines.append("## Trajectory summary")
    lines.append("")
    lines.append("| | poses | start (x, y, z) [m] | end (x, y, z) [m] | path length [m] |")
    lines.append("|---|---:|---|---|---:|")
    for name in ("GT", "LIO", "VIO", "VINS"):
        d = odoms[name].array()
        if len(d) == 0:
            lines.append(f"| **{name}** | 0 | — | — | — |")
            continue
        lines.append(
            f"| **{name}** | {len(d)} "
            f"| ({d[0,1]:+.2f}, {d[0,2]:+.2f}, {d[0,3]:+.2f}) "
            f"| ({d[-1,1]:+.2f}, {d[-1,2]:+.2f}, {d[-1,3]:+.2f}) "
            f"| {odoms[name].path_length():.2f} |"
        )
    lines.append("")

    # End-position error vs GT
    if len(odoms["GT"]) > 0:
        gt_end = odoms["GT"].array()[-1, 1:4]
        gt_pl = odoms["GT"].path_length()
        lines.append("## End-position error vs GT")
        lines.append("")
        lines.append("| | Δx | Δy | Δz | |err| [m] | |err| / GT_path |")
        lines.append("|---|---:|---:|---:|---:|---:|")
        for name in ("LIO", "VIO", "VINS"):
            d = odoms[name].array()
            if len(d) == 0:
                continue
            err = d[-1, 1:4] - gt_end
            mag = float(np.linalg.norm(err))
            rel = (mag / gt_pl) if gt_pl > 0 else float("inf")
            lines.append(
                f"| **{name}** | {err[0]:+.2f} | {err[1]:+.2f} | {err[2]:+.2f} "
                f"| {mag:.2f} | {rel*100:.1f}% |"
            )
        lines.append("")
        # Path-length ratio
        lines.append("## Path-length ratio (estimator / GT)")
        lines.append("")
        lines.append("| | path length [m] | ratio |")
        lines.append("|---|---:|---:|")
        for name in ("LIO", "VIO", "VINS"):
            pl = odoms[name].path_length()
            ratio = (pl / gt_pl) if gt_pl > 0 else float("inf")
            lines.append(f"| **{name}** | {pl:.2f} | {ratio:.2f} |")
        lines.append("")

    # IMU sanity
    sn = imu_sanity(imu)
    if sn:
        lines.append("## IMU sanity")
        lines.append("")
        lines.append("Stationary on flat ground a healthy IMU reports |accel| ≈ 9.81 m/s² "
                     "with gravity aligned mostly along Z.")
        lines.append("")
        lines.append(f"- Samples: {int(sn['n'])}")
        lines.append(f"- |accel| mean ± std: **{sn['|a|_mean']:.3f} ± {sn['|a|_std']:.3f} m/s²**")
        lines.append(f"- Per-axis mean: ax={sn['ax_mean']:+.3f}, ay={sn['ay_mean']:+.3f}, "
                     f"az={sn['az_mean']:+.3f} m/s²")
        lines.append("")

    # Cameras
    lines.append("## Camera samples")
    lines.append("")
    if _HAVE_CV2 and n_frames > 0:
        lines.append(f"{n_frames} sample frames saved under `camera_frames/`. "
                     "Inspect for tracking failure modes (blur, low contrast, "
                     "feature-poor walls).")
    elif not _HAVE_CV2:
        lines.append("`opencv-python` not installed on host — camera-frame export skipped. "
                     "`pip3 install --user --break-system-packages opencv-python` to enable.")
    else:
        lines.append("No images found in bag.")
    lines.append("")

    # Quick interpretation
    lines.append("## Interpretation")
    lines.append("")
    if len(odoms["GT"]) == 0:
        lines.append("- No GT data — `gt_to_path.py` wasn't running or `/ground_truth/odom` "
                     "wasn't recorded.")
    else:
        gt_pl = odoms["GT"].path_length()
        for name in ("LIO", "VIO", "VINS"):
            d = odoms[name].array()
            if len(d) == 0:
                lines.append(f"- **{name}**: no data recorded.")
                continue
            pl = odoms[name].path_length()
            err = float(np.linalg.norm(d[-1, 1:4] - odoms["GT"].array()[-1, 1:4]))
            ratio = pl / gt_pl if gt_pl > 0 else float("inf")
            if ratio > 1.5 or ratio < 0.7:
                lines.append(
                    f"- **{name}**: path-length ratio = **{ratio:.2f}** "
                    f"(should be ≈ 1.0). Scale drift suspected — common cause for mono "
                    f"VIO with forward driving (no parallax), or feature loss in dim scenes.")
            elif err > 0.05 * gt_pl:
                lines.append(
                    f"- **{name}**: scale OK (ratio {ratio:.2f}) but end-point error "
                    f"{err:.2f} m on a {gt_pl:.1f} m drive is high — likely a yaw offset "
                    f"between the estimator's world frame and GT. Run `evo_ape -va` for "
                    f"Umeyama-aligned APE to confirm.")
            else:
                lines.append(
                    f"- **{name}**: looks healthy. End-point error {err:.2f} m on a "
                    f"{gt_pl:.1f} m drive ({err/gt_pl*100:.1f}%).")

    lines.append("")
    lines.append("---")
    lines.append("")
    lines.append("See [`docs/ANALYSIS.md`](../docs/ANALYSIS.md) for a worked-example "
                 "case study and per-failure-mode root-cause notes.")
    lines.append("")

    out.write_text("\n".join(lines))


# ──────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────
def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag", help="Path to bag directory (contains metadata.yaml)")
    parser.add_argument("-o", "--output", default=None,
                        help="Output dir (default: <bag>_report/ next to the bag)")
    args = parser.parse_args()

    bag_dir = Path(args.bag).expanduser().resolve()
    if not (bag_dir / "metadata.yaml").exists():
        print(f"[analyze_bag] No metadata.yaml in {bag_dir}", file=sys.stderr)
        return 1
    out_dir = Path(args.output).expanduser().resolve() if args.output \
        else bag_dir.parent / (bag_dir.name + "_report")
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"[analyze_bag] Reading {bag_dir}")
    odoms, imu, stats, _ = read_bag(bag_dir)

    print(f"[analyze_bag] Writing plots to {out_dir}")
    plot_xy(odoms,  out_dir / "trajectory_xy.png")
    plot_3d(odoms,  out_dir / "trajectory_3d.png")
    plot_xyz(odoms, out_dir / "trajectory_xyz.png")
    plot_imu(imu,   out_dir / "imu_accel.png",
                    out_dir / "imu_gyro.png")

    print("[analyze_bag] Sampling camera frames")
    n_frames = save_sample_frames(bag_dir, out_dir / "camera_frames", n_samples=6)

    print("[analyze_bag] Writing summary.md")
    write_summary(out_dir / "summary.md", bag_dir, odoms, imu, stats, n_frames)

    print(f"\n[analyze_bag] Done. Open {out_dir / 'summary.md'} to read the report.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
