#!/usr/bin/env python3
# gz gpu_lidar publishes a basic PointCloud2 with only x, y, z, intensity
# fields. FAST_LIO's Ouster (lidar_type=3) and Velodyne (lidar_type=2)
# preprocessors require per-point `ring` (laser-line index) and `t`
# (timestamp offset within the scan). Without those, PCL fills them
# with 0, all points end up on ring 0, and FAST_LIO's per-ring filters
# drop almost everything — you see "No point, skip this scan!" forever.
#
# This node enriches the basic gz cloud with synthesized ring + t fields
# computed from each point's geometry, so FAST_LIO sees the topic it
# expects. Defaults match the rs/Ouster-style 16-line lidar configured
# in models/explorer_r2/model.sdf.
#
# Topics:
#   in : /lidar/points (configurable via input_topic param)
#   out: /lidar/points_lio (configurable via output_topic param)
#
# Usage:
#   ros2 run explorer_r2_sim lidar_field_adapter.py
#   ros2 run explorer_r2_sim lidar_field_adapter.py \
#     --ros-args -p scan_lines:=32 -p min_elev_deg:=-22.5 -p max_elev_deg:=22.5

import math

import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField


class LidarFieldAdapter(Node):
    def __init__(self):
        super().__init__("lidar_field_adapter")
        self.declare_parameter("scan_lines", 16)
        self.declare_parameter("min_elev_deg", -15.0)
        self.declare_parameter("max_elev_deg",  15.0)
        self.declare_parameter("scan_rate_hz",  15.0)
        self.declare_parameter("input_topic",  "/lidar/points")
        self.declare_parameter("output_topic", "/lidar/points_lio")

        self.N = int(self.get_parameter("scan_lines").value)
        self.min_elev = math.radians(float(self.get_parameter("min_elev_deg").value))
        self.max_elev = math.radians(float(self.get_parameter("max_elev_deg").value))
        self.elev_span = self.max_elev - self.min_elev
        self.scan_period_ns = int(1e9 / float(self.get_parameter("scan_rate_hz").value))

        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self._sub = self.create_subscription(PointCloud2, in_topic, self._cb, qos)
        self._pub = self.create_publisher(PointCloud2, out_topic, qos)

        self.get_logger().info(
            f"Enriching {in_topic} → {out_topic} "
            f"(N={self.N} lines, elev=[{math.degrees(self.min_elev):.1f}, "
            f"{math.degrees(self.max_elev):.1f}]°, "
            f"{1e9/self.scan_period_ns:.0f} Hz)")

    # ------------------------------------------------------------------
    # Output point dtype matches ouster_ros::Point field names — FAST_LIO
    # looks these up by name via PCL's PointCloud2 → PointCloud conversion.
    # Field types match FAST_LIO's ouster_ros::Point definition exactly
    # (preprocess.h: ring is std::uint8_t, not uint16). PCL matches
    # PointCloud2 fields by NAME and TYPE — a UINT16 ring won't bind to
    # a uint8_t struct member.
    _OUT_DTYPE = np.dtype([
        ("x",            np.float32),
        ("y",            np.float32),
        ("z",            np.float32),
        ("intensity",    np.float32),
        ("t",            np.uint32),
        ("reflectivity", np.uint16),
        ("ring",         np.uint8),
        ("ambient",      np.uint16),
        ("range",        np.uint32),
    ])

    _FIELD_TYPES = [
        ("x",            PointField.FLOAT32),
        ("y",            PointField.FLOAT32),
        ("z",            PointField.FLOAT32),
        ("intensity",    PointField.FLOAT32),
        ("t",            PointField.UINT32),
        ("reflectivity", PointField.UINT16),
        ("ring",         PointField.UINT8),
        ("ambient",      PointField.UINT16),
        ("range",        PointField.UINT32),
    ]
    # ------------------------------------------------------------------

    def _cb(self, msg: PointCloud2) -> None:
        pts = pc2.read_points_numpy(
            msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
        if pts.size == 0:
            return

        x = pts[:, 0].astype(np.float32)
        y = pts[:, 1].astype(np.float32)
        z = pts[:, 2].astype(np.float32)
        intensity = pts[:, 3].astype(np.float32)

        # ring from elevation
        r_xy = np.sqrt(x * x + y * y)
        elev = np.arctan2(z, np.maximum(r_xy, 1e-9))
        ring_f = (elev - self.min_elev) / self.elev_span * (self.N - 1)
        ring = np.clip(np.round(ring_f), 0, self.N - 1).astype(np.uint8)

        # t (per-point time offset within scan) — linear in azimuth
        azi = np.arctan2(y, x)
        azi = np.where(azi < 0, azi + 2 * np.pi, azi)
        t_ns = (azi / (2 * np.pi) * self.scan_period_ns).astype(np.uint32)

        # range in mm
        r_3d = np.sqrt(x * x + y * y + z * z)
        rng_mm = np.clip(r_3d * 1000.0, 0, np.iinfo(np.uint32).max).astype(np.uint32)

        out = np.empty(x.shape[0], dtype=self._OUT_DTYPE)
        out["x"] = x
        out["y"] = y
        out["z"] = z
        out["intensity"] = intensity
        out["t"] = t_ns
        out["reflectivity"] = np.clip(intensity, 0, 65535).astype(np.uint16)
        out["ring"] = ring
        out["ambient"] = 0
        out["range"] = rng_mm

        cloud = PointCloud2()
        cloud.header = msg.header
        cloud.height = 1
        cloud.width = int(out.shape[0])
        cloud.is_dense = True
        cloud.is_bigendian = False
        cloud.point_step = self._OUT_DTYPE.itemsize
        cloud.row_step = cloud.point_step * cloud.width

        offset = 0
        fields = []
        for name, ros_type in self._FIELD_TYPES:
            pf = PointField()
            pf.name = name
            pf.offset = offset
            pf.datatype = ros_type
            pf.count = 1
            fields.append(pf)
            offset += self._OUT_DTYPE.fields[name][0].itemsize
        cloud.fields = fields
        cloud.data = out.tobytes()

        self._pub.publish(cloud)


def main():
    rclpy.init()
    try:
        rclpy.spin(LidarFieldAdapter())
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
