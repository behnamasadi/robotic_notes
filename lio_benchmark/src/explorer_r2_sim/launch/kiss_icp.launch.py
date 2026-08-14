# Attach KISS-ICP (PRBonn, Vizzo et al.) to a running sim — a LiDAR-only
# baseline (no IMU input) so the side-by-side compares fairly the
# benefit of IMU fusion: ground truth vs FAST-LIO (LIO, EKF) vs DLIO
# (LIO, GICP) vs KISS-ICP (no IMU, just point-to-point ICP).
#
# KISS-ICP is intentionally a single node with almost no parameters —
# its design pitch is "ICP-only odometry that works without per-dataset
# tuning". We give it /lidar/points and a static TF that glues its odom
# frame to the sim's, and that's it.
#
# Sim RViz (rviz/sim.rviz) renders /kiss/odometry + /kiss/trajectory as
# the orange KISS-ICP path.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    sim_share      = get_package_share_directory("explorer_r2_sim")
    kiss_share     = get_package_share_directory("kiss_icp")
    sim_override   = os.path.join(sim_share, "config", "kiss_icp.yaml")
    kiss_default   = os.path.join(kiss_share, "config", "config.yaml")

    # Later files win on conflicts → sim_override goes last.
    common_params = [
        kiss_default,
        sim_override,
        {
            "use_sim_time":     True,
            # KISS-ICP looks up the LiDAR → base_link transform via TF if
            # base_frame is non-empty, and compensates so its output pose
            # tracks the base (not the LiDAR centre 1.05 m higher). This
            # matters for a fair comparison against FAST-LIO / DLIO,
            # which also report base-link poses.
            "base_frame":       "explorer_r2/base_link",
            "lidar_odom_frame": "kiss_icp/odom",
            "publish_odom_tf":  False,
        },
    ]

    kiss_node = Node(
        package="kiss_icp",
        executable="kiss_icp_node",
        name="kiss_icp_node",
        output="screen",
        parameters=common_params,
        # /lidar/points_lio is the lidar_field_adapter's enriched output —
        # raw /lidar/points has no per-point time field, which makes
        # KISS-ICP print "Disabling scan deskewing" and fall back to
        # per-scan registration. The adapter adds ring + t fields for
        # FAST-LIO; KISS-ICP picks them up automatically and enables
        # constant-velocity deskew, matching its design intent.
        remappings=[("pointcloud_topic", "/lidar/points_lio")],
    )

    # Glue kiss_icp/odom into the sim's fixed frame. Identity at startup;
    # KISS-ICP drifts from this static link over time — same pattern as
    # FAST-LIO's camera_init and DLIO's dlio/odom.
    kiss_world_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="kiss_icp_world_to_odom",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "explorer_r2/odom",
            "--child-frame-id", "kiss_icp/odom",
        ],
        output="screen",
    )

    # KISS-ICP publishes /kiss/odometry but no Path topic of its own
    # (FAST-LIO and DLIO both ship a Path natively). Convert the
    # Odometry stream into a Path on /kiss/path so the RViz config can
    # render the trajectory as a line, matching the other two estimators.
    kiss_path_publisher = Node(
        package="explorer_r2_sim",
        executable="odom_to_path.py",
        name="kiss_icp_odom_to_path",
        parameters=[{
            "use_sim_time":  True,
            "input_topic":   "/kiss/odometry",
            "output_topic":  "/kiss/path",
            "fixed_frame":   "explorer_r2/odom",
        }],
        output="screen",
    )

    return LaunchDescription([kiss_world_tf, kiss_node, kiss_path_publisher])
