# Attach DLIO (Direct LiDAR-Inertial Odometry, UCLA Vectr) to a running
# sim, as a side-by-side comparison to FAST-LIO (lio.launch.py).
#
# Two nodes plus a static TF glue, mirroring the FAST-LIO setup:
#   1. dlio_odom_node — core estimator. Subscribes /lidar/points + /imu,
#      publishes /dlio/odom_node/{odom,path,pose}, /dlio/odom_node/pointcloud/deskewed.
#      DLIO does its own continuous-time deskewing from IMU integration,
#      so we feed it the raw /lidar/points (no ring/t adapter required —
#      unlike FAST-LIO's Ouster preprocessor).
#   2. dlio_map_node — assembles keyframes into a global map.
#   3. static TF: explorer_r2/odom → dlio/odom, so RViz can render
#      DLIO's outputs in the sim's fixed frame.
#
# The sim's RViz (rviz/sim.rviz) already has DLIO Odom/Path/Map displays
# wired to /dlio/odom_node/* topics in green (~80;200;120).

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share        = get_package_share_directory("explorer_r2_sim")
    dlio_pkg_share   = get_package_share_directory("direct_lidar_inertial_odometry")
    sim_override     = os.path.join(pkg_share, "config", "dlio.yaml")
    dlio_default     = os.path.join(dlio_pkg_share, "cfg", "dlio.yaml")
    dlio_params      = os.path.join(dlio_pkg_share, "cfg", "params.yaml")

    # Order matters: later files override earlier ones key-by-key, so the
    # sim_override yaml goes last to win on frames + extrinsics.
    common_params = [dlio_params, dlio_default, sim_override]

    remaps = [
        ("pointcloud", "/lidar/points"),
        ("imu",        "/imu"),
        ("odom",       "/dlio/odom_node/odom"),
        ("pose",       "/dlio/odom_node/pose"),
        ("path",       "/dlio/odom_node/path"),
        ("kf_pose",    "/dlio/odom_node/keyframes"),
        ("kf_cloud",   "/dlio/odom_node/pointcloud/keyframe"),
        ("deskewed",   "/dlio/odom_node/pointcloud/deskewed"),
    ]

    dlio_odom = Node(
        package="direct_lidar_inertial_odometry",
        executable="dlio_odom_node",
        name="dlio_odom_node",
        parameters=common_params,
        remappings=remaps,
        output="screen",
    )

    dlio_map = Node(
        package="direct_lidar_inertial_odometry",
        executable="dlio_map_node",
        name="dlio_map_node",
        parameters=common_params,
        remappings=[("keyframes", "/dlio/odom_node/pointcloud/keyframe")],
        output="screen",
    )

    # Glue DLIO's `dlio/odom` (defined in config/dlio.yaml) into the
    # sim's fixed frame. Identity at startup; DLIO drifts from this
    # static link over time — same pattern as the FAST-LIO glue.
    dlio_world_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="dlio_world_to_odom",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "explorer_r2/odom",
            "--child-frame-id", "dlio/odom",
        ],
        output="screen",
    )

    return LaunchDescription([dlio_world_tf, dlio_odom, dlio_map])
