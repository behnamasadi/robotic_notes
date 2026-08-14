# Run the four estimators against a recorded bag instead of Gazebo.
# Brings up: ros_gz_bridge is skipped, Gazebo is skipped, only the
# estimator nodes + ground-truth republisher (when GT is in the bag)
# + RViz + a `ros2 bag play` of the user-supplied bag.
#
# Usage:
#   ros2 launch explorer_r2_sim bag.launch.py \
#     bag_path:=/datasets/newer_college/quad_easy \
#     dataset:=newer_college
#
# The `dataset:` argument selects a per-dataset config override under
# config/datasets/<dataset>.yaml (topic remaps, frame names, extrinsic
# overrides per LIO). See config/datasets/newer_college.yaml as a
# template.
#
# Sensor topic conventions assumed across datasets — your override
# yaml will remap the bag's actual topic names to:
#   /lidar/points       PointCloud2 (raw scan)
#   /lidar/points_lio   PointCloud2 with ring + t fields (LIO-SAM /
#                       FAST-LIO Ouster preprocessor expect this)
#   /imu                sensor_msgs/Imu
#   /ground_truth/pose  tf2_msgs/TFMessage from a GT broadcaster (if
#                       the bag has GT)
#
# What this launch skips compared to cave.launch.py:
#   - Gazebo (no `world.launch.py`)
#   - spawn_robot.launch.py
#   - ros_gz_bridge
#   - joy / teleop / rqt_steering (you drive the rover by playing the bag)

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("explorer_r2_sim")
    launch_dir = os.path.join(pkg_share, "launch")

    def _setup(context, *_a, **_kw):
        bag_path = LaunchConfiguration("bag_path").perform(context)
        dataset = LaunchConfiguration("dataset").perform(context)
        rate = LaunchConfiguration("rate").perform(context)
        loop = LaunchConfiguration("loop").perform(context)

        if not bag_path:
            raise RuntimeError(
                "bag.launch.py requires bag_path:=/path/to/bag (rosbag2 "
                "directory, or ROS1 bag — auto-detected by `ros2 bag play`)")

        # Per-dataset overrides — frame names, extrinsics, topic remaps
        # for each LIO. Loaded by the lio.launch.py / dlio.launch.py /
        # kiss_icp.launch.py / lio_sam_container.launch.py via their
        # own param-file mechanisms.
        dataset_cfg_dir = os.path.join(pkg_share, "config", "datasets")
        dataset_yaml = os.path.join(dataset_cfg_dir, f"{dataset}.yaml")
        if not os.path.exists(dataset_yaml):
            print(f"[bag.launch.py] WARNING: dataset config not found at "
                  f"{dataset_yaml} — using default in-sim configs, results "
                  f"will probably be off (wrong frame names, extrinsics).")

        # Bag player. `--clock` makes the player publish /clock so
        # use_sim_time:=true everywhere works. `--rate` lets you speed
        # up or slow down replay. `--loop` repeats the bag indefinitely.
        # `--remap` rewrites the bag's topic names to the canonical
        # ones our estimators subscribe to (/lidar/points, /imu).
        # Per-dataset topic conventions live in
        # config/datasets/<dataset>.yaml; for now the common cases
        # are hard-coded here. Add more elif branches as you add
        # more datasets.
        bag_play_cmd = ["ros2", "bag", "play", bag_path,
                        "--clock", "1000.0",
                        "--rate", rate]
        if loop == "true":
            bag_play_cmd.append("--loop")

        if dataset == "newer_college":
            bag_play_cmd += [
                "--remap",
                "/os_cloud_node/points:=/lidar/points",
                "/os_cloud_node/imu:=/imu",
            ]
        elif dataset == "kitti":
            bag_play_cmd += [
                "--remap",
                "/kitti/velo/pointcloud:=/lidar/points",
                "/kitti/oxts/imu:=/imu",
            ]
        elif dataset == "hilti":
            bag_play_cmd += [
                "--remap",
                "/hesai/pandar:=/lidar/points",
                "/alphasense/imu:=/imu",
            ]
        # else: assume the bag publishes /lidar/points and /imu directly

        bag_player = ExecuteProcess(cmd=bag_play_cmd, output="screen")

        # Cloud's header.frame_id from real-world bags (os_sensor for
        # Newer College, velo_link for KITTI, etc.) isn't part of our
        # sim's TF tree, so RViz can't transform it into the Fixed
        # Frame (explorer_r2/odom) and the Lidar Cloud display sits
        # silent with a "Could not transform" error.
        #
        # The fix: tie the cloud's frame to one estimator's moving
        # baselink. We pick DLIO (most reliable in our experience) so
        # the cloud renders at where DLIO thinks the rover is. The
        # other estimators' trajectories visualise their *drift
        # relative to DLIO*. Static TFs are fine as children of
        # dynamic frames in tf2.
        from launch.actions import OpaqueFunction as _OF  # noqa
        bag_cloud_frame = {
            "newer_college": "os_sensor",
            "kitti":         "velo_link",
            "hilti":         "hesai_lidar",
        }.get(dataset)
        cloud_tf_nodes = []
        if bag_cloud_frame:
            cloud_tf_nodes.append(Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=f"bag_cloud_to_dlio_baselink",
                arguments=[
                    "--x", "0", "--y", "0", "--z", "0",
                    "--roll", "0", "--pitch", "0", "--yaw", "0",
                    "--frame-id", "dlio/baselink",
                    "--child-frame-id", bag_cloud_frame,
                ],
                output="screen",
            ))

        # RViz
        rviz_cfg = os.path.join(pkg_share, "rviz", "sim.rviz")
        rviz_node = Node(
            package="rviz2", executable="rviz2", name="rviz2",
            arguments=["-d", rviz_cfg],
            parameters=[{"use_sim_time": True}],
            output="screen",
        )

        # lidar_field_adapter.py: republishes /lidar/points →
        # /lidar/points_lio with synthesised ring + t fields. FAST-LIO
        # and KISS-ICP both subscribe to /lidar/points_lio (per
        # config/lio.yaml and config/kiss_icp.yaml), so without this
        # adapter they sit silent. DLIO subscribes to /lidar/points
        # directly and doesn't need it. For Ouster bags (NCD, MulRan,
        # HILTI) the source cloud usually already has ring + t; the
        # adapter still relays the topic so downstream subscribers see
        # something on /lidar/points_lio.
        lidar_adapter = Node(
            package="explorer_r2_sim",
            executable="lidar_field_adapter.py",
            name="lidar_field_adapter",
            output="screen",
            parameters=[{
                "scan_lines":     64,        # NCD's OS1 has 64 beams
                "min_elev_deg":  -22.5,
                "max_elev_deg":   22.5,
                "scan_rate_hz":   10.0,
                "input_topic":   "/lidar/points",
                "output_topic":  "/lidar/points_lio",
                "use_sim_time":  True,
            }],
        )

        # Estimators — reuse the same launches that cave.launch.py uses.
        # Each is gated by whether its package is installed (same
        # try/except pattern as cave.launch.py).
        actions = [bag_player, rviz_node, lidar_adapter] + cloud_tf_nodes

        from ament_index_python.packages import PackageNotFoundError
        for pkg, launch_file, name in [
            ("fast_lio",                          "lio.launch.py",       "FAST-LIO"),
            ("direct_lidar_inertial_odometry",    "dlio.launch.py",      "DLIO"),
            ("kiss_icp",                          "kiss_icp.launch.py",  "KISS-ICP"),
        ]:
            try:
                get_package_share_directory(pkg)
                actions.append(IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(launch_dir, launch_file))))
            except PackageNotFoundError:
                print(f"[bag.launch.py] {pkg} not installed — skipping {name}.")

        return actions

    return LaunchDescription([
        DeclareLaunchArgument(
            "bag_path", default_value="",
            description="Absolute path to a rosbag2 directory (or ROS 1 .bag). "
                        "Required."),
        DeclareLaunchArgument(
            "dataset", default_value="default",
            description="Name of a dataset config under config/datasets/<NAME>.yaml. "
                        "Common values: newer_college, kitti, hilti, mulran."),
        DeclareLaunchArgument(
            "rate", default_value="1.0",
            description="Bag playback rate (1.0 = real-time, 0.5 = half speed, etc.)."),
        DeclareLaunchArgument(
            "loop", default_value="false",
            description="If true, loop the bag indefinitely."),
        OpaqueFunction(function=_setup),
    ])
