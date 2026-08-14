# Launch file for the LIO-SAM sibling container (see docker-compose.yml
# `lio_sam` service). Replaces upstream's run.launch.py — that one also
# starts robot_state_publisher with their own xacro and a static
# map→odom publisher, both of which dump bare-named frames (`map`,
# `odom`, `base_link`, `lidar_link`, …) into our TF graph and conflict
# with ours.
#
# This launch only runs the four LIO-SAM nodes themselves, all loading
# the sim-specific params (frames, extrinsic, topics) from the params
# yaml mounted at /root/ros2_ws/install/lio_sam/share/lio_sam/config/
# params.yaml (volume-mounted by compose from our config/lio_sam.yaml).
# The two static TFs LIO-SAM needs (explorer_r2/odom → lio_sam/map and
# lio_sam/base_link → lio_sam/lidar_link) are published from the sim
# container's world.launch.py and reach this container via DDS.
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params = PathJoinSubstitution(
        [FindPackageShare("lio_sam"), "config", "params.yaml"]
    )

    # Static TF that LIO-SAM's imuPreintegration needs at startup:
    # lookupTransform(lidarFrame, baselinkFrame). Both frames live in
    # the lio_sam/ namespace; baselinkFrame isn't published until
    # mapOptimization runs — chicken-and-egg. Pre-seeding this static
    # with the configured LiDAR-1.05-m-above-base extrinsic makes the
    # lookup succeed immediately.
    extrinsic_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lio_sam_baselink_to_lidar",
        arguments=[
            "--x", "0", "--y", "0", "--z", "1.05",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "lio_sam/base_link",
            "--child-frame-id", "lio_sam/lidar_link",
        ],
        output="screen",
    )

    # Static TF gluing lio_sam/map (LIO-SAM's root) into the sim's
    # fixed frame (explorer_r2/odom). The z=1.05 shift compensates an
    # internal LIO-SAM behaviour: it tracks the LiDAR pose via scan
    # matching, then applies our configured extrinsic to derive
    # base_link — but its internal origin is at the FIRST LiDAR pose,
    # so when it computes base = lidar - extrinsic, it ends up
    # publishing lio_sam/base_link at z=-1.05 in its own odom frame
    # instead of z=0. Lifting the glue by +1.05 cancels that offset
    # so lio_sam/base_link lands at the correct world z. Works for
    # arbitrary rover motion because the IMU↔LiDAR extrinsic is rigid.
    map_glue_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lio_sam_world_to_map",
        arguments=[
            "--x", "0", "--y", "0", "--z", "1.05",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "explorer_r2/odom",
            "--child-frame-id", "lio_sam/map",
        ],
        output="screen",
    )

    # Bridge lio_sam/map → lio_sam/odom with a static identity TF.
    # LIO-SAM's mapOptimization publishes this transform only after a
    # loop closure or initial scan-to-map alignment fires; on short
    # straight drives it never appears, leaving lio_sam/odom (and
    # everything downstream — lio_sam/base_link, lio_sam/lidar_link)
    # disconnected from the rest of the TF tree, so RViz reports
    # "No transform from lio_sam/base_link to explorer_r2/odom".
    # The static identity here fills the gap so the chain stays
    # connected from the first scan. When LIO-SAM does start
    # publishing map→odom itself (post-loop-closure), tf2 will see
    # two publishers on the same edge and warn — that's the cue to
    # revisit this static.
    map_to_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lio_sam_map_to_odom",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "lio_sam/map",
            "--child-frame-id", "lio_sam/odom",
        ],
        output="screen",
    )

    return LaunchDescription([
        extrinsic_tf,
        map_glue_tf,
        map_to_odom_tf,
        Node(
            package="lio_sam", executable="lio_sam_imuPreintegration",
            name="lio_sam_imuPreintegration",
            parameters=[params], output="screen",
        ),
        Node(
            package="lio_sam", executable="lio_sam_imageProjection",
            name="lio_sam_imageProjection",
            parameters=[params], output="screen",
        ),
        Node(
            package="lio_sam", executable="lio_sam_featureExtraction",
            name="lio_sam_featureExtraction",
            parameters=[params], output="screen",
        ),
        Node(
            package="lio_sam", executable="lio_sam_mapOptimization",
            name="lio_sam_mapOptimization",
            parameters=[params], output="screen",
        ),
    ])
