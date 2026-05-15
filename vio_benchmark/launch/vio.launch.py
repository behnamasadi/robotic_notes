# Attach OpenVINS to a running sim.
#   ros2 launch explorer_r2_sim vio.launch.py
#
# Subscribes to /imu + /rs_front/image, publishes /ov_msckf/*. The sim's
# RViz (rviz/sim.rviz) already has displays for those topics, so this
# launch does NOT spawn its own RViz.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Seconds to wait after launch before starting OpenVINS itself. The
# rover spawns at z=0.4, settles to ~z=0.25, and produces an impact
# transient on /imu during the first second. If OpenVINS subscribes
# during that window its static-init triggers on impact data and
# converges to a wrong gravity vector → constant residual acceleration
# → drift forever. Wait it out.
VIO_START_DELAY_SEC = 8.0


def generate_launch_description():
    pkg_share = get_package_share_directory("explorer_r2_sim")
    default_cfg = os.path.join(
        pkg_share, "config", "openvins", "estimator_config.yaml")

    # Arg is `vio_config_path` (not the more obvious `config_path`) so that
    # this launch can be IncludeLaunchDescription'd alongside lio.launch.py
    # from cave.launch.py without the two clobbering each other's value.
    vio_config_path = LaunchConfiguration("vio_config_path")

    # Re-use OpenVINS' own subscribe.launch.py — it knows how to bring up
    # run_subscribe_msckf with the right parameter wiring.
    ov_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ov_msckf"), "launch", "subscribe.launch.py"
            ])
        ),
        launch_arguments={
            "config_path":  vio_config_path,
            "use_stereo":   "true",        # rs_front + rs_front_right
            "max_cameras":  "2",
            "verbosity":    "INFO",
            "rviz_enable":  "false",
        }.items(),
    )

    # Glue OpenVINS' `global` frame to the sim's `explorer_r2/odom` so RViz
    # can transform /ov_msckf/* into the fixed frame. Identity at startup;
    # OpenVINS drifts away from this static link over time — that's how
    # we visualise its error against ground truth.
    vio_world_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="vio_world_to_odom",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "explorer_r2/odom",
            "--child-frame-id", "global",
        ],
        output="screen",
    )

    # The static TF can start immediately — it has no init phase. Delay
    # only the actual OpenVINS node so the spawn-drop transient has time
    # to die out before VIO starts collecting IMU samples for init.
    return LaunchDescription([
        DeclareLaunchArgument("vio_config_path", default_value=default_cfg,
                              description="OpenVINS estimator_config.yaml"),
        vio_world_tf,
        TimerAction(period=VIO_START_DELAY_SEC, actions=[ov_launch]),
    ])
