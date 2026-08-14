# World only — gz sim + bridge + RViz + joy + teleop. No robot, no
# estimators. Used as Mode 2 step 1, and IncludeLaunchDescription'd from
# cave.launch.py (Mode 1).
#
# Launch arguments:
#   world:=<preset|path|fuel-url>   tunnel | cave | rubicon | tugbot_depot
#                                   | singapore_river, or a local SDF path,
#                                   or a Fuel world URL. Default: tunnel.
#   gui / rviz / joy / teleop / rqt_steering / verbose — as before.

import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


WORLD_PRESETS: "dict[str, str]" = {
    "tunnel":          "tunnel.sdf",
    "cave":            "cave.sdf",
    "rubicon":         "https://app.gazebosim.org/abdsemiz/fuel/worlds/Rubicon%20World",
    "tugbot_depot":    "https://app.gazebosim.org/Aiosama/fuel/worlds/tugbot_depot%201",
    "singapore_river": "https://app.gazebosim.org/monkescripts/fuel/worlds/Singapore%20River%20Robot%20X%202026%20world",
}

_APP_FUEL_RE = re.compile(r"^https?://app\.gazebosim\.org/([^/]+)/fuel/worlds/(.+)$")


def _rewrite_fuel_url(url):
    m = _APP_FUEL_RE.match(url)
    if not m:
        return url
    user, name = m.group(1), m.group(2)
    return f"https://fuel.gazebosim.org/1.0/{user}/worlds/{name}"


def resolve_world(world_arg, pkg_share):
    value = WORLD_PRESETS[world_arg] if world_arg in WORLD_PRESETS else world_arg
    if value.endswith(".sdf") and not os.path.isabs(value):
        return os.path.join(pkg_share, "worlds", value)
    if value.startswith("http"):
        return _rewrite_fuel_url(value)
    return value


def generate_launch_description():
    pkg_share = get_package_share_directory("explorer_r2_sim")
    bridge_cfg = os.path.join(pkg_share, "config", "bridge.yaml")
    joy_cfg = os.path.join(pkg_share, "config", "joy_teleop.yaml")
    rviz_cfg = os.path.join(pkg_share, "rviz", "sim.rviz")

    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    teleop = LaunchConfiguration("teleop")
    joy = LaunchConfiguration("joy")
    verbose = LaunchConfiguration("verbose")
    server_only = PythonExpression(["'' if '", gui, "' == 'true' else ' -s'"])
    # --headless-rendering forces gz-sim's ogre2 backend onto EGL offscreen
    # render targets, regardless of $DISPLAY. Without this flag, gz-sim with
    # $DISPLAY set will try GLX first, fail or fall back to Mesa software,
    # and drag the sensor cameras down to single-digit Hz. The env vars
    # __EGL_VENDOR_LIBRARY_FILENAMES and OGRE_RTT_MODE=FBO in compose are
    # necessary but not sufficient — the renderer also needs to be *told*
    # to take the offscreen path, hence this flag.
    headless_flag = PythonExpression(
        ["'' if '", gui, "' == 'true' else ' --headless-rendering'"])

    def _make_gz_launch(context, *_args, **_kwargs):
        world_resolved = resolve_world(
            LaunchConfiguration("world").perform(context), pkg_share)
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"
                ])
            ),
            launch_arguments={
                "gz_args": [world_resolved, " -r", server_only,
                            headless_flag, " -v ", verbose],
                "on_exit_shutdown": "true",
            }.items(),
        )]

    gz_sim_launch = OpaqueFunction(function=_make_gz_launch)

    parameter_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        parameters=[{"config_file": bridge_cfg, "use_sim_time": True}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_cfg],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(rviz),
    )

    teleop_proc = ExecuteProcess(
        cmd=[
            "xterm", "-e",
            "ros2", "run", "teleop_twist_keyboard", "teleop_twist_keyboard",
            "--ros-args", "-r", "cmd_vel:=/cmd_vel",
        ],
        output="screen",
        condition=IfCondition(teleop),
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[joy_cfg, {"use_sim_time": True}],
        output="screen",
        condition=IfCondition(joy),
    )
    joy_teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        parameters=[joy_cfg, {"use_sim_time": True}],
        remappings=[("/cmd_vel", "/cmd_vel")],
        output="screen",
        condition=IfCondition(joy),
    )

    rqt_steering = ExecuteProcess(
        cmd=["rqt", "--standalone", "rqt_robot_steering"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("rqt_steering")),
    )

    # Ground-truth republisher: converts /ground_truth/pose (TFMessage from
    # gz) into /ground_truth/path (Path) + /ground_truth/odom (Odometry) so
    # RViz can draw the GT trail and arrow alongside VIO / LIO / wheel-odom
    # in the same view. Subtracts the first sample so the trail starts at
    # the same origin as wheel-odom.
    #
    # When rover_tf:=ground_truth (default), gt_to_path also broadcasts the
    # explorer_r2/odom → explorer_r2/base_link TF — this anchors the rover
    # mesh in RViz at the true pose so LIO drift shows up as offset arrows
    # rather than the rover itself moving wrong.
    # When rover_tf:=wheel, the broadcaster is disabled and DiffDrive's
    # wheel-odometry TF is bridged from gz instead (see wheel_odo_tf_bridge
    # below). That matches what most ROS stacks do on real robots — the
    # rover mesh drifts as wheel encoders accumulate error.
    rover_tf_is_gt    = PythonExpression(["'", LaunchConfiguration("rover_tf"), "' == 'ground_truth'"])
    rover_tf_is_wheel = PythonExpression(["'", LaunchConfiguration("rover_tf"), "' == 'wheel'"])

    gt_to_path = Node(
        package="explorer_r2_sim",
        executable="gt_to_path.py",
        name="gt_to_path",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "publish_tf":   rover_tf_is_gt,
        }],
    )

    # LIO-SAM static TFs were here, but LIO-SAM is opt-in only via the
    # `lio_sam` compose profile (see docker-compose.yml). If you start
    # the LIO-SAM service, you'll need to re-add these — see the README
    # "LIO-SAM (experimental)" section for the lio_sam/* TF chain and
    # why it currently doesn't visualise cross-container.

    # Bridge DiffDrive's wheel-odo TF (/model/explorer_r2/tf) to /tf so it
    # drives explorer_r2/odom → explorer_r2/base_link. Only launched when
    # rover_tf:=wheel; with the default rover_tf:=ground_truth this bridge
    # is absent and the gt_to_path broadcaster owns base_link instead.
    wheel_odo_tf_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="wheel_odo_tf_bridge",
        arguments=["/model/explorer_r2/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"],
        remappings=[("/model/explorer_r2/tf", "/tf")],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(rover_tf_is_wheel),
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui",     default_value="true"),
        DeclareLaunchArgument("rviz",    default_value="true"),
        DeclareLaunchArgument("teleop",  default_value="false"),
        DeclareLaunchArgument("joy",     default_value="true"),
        DeclareLaunchArgument("rqt_steering", default_value="false"),
        DeclareLaunchArgument(
            "world", default_value="tunnel",
            description=("World to load. Preset short names: "
                         + ", ".join(WORLD_PRESETS.keys())
                         + ". Or a local .sdf path or a Fuel world URL.")),
        DeclareLaunchArgument("verbose", default_value="3"),
        DeclareLaunchArgument(
            "rover_tf", default_value="ground_truth",
            description=("Source of the explorer_r2/odom→base_link TF in "
                         "RViz. 'ground_truth' (default) anchors the rover "
                         "mesh to truth so LIO drift shows as offset arrows. "
                         "'wheel' uses DiffDrive's wheel odometry, like a "
                         "real ROS stack — rover mesh drifts with encoder error.")),

        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            os.path.join(pkg_share, "worlds") + ":" +
            os.path.join(pkg_share, "models") + ":" +
            os.environ.get("GZ_SIM_RESOURCE_PATH", "")),
        SetEnvironmentVariable(
            "IGN_GAZEBO_RESOURCE_PATH",
            os.path.join(pkg_share, "worlds") + ":" +
            os.path.join(pkg_share, "models") + ":" +
            os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")),

        gz_sim_launch,
        parameter_bridge,
        rviz_node,
        teleop_proc,
        joy_node,
        joy_teleop_node,
        rqt_steering,
        gt_to_path,
        wheel_odo_tf_bridge,
    ])
