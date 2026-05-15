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
    gt_to_path = Node(
        package="explorer_r2_sim",
        executable="gt_to_path.py",
        name="gt_to_path",
        output="screen",
        parameters=[{"use_sim_time": True}],
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
    ])
