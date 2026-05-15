# Spawn EXPLORER_R2 into a running gz sim. Mode 2 step 2.
#
# Polls the gz spawn service until gz is up, so it's race-free —
# works whether you run it right after gz starts or much later.
#
# Launch arguments:
#   world:=<preset>   Used only to look up the default SPAWN_POSES entry.
#                     Defaults to "tunnel". If your world isn't in the table,
#                     pass spawn_x/y/z/yaw explicitly.
#   spawn_x / spawn_y / spawn_z / spawn_yaw
#                     Override the per-world default spawn pose. Use a
#                     literal float; "auto" (the default) means use
#                     SPAWN_POSES[<world>].

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Per-world default spawn pose: (x, y, z, yaw). z=0.4 keeps the rover's
# wheels just above the ground. Unknown worlds fall back to SPAWN_FALLBACK.
SPAWN_POSES: "dict[str, tuple[float, float, float, float]]" = {
    "tunnel":          (10.0, 0.0, 0.4, 0.0),
    "cave":            ( 2.0, 0.0, 0.4, 0.0),
    "rubicon":         ( 0.0, 0.0, 0.5, 0.0),
    "tugbot_depot":    ( 0.0, 0.0, 0.5, 0.0),
    "singapore_river": ( 0.0, 0.0, 0.5, 0.0),
}
SPAWN_FALLBACK = (0.0, 0.0, 0.4, 0.0)


def generate_launch_description():
    pkg_share = get_package_share_directory("explorer_r2_sim")
    model_sdf = os.path.join(pkg_share, "models", "explorer_r2", "model.sdf")

    def _coalesce(context, arg_name, default_value):
        v = LaunchConfiguration(arg_name).perform(context)
        return v if v != "auto" else str(default_value)

    def _make_spawn(context, *_args, **_kwargs):
        world_arg = LaunchConfiguration("world").perform(context)
        defaults = SPAWN_POSES.get(world_arg, SPAWN_FALLBACK)
        sx = _coalesce(context, "spawn_x",   defaults[0])
        sy = _coalesce(context, "spawn_y",   defaults[1])
        sz = _coalesce(context, "spawn_z",   defaults[2])
        syaw = _coalesce(context, "spawn_yaw", defaults[3])
        return [Node(
            package="ros_gz_sim",
            executable="create",
            name="spawn_explorer_r2",
            arguments=[
                "-name", "explorer_r2",
                "-file", model_sdf,
                "-x", sx, "-y", sy, "-z", sz, "-Y", syaw,
            ],
            output="screen",
        )]

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="tunnel",
                              description="Only used to pick a SPAWN_POSES default"),
        DeclareLaunchArgument("spawn_x",   default_value="auto"),
        DeclareLaunchArgument("spawn_y",   default_value="auto"),
        DeclareLaunchArgument("spawn_z",   default_value="auto"),
        DeclareLaunchArgument("spawn_yaw", default_value="auto"),
        OpaqueFunction(function=_make_spawn),
    ])
