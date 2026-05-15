#!/usr/bin/env python3
"""scripted_drive.py — drive the rover with a deterministic open-loop velocity
profile, for VIO control experiments.

Why open-loop: human teleop varies trial-to-trial. To compare VIO behaviour
across runs (fast vs slow, calibration A vs B), we want the *exact same*
motion. This script publishes /cmd_vel at 10 Hz from a hard-coded schedule.

The schedule is tuned to:
  - Let OpenVINS' 8 s startup + 5 s init window cover stationary samples.
  - Stay slow enough that KLT can track features between frames at the
    sim's 26 Hz camera rate. Max linear speed 0.5 m/s = ~2 cm per frame.
  - Mix forward, turning, ZUPT pauses, and a brief reverse to give stereo
    triangulation excellent parallax for scale lock-in.

Usage (inside the sim container, sim already up):
    ros2 run rclpy ...
    OR
    python3 src/explorer_r2_sim/scripts/scripted_drive.py
"""

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# (start_t_sec, linear_x_mps, angular_z_radps, label)
# Phases play in order; total runtime is the last phase end.
SCHEDULE = [
    (0.0,   0.0,   0.0,  "wait for VIO init"),         # 0–10
    (10.0,  0.0,   0.0,  "ZUPT — stay still"),         # 10–20
    (20.0,  0.5,   0.0,  "forward 0.5 m/s straight"),  # 20–35  (covers 7.5 m)
    (35.0,  0.0,   0.0,  "ZUPT"),                      # 35–37
    (37.0,  0.4,  +0.20, "forward + gentle right"),    # 37–47
    (47.0,  0.0,   0.0,  "ZUPT"),                      # 47–49
    (49.0,  0.4,  -0.20, "forward + gentle left"),     # 49–59
    (59.0,  0.0,   0.0,  "ZUPT"),                      # 59–61
    (61.0, -0.2,   0.0,  "reverse 0.2 m/s"),           # 61–66  parallax gold
    (66.0,  0.0,   0.0,  "ZUPT"),                      # 66–68
    (68.0,  0.3,  +0.10, "forward + slow yaw"),        # 68–80
    (80.0,  0.0,   0.0,  "stop"),                      # 80–82
]
END_T = 82.0
PUBLISH_RATE_HZ = 10.0


def main():
    rclpy.init()
    node = Node("scripted_drive")
    # Publish to /cmd_vel_in (smoother input), NOT /cmd_vel. The
    # cmd_vel_smoother_node ramps step commands into a smooth velocity
    # profile so gz-sim-diff-drive doesn't step the wheel velocities and
    # produce impulsive ground-contact forces. The smoother forwards to
    # /cmd_vel which the bridge picks up as before.
    pub = node.create_publisher(Twist, "/cmd_vel_in", 10)
    period = 1.0 / PUBLISH_RATE_HZ

    t0 = time.monotonic()
    last_label = ""
    node.get_logger().info(f"Starting {END_T:.0f} s scripted drive")

    while rclpy.ok():
        t = time.monotonic() - t0
        if t > END_T:
            break

        phase = SCHEDULE[0]
        for p in SCHEDULE:
            if t >= p[0]:
                phase = p
        _, lin, ang, label = phase
        if label != last_label:
            node.get_logger().info(f"[t={t:6.1f}s] {label}  v={lin:+.2f} w={ang:+.2f}")
            last_label = label

        msg = Twist()
        msg.linear.x = float(lin)
        msg.angular.z = float(ang)
        pub.publish(msg)
        time.sleep(period)

    stop = Twist()
    pub.publish(stop)
    time.sleep(0.2)
    pub.publish(stop)
    node.get_logger().info("scripted drive complete")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
