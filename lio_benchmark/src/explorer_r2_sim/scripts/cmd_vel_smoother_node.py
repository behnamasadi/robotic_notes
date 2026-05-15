#!/usr/bin/env python3
"""cmd_vel_smoother_node.py — bounded-acceleration ramp on /cmd_vel.

Background: gz-sim-diff-drive sets target wheel velocities directly
from /cmd_vel. When a step input arrives (forward → reverse, straight
→ arc, etc.) the wheel velocities jump in one timestep and the
physics solver generates impulsive ground-contact forces — which
gz-sim's bandwidth-unlimited IMU sensor faithfully reports as
single-sample acceleration spikes.

This node sits between the command source and the bridge:

    scripted_drive.py  ─►  /cmd_vel_in   ─►  this node  ─►  /cmd_vel  ─►  bridge ─► wheels

and ramps the published /cmd_vel toward the most-recent input with
bounded linear and angular acceleration. The wheel-velocity command
into gz-sim then changes smoothly over many steps, replacing the
impulse with a smooth acceleration profile.

Defaults (max_linear_accel=0.5 m/s², max_angular_accel=1.0 rad/s²)
roughly match a real DC-motor-driven wheeled rover with a non-trivial
gearbox — going 0→0.5 m/s takes ~1 second, which is normal.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def _ramp(current, target, max_step):
    """Move 'current' toward 'target' by at most ±max_step."""
    delta = target - current
    if abs(delta) <= max_step:
        return target
    return current + (max_step if delta > 0 else -max_step)


class CmdVelSmoother(Node):
    def __init__(self):
        super().__init__("cmd_vel_smoother")
        self.declare_parameter("max_linear_accel", 0.5)   # m/s²
        self.declare_parameter("max_angular_accel", 1.0)  # rad/s²
        self.declare_parameter("output_rate_hz", 50.0)
        self.declare_parameter("input_topic", "/cmd_vel_in")
        self.declare_parameter("output_topic", "/cmd_vel")

        self.max_lin = float(self.get_parameter("max_linear_accel").value)
        self.max_ang = float(self.get_parameter("max_angular_accel").value)
        rate = float(self.get_parameter("output_rate_hz").value)
        self.dt = 1.0 / rate

        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value

        self.target_lin = 0.0
        self.target_ang = 0.0
        self.current_lin = 0.0
        self.current_ang = 0.0

        self.sub = self.create_subscription(Twist, in_topic, self.on_cmd, 10)
        self.pub = self.create_publisher(Twist, out_topic, 10)
        self.create_timer(self.dt, self.publish_step)

        self.get_logger().info(
            f"cmd_vel smoother: {in_topic} -> {out_topic}  "
            f"max_a={self.max_lin:.2f} m/s²  max_α={self.max_ang:.2f} rad/s²  "
            f"@ {rate:.0f} Hz"
        )

    def on_cmd(self, msg):
        self.target_lin = msg.linear.x
        self.target_ang = msg.angular.z

    def publish_step(self):
        self.current_lin = _ramp(self.current_lin, self.target_lin,
                                 self.max_lin * self.dt)
        self.current_ang = _ramp(self.current_ang, self.target_ang,
                                 self.max_ang * self.dt)
        out = Twist()
        out.linear.x = self.current_lin
        out.angular.z = self.current_ang
        self.pub.publish(out)


def main():
    rclpy.init()
    node = CmdVelSmoother()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
