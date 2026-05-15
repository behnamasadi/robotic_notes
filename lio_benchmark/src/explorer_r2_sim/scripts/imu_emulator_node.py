#!/usr/bin/env python3
"""imu_emulator_node.py — realistic MEMS IMU emulator on top of gz-sim's stream.

Why this exists
---------------
gz-sim's built-in `imu_sensor` reports the rigid body's instantaneous
acceleration + angular velocity from the physics solver, plus Gaussian
noise + bias drift configured in the SDF. That's it. It has **no
bandwidth model, no saturation, no quantization** — none of the
physical signal-processing that any real MEMS IMU does between its
sense element and its output register.

The consequence (verified in this project's 2026-05-15 debug session):
when gz-sim-diff-drive steps wheel velocities to track a new /cmd_vel,
the resulting impulsive ground-contact forces appear in the IMU stream
as single-sample 50–200 m/s² (5–20 g) spikes. A real BMI088 has a
145 Hz mechanical bandwidth plus an anti-alias filter and full-scale
clipping at ±16 g, so it could never produce that signal. VIO
estimators tuned against real IMU data (EuRoC, TUM-VIO) interpret the
spikes as motion-onset jerks and explode by 3-10× in scale.

Upstream is aware (gazebosim/gz-sensors#490, Dec 2024, no PR yet). This
node implements the equivalent processing downstream so consumers see
a believable IMU stream.

What this node models
---------------------
Per-channel, per sample:

    raw  →  saturate to ±range  →  4-pole Butterworth lowpass  →  quantize

- **Bandwidth**: 4-pole Butterworth lowpass at 50 Hz cutoff. We're
  given a 250 Hz raw rate from gz-sim; the BMI088 datasheet uses a
  ratio of (sample-rate)/(3 dB cutoff) ≈ 2.5 for its built-in
  filters, which would put us at 100 Hz. We use 50 Hz instead because
  it more reliably attenuates the broadband impulse content gz-sim
  produces — the impulses span DC to Nyquist, and pushing the cutoff
  lower buys more rejection at the cost of a few ms of group delay
  (still well below the camera frame interval at 30 Hz).
- **Saturation**: ±16 g on accel, ±2000 °/s on gyro. BMI088 default
  full-scale range.
- **Quantization**: 16-bit per channel. Real MEMS ADC depth.

The SOS (Second-Order Sections) coefficients for the Butterworth are
precomputed on the host with `scipy.signal.butter(4, 50/125, 'low',
output='sos')` so this script has **no scipy dependency at runtime**.
Verified frequency response:
    0 Hz   →   0 dB     (DC passes through)
    25 Hz  →  -0.01 dB  (clean passband)
    50 Hz  →  -3 dB     (the design cutoff)
    75 Hz  →  -22 dB    (strongly attenuated)
    100 Hz →  -50 dB    (essentially gone)

References
----------
- gazebosim/gz-sensors#490 — upstream-proposed IMU lowpass parameter
- BMI088 datasheet section 4.4 — bandwidth/ODR table
- rotors_simulator imu_plugin (ETH ASL) — the realistic IMU model
  used to generate the EuRoC MAV dataset, structurally the design
  this node mirrors (Gazebo Classic, can't be dropped into gz-sim
  directly)
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


# Precomputed SOS coefficients for a 4-pole Butterworth lowpass at
# cutoff_hz=50, sample_rate_hz=250. Each row is [b0, b1, b2, a0, a1, a2]
# with a0 = 1. Designed with scipy.signal.butter; embedding the constants
# here removes scipy as a runtime dependency.
BUTTER_50HZ_4POLE_SOS = [
    [0.0465829066, 0.0931658133, 0.0465829066, 1.0, -0.3289756774, 0.0645876549],
    [1.0,          2.0,          1.0,          1.0, -0.4531195207, 0.4663255708],
]


class Biquad:
    """One Second-Order Section, Direct Form II Transposed.

    DF2T is numerically stable for the modest pole magnitudes we have
    here (poles inside the unit circle, well away from the boundary).
    """
    __slots__ = ("b0", "b1", "b2", "a1", "a2", "s1", "s2")

    def __init__(self, b0, b1, b2, a0, a1, a2):
        if abs(a0 - 1.0) > 1e-9:
            raise ValueError("biquad a0 must be 1 (SOS rows are already normalised)")
        self.b0 = b0
        self.b1 = b1
        self.b2 = b2
        self.a1 = a1
        self.a2 = a2
        self.s1 = 0.0
        self.s2 = 0.0

    def step(self, x):
        y = self.b0 * x + self.s1
        self.s1 = self.b1 * x + self.s2 - self.a1 * y
        self.s2 = self.b2 * x - self.a2 * y
        return y


class SOSFilter:
    """Cascade of biquads, evaluated low-section-first like scipy.signal.sosfilt."""
    __slots__ = ("biquads",)

    def __init__(self, sos):
        self.biquads = [Biquad(*row) for row in sos]

    def step(self, x):
        for bq in self.biquads:
            x = bq.step(x)
        return x


class IMUEmulator(Node):
    def __init__(self):
        super().__init__("imu_emulator")
        # All defaults match a BMI088 in its mid-range configuration.
        self.declare_parameter("accel_range_g", 16.0)
        self.declare_parameter("gyro_range_dps", 2000.0)
        self.declare_parameter("accel_bits", 16)
        self.declare_parameter("gyro_bits", 16)
        self.declare_parameter("input_topic", "/imu")
        self.declare_parameter("output_topic", "/imu_filtered")

        # Saturation thresholds (full-scale range, m/s² and rad/s).
        self.accel_max = float(self.get_parameter("accel_range_g").value) * 9.81
        self.gyro_max = float(self.get_parameter("gyro_range_dps").value) * math.pi / 180.0

        # Quantization step (LSB size).
        a_bits = int(self.get_parameter("accel_bits").value)
        g_bits = int(self.get_parameter("gyro_bits").value)
        self.accel_lsb = 2.0 * self.accel_max / (2 ** a_bits)
        self.gyro_lsb = 2.0 * self.gyro_max / (2 ** g_bits)

        # One independent filter per channel — biquad state is per-channel,
        # not per-axis-type, so don't share filter objects across channels.
        sos = BUTTER_50HZ_4POLE_SOS
        self.f_ax = SOSFilter(sos); self.f_ay = SOSFilter(sos); self.f_az = SOSFilter(sos)
        self.f_gx = SOSFilter(sos); self.f_gy = SOSFilter(sos); self.f_gz = SOSFilter(sos)

        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value
        self.sub = self.create_subscription(Imu, in_topic, self.cb, 200)
        self.pub = self.create_publisher(Imu, out_topic, 200)

        self.n_samples = 0
        self.n_clipped = 0
        self.create_timer(5.0, self.report)

        self.get_logger().info(
            f"IMU emulator: {in_topic} -> {out_topic}\n"
            f"  4-pole Butterworth lowpass, cutoff 50 Hz @ 250 Hz raw rate\n"
            f"  range: ±{self.accel_max:.1f} m/s² / ±{self.gyro_max:.2f} rad/s\n"
            f"  resolution: {self.accel_lsb * 1000:.3f} m·g LSB / "
            f"{self.gyro_lsb * 180 / math.pi * 3600:.3f} mdps LSB"
        )

    @staticmethod
    def _saturate(x, limit):
        if x > limit:
            return limit, True
        if x < -limit:
            return -limit, True
        return x, False

    def _process(self, x, limit, sos_filter, lsb):
        x_sat, clipped = self._saturate(x, limit)
        y = sos_filter.step(x_sat)
        y_q = round(y / lsb) * lsb
        return y_q, clipped

    def cb(self, msg):
        ax, c1 = self._process(msg.linear_acceleration.x, self.accel_max, self.f_ax, self.accel_lsb)
        ay, c2 = self._process(msg.linear_acceleration.y, self.accel_max, self.f_ay, self.accel_lsb)
        az, c3 = self._process(msg.linear_acceleration.z, self.accel_max, self.f_az, self.accel_lsb)
        gx, c4 = self._process(msg.angular_velocity.x, self.gyro_max, self.f_gx, self.gyro_lsb)
        gy, c5 = self._process(msg.angular_velocity.y, self.gyro_max, self.f_gy, self.gyro_lsb)
        gz, c6 = self._process(msg.angular_velocity.z, self.gyro_max, self.f_gz, self.gyro_lsb)

        if c1 or c2 or c3 or c4 or c5 or c6:
            self.n_clipped += 1
        self.n_samples += 1

        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.linear_acceleration.x = ax
        out.linear_acceleration.y = ay
        out.linear_acceleration.z = az
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        out.angular_velocity.x = gx
        out.angular_velocity.y = gy
        out.angular_velocity.z = gz
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        self.pub.publish(out)

    def report(self):
        if self.n_samples > 0:
            pct = 100.0 * self.n_clipped / self.n_samples
            self.get_logger().info(
                f"{self.n_samples} samples / {self.n_clipped} clipped ({pct:.2f} %)"
            )


def main():
    rclpy.init()
    node = IMUEmulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
