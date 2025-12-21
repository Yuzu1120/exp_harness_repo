#!/usr/bin/python3
# SPDX-FileCopyrightText: 2025 Yuzuki Fujita
# SPDX-License-Identifier: BSD-3-Clause

import math
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class MetricPub(Node):
    """ノイズ付きの正弦波メトリクスをpublishするノード。"""

    def __init__(self) -> None:
        super().__init__("metric_pub")

        self.declare_parameter("topic", "/metric")
        self.declare_parameter("period_sec", 0.02)
        self.declare_parameter("gain", 1.0)
        self.declare_parameter("bias", 0.0)
        self.declare_parameter("noise_std", 0.02)

        self.topic = str(self.get_parameter("topic").value)
        self.period = float(self.get_parameter("period_sec").value)

        self.pub = self.create_publisher(Float32, self.topic, 10)

        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.timer = self.create_timer(self.period, self._tick)

        self.get_logger().info(f"Publishing metric on {self.topic} (period={self.period}s)")

    def _tick(self) -> None:
        gain = float(self.get_parameter("gain").value)
        bias = float(self.get_parameter("bias").value)
        noise_std = float(self.get_parameter("noise_std").value)

        t = self.get_clock().now().nanoseconds * 1e-9 - self.t0
        value = gain * math.sin(t) + bias + random.gauss(0.0, noise_std)

        msg = Float32()
        msg.data = float(value)
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = MetricPub()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
