#!/usr/bin/python3
# SPDX-FileCopyrightText: 2025 Yuzuki Fujita
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
from rclpy.node import Node

from exp_harness_interfaces.msg import ExperimentReport


class ReportPrinter(Node):
    """ExperimentReport を受信して読みやすく表示するノード。"""

    def __init__(self) -> None:
        super().__init__("report_printer")

        self.declare_parameter("report_topic", "/exp/report")
        topic = str(self.get_parameter("report_topic").value)

        self.create_subscription(ExperimentReport, topic, self._cb, 10)
        self.get_logger().info(f"レポート購読開始: {topic}")

    def _cb(self, r: ExperimentReport) -> None:
        """レポートをログとして出力する。"""
        self.get_logger().info(
            f"実験ID={r.experiment_id}, パラメータ={r.param_name}, "
            f"A={r.a_value}, B={r.b_value}, "
            f"平均(pre)={r.pre_mean:.3f}, 平均(post)={r.post_mean:.3f}, "
            f"Δ={r.delta_mean:.3f}, score={r.score:.2f}, "
            f"成功={r.success}, note={r.note}"
        )


def main() -> None:
    rclpy.init()
    node = ReportPrinter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
