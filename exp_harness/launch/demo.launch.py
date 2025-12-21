# SPDX-FileCopyrightText: 2025 Yuzuki Fujita
# SPDX-License-Identifier: BSD-3-Clause

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    metric_pub = Node(
        package="exp_harness",
        executable="metric_pub_node",
        name="metric_pub",
        output="screen",
        parameters=[
            {"topic": "/metric"},
            {"period_sec": 0.02},
            {"gain": 1.0},
            {"bias": 0.0},
            {"noise_std": 0.02},
        ],
    )

    server = Node(
        package="exp_harness",
        executable="experiment_server_node",
        name="experiment_server",
        output="screen",
        parameters=[
            {"metric_topic": "/metric"},
            {"report_topic": "/exp/report"},
            {"buffer_sec": 30.0},
            {"min_samples": 30},
            {"default_target_node": "/metric_pub"},
            {"default_param_name": "gain"},
        ],
    )

    printer = Node(
        package="exp_harness",
        executable="report_printer_node",
        name="report_printer",
        output="screen",
        parameters=[{"report_topic": "/exp/report"}],
    )

    return LaunchDescription([metric_pub, server, printer])
