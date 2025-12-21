#!/usr/bin/python3
# SPDX-FileCopyrightText: 2025 Yuzuki Fujita
# SPDX-License-Identifier: BSD-3-Clause

import time
from collections import deque
from typing import Deque, Tuple

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

from exp_harness_interfaces.msg import ExperimentReport
from exp_harness_interfaces.srv import RunExperiment

from .stats_welford import EPS, Welford


class ExperimentServer(Node):
    """
    実験を自動実行するサービスノード。

    RunExperiment サービスを受け取ると、
    1. 対象ノードのパラメータをA に設定
    2. 指定時間メトリクスを計測（事前区間）
    3. 対象ノードのパラメータをB に設定
    4. 指定時間メトリクスを計測（事後区間）
    5. 統計量をExperimentReport としてpublish する
    """

    def __init__(self) -> None:
        super().__init__("experiment_server")

        # パラメータ定義
        self.declare_parameter("metric_topic", "/metric")
        self.declare_parameter("report_topic", "/exp/report")
        self.declare_parameter("buffer_sec", 30.0)
        self.declare_parameter("min_samples", 30)
        self.declare_parameter("default_target_node", "/metric_pub")
        self.declare_parameter("default_param_name", "gain")

        self.metric_topic = str(self.get_parameter("metric_topic").value)
        self.report_topic = str(self.get_parameter("report_topic").value)
        self.buffer_sec = float(self.get_parameter("buffer_sec").value)
        self.min_samples = int(self.get_parameter("min_samples").value)
        self.default_target_node = str(self.get_parameter("default_target_node").value)
        self.default_param_name = str(self.get_parameter("default_param_name").value)

        # メトリクスのリングバッファ（時刻, 値）
        self.metric_buf: Deque[Tuple[float, float]] = deque()
        self.create_subscription(Float32, self.metric_topic, self._on_metric, 100)

        # レポート publish
        self.pub_report = self.create_publisher(ExperimentReport, self.report_topic, 10)

        # 実験実行サービス
        self.create_service(RunExperiment, "/experiment/run", self._on_run)

        self.get_logger().info(
            f"metric_topic={self.metric_topic}, report_topic={self.report_topic}, "
            f"min_samples={self.min_samples}"
        )

    def _on_metric(self, msg: Float32) -> None:
        """メトリクスを受信してバッファに保存する。"""
        now = time.time()
        self.metric_buf.append((now, float(msg.data)))

        # 古いデータを削除
        cutoff = now - self.buffer_sec
        while self.metric_buf and self.metric_buf[0][0] < cutoff:
            self.metric_buf.popleft()

    def _collect_stats(self, t0: float, t1: float) -> Welford:
        """指定時間区間 [t0, t1] のメトリクスから統計量を計算する。"""
        w = Welford()
        for t, v in self.metric_buf:
            if t0 <= t <= t1:
                w.push(v)
        return w

    def _set_param_double(self, target_node: str, param_name: str, value: float) -> bool:
        """対象ノードの double 型パラメータを設定する。"""
        cli = self.create_client(SetParameters, f"{target_node}/set_parameters")
        if not cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"SetParameters が利用できません: {target_node}")
            return False

        p = Parameter()
        p.name = param_name
        p.value = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE,
            double_value=float(value),
        )

        req = SetParameters.Request()
        req.parameters = [p]

        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)

        if not fut.done():
            self.get_logger().error("パラメータ設定がタイムアウトしました")
            return False

        results = fut.result().results
        if not results or not results[0].successful:
            self.get_logger().error("パラメータ設定に失敗しました")
            return False

        return True

    def _on_run(
        self,
        req: RunExperiment.Request,
        res: RunExperiment.Response,
    ) -> RunExperiment.Response:
        """実験実行サービスのコールバック。"""

        exp_id = req.experiment_id if req.experiment_id else "exp"
        metric_topic = req.metric_topic if req.metric_topic else self.metric_topic
        target_node = req.target_node if req.target_node else self.default_target_node
        param_name = req.param_name if req.param_name else self.default_param_name

        a_value = float(req.a_value)
        b_value = float(req.b_value)
        pre_d = float(req.pre_duration_sec)
        post_d = float(req.post_duration_sec)

        if metric_topic != self.metric_topic:
            res.accepted = False
            res.message = f"metric_topic が一致しません（監視中: {self.metric_topic}）"
            return res

        # --- A区間 ---
        if not self._set_param_double(target_node, param_name, a_value):
            res.accepted = False
            res.message = "パラメータA の設定に失敗しました"
            return res

        t_a0 = time.time()
        t_a1 = t_a0 + pre_d
        while time.time() < t_a1:
            time.sleep(0.005)
        pre = self._collect_stats(t_a0, t_a1)

        # --- B区間 ---
        if not self._set_param_double(target_node, param_name, b_value):
            res.accepted = False
            res.message = "パラメータB の設定に失敗しました"
            return res

        t_b0 = time.time()
        t_b1 = t_b0 + post_d
        while time.time() < t_b1:
            time.sleep(0.005)
        post = self._collect_stats(t_b0, t_b1)

        # --- レポート作成 ---
        rep = ExperimentReport()
        rep.stamp = self.get_clock().now().to_msg()

        rep.experiment_id = exp_id
        rep.metric_topic = metric_topic
        rep.target_node = target_node
        rep.param_name = param_name
        rep.a_value = a_value
        rep.b_value = b_value
        rep.pre_duration_sec = pre_d
        rep.post_duration_sec = post_d

        rep.pre_n = int(pre.n)
        rep.pre_mean = float(pre.mean)
        rep.pre_var = float(pre.var())

        rep.post_n = int(post.n)
        rep.post_mean = float(post.mean)
        rep.post_var = float(post.var())

        rep.delta_mean = float(post.mean - pre.mean)
        denom = (pre.var() + EPS) ** 0.5
        rep.score = float(abs(rep.delta_mean) / denom)

        enough = (pre.n >= self.min_samples) and (post.n >= self.min_samples)
        rep.success = bool(enough)
        rep.note = (
            "ok"
            if enough
            else f"サンプル数不足（pre={pre.n}, post={post.n}）"
        )

        self.pub_report.publish(rep)
        self.get_logger().info(
            f"レポート送信: id={rep.experiment_id}, Δmean={rep.delta_mean:.3f}, "
            f"score={rep.score:.2f}, success={rep.success}"
        )

        res.accepted = True
        res.message = "レポートをpublish しました"
        return res


def main() -> None:
    rclpy.init()
    node = ExperimentServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
