#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2025 Yuzuki Fujita
# SPDX-License-Identifier: BSD-3-Clause

set -euo pipefail

source /opt/ros/humble/setup.bash

if [ -f "install/setup.bash" ]; then
  source install/setup.bash
fi

echo "[TEST] launch start"
ros2 launch exp_harness demo.launch.py > /tmp/exp_harness_launch.log 2>&1 &
LAUNCH_PID=$!

cleanup() {
  echo "[TEST] cleanup"
  kill ${LAUNCH_PID} >/dev/null 2>&1 || true
}
trap cleanup EXIT

# ノード起動待ち
sleep 2

echo "[TEST] service exists?"
ros2 service list | grep -q "/experiment/run"

echo "[TEST] call service"
ros2 service call /experiment/run exp_harness_interfaces/srv/RunExperiment "{
  experiment_id: 'ci_demo',
  metric_topic: '/metric',
  target_node: '/metric_pub',
  param_name: 'gain',
  a_value: 0.5,
  b_value: 1.5,
  pre_duration_sec: 1.2,
  post_duration_sec: 1.2
}" > /tmp/exp_harness_call.log 2>&1

echo "[TEST] service response accepted?"
grep -q "accepted: True" /tmp/exp_harness_call.log

# レポートが出るまで待つ
sleep 2

echo "[TEST] report printed?"
grep -q "実験ID=ci_demo" /tmp/exp_harness_launch.log

echo "[TEST] score printed?"
grep -q "score=" /tmp/exp_harness_launch.log

echo "[TEST] PASS"
