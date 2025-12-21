#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2025 Yuzuki Fujita
# SPDX-License-Identifier: BSD-3-Clause

set -e
set -o pipefail

source /opt/ros/humble/setup.bash

if [ -f "install/setup.bash" ]; then
  source install/setup.bash
fi

echo "[TEST] build result check"

# パッケージが見えるか
ros2 pkg list | grep -q exp_harness
ros2 pkg list | grep -q exp_harness_interfaces

echo "[TEST] interface generation check"

# 独自 msg / srv が生成されているか
ros2 interface show exp_harness_interfaces/msg/ExperimentReport >/dev/null
ros2 interface show exp_harness_interfaces/srv/RunExperiment >/dev/null

echo "[TEST] executable check"

# ノードの実行ファイルが登録されているか
ros2 pkg executables exp_harness | grep -q metric_pub_node
ros2 pkg executables exp_harness | grep -q experiment_server_node
ros2 pkg executables exp_harness | grep -q report_printer_node

echo "[TEST] node startup check (short)"

# ノードが起動できるか（即kill）
ros2 run exp_harness metric_pub_node &
PID=$!
sleep 1
kill "$PID" 2>/dev/null || true
wait "$PID" 2>/dev/null || true

echo "[TEST] SUCCESS"
