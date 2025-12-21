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

PKGS="$(ros2 pkg list)"
case "$PKGS" in
  *"exp_harness"*) ;;
  *) echo "[TEST] ERROR: exp_harness not found in ros2 pkg list"; exit 1 ;;
esac
case "$PKGS" in
  *"exp_harness_interfaces"*) ;;
  *) echo "[TEST] ERROR: exp_harness_interfaces not found in ros2 pkg list"; exit 1 ;;
esac

echo "[TEST] interface generation check"
ros2 interface show exp_harness_interfaces/msg/ExperimentReport >/dev/null
ros2 interface show exp_harness_interfaces/srv/RunExperiment >/dev/null

echo "[TEST] executable check"
EXES="$(ros2 pkg executables exp_harness)"
case "$EXES" in
  *"metric_pub_node"*) ;;
  *) echo "[TEST] ERROR: metric_pub_node not registered"; exit 1 ;;
esac
case "$EXES" in
  *"experiment_server_node"*) ;;
  *) echo "[TEST] ERROR: experiment_server_node not registered"; exit 1 ;;
esac
case "$EXES" in
  *"report_printer_node"*) ;;
  *) echo "[TEST] ERROR: report_printer_node not registered"; exit 1 ;;
esac

echo "[TEST] node startup check (short)"
ros2 run exp_harness metric_pub_node >/tmp/metric_pub_node.log 2>&1 &
PID=$!
sleep 1
kill "$PID" 2>/dev/null || true
wait "$PID" 2>/dev/null || true

echo "[TEST] SUCCESS"

