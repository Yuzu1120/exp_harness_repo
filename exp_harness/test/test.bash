#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2025 Yuzuki Fujita
# SPDX-License-Identifier: BSD-3-Clause

set -euo pipefail

export ROS2CLI_DISABLE_DAEMON=1

LAUNCH_PID=""

on_err() {
  echo "[TEST] ERROR: dumping debug info..."
  ros2 node list || true
  ros2 service list || true
  ros2 topic list || true

  echo "[TEST] ---- launch log (tail) ----"
  [ -f /tmp/exp_harness_launch.log ] && tail -n 200 /tmp/exp_harness_launch.log || true

  echo "[TEST] ---- call log (tail) ----"
  [ -f /tmp/exp_harness_call.log ] && tail -n 200 /tmp/exp_harness_call.log || true

  echo "[TEST] ---- report log (tail) ----"
  [ -f /tmp/exp_harness_report.log ] && tail -n 200 /tmp/exp_harness_report.log || true
}

cleanup() {
  echo "[TEST] cleanup"

  pkill -f "ros2 launch exp_harness demo.launch.py" 2>/dev/null || true
  pkill -f "experiment_server_node" 2>/dev/null || true
  pkill -f "metric_pub_node" 2>/dev/null || true
  pkill -f "report_printer_node" 2>/dev/null || true
  sleep 1

  # 今回起動した launch を落とす
  if [ -n "${LAUNCH_PID}" ] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill "${LAUNCH_PID}" 2>/dev/null || true
    sleep 1
    kill -9 "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap on_err ERR

# --- env ---
set +u; source /opt/ros/humble/setup.bash; set -u
if [ -f "install/setup.bash" ]; then
  set +u; source install/setup.bash; set -u
fi

echo "[TEST] cleanup before launch"
# 念のため（EXITのcleanupが動かないケース対策）
pkill -f "ros2 launch exp_harness demo.launch.py" 2>/dev/null || true
pkill -f "experiment_server_node" 2>/dev/null || true
pkill -f "metric_pub_node" 2>/dev/null || true
pkill -f "report_printer_node" 2>/dev/null || true
sleep 1

echo "[TEST] launch start"
ros2 launch exp_harness demo.launch.py > /tmp/exp_harness_launch.log 2>&1 &
LAUNCH_PID=$!

echo "[TEST] wait for nodes/services (<=20s)"
for i in $(seq 1 20); do
  if ros2 node list 2>/dev/null | grep -q "^/experiment_server$" \
    && ros2 node list 2>/dev/null | grep -q "^/metric_pub$" \
    && ros2 service list 2>/dev/null | grep -q "^/experiment/run$"; then
    break
  fi
  sleep 1
done

ros2 node list | grep -q "^/experiment_server$"
ros2 node list | grep -q "^/metric_pub$"
ros2 service list | grep -q "^/experiment/run$"

echo "[TEST] topic sanity (list only)"
ros2 topic list | grep -q "^/metric$"
ros2 topic list | grep -q "^/exp/report$"

# 受信取り逃がし防止：echo を先に待機させる
echo "[TEST] start report receiver BEFORE service call"
rm -f /tmp/exp_harness_report.log
timeout 8 ros2 topic echo /exp/report --once > /tmp/exp_harness_report.log 2>&1 &
ECHO_PID=$!

echo "[TEST] call service (timeout + accepted check)"
timeout 10 ros2 service call /experiment/run exp_harness_interfaces/srv/RunExperiment "{
  experiment_id: 'ci_demo',
  metric_topic: '/metric',
  target_node: '/metric_pub',
  param_name: 'gain',
  a_value: 0.5,
  b_value: 1.5,
  pre_duration_sec: 0.7,
  post_duration_sec: 0.7
}" > /tmp/exp_harness_call.log 2>&1

# accepted 判定
if grep -qi 'accepted=True' /tmp/exp_harness_call.log || grep -Eqi 'accepted:[[:space:]]*true' /tmp/exp_harness_call.log; then
  echo "[TEST] accepted OK"
else
  echo "[TEST] ERROR: service not accepted"
  cat /tmp/exp_harness_call.log || true
  exit 1
fi


# 受信できたかチェック
if ! grep -Eq '^(experiment_id|success|note):' /tmp/exp_harness_report.log; then
  echo "[TEST] ERROR: report not received"
  echo "[TEST] ---- report log (tail) ----"
  tail -n 200 /tmp/exp_harness_report.log || true
  exit 1
fi

if ! grep -q '^experiment_id: ci_demo' /tmp/exp_harness_report.log; then
  echo "[TEST] ERROR: report id mismatch"
  echo "[TEST] ---- report log (tail) ----"
  tail -n 200 /tmp/exp_harness_report.log || true
  exit 1
fi

echo "[TEST] PASS"

