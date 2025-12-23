#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2025 Yuzuki Fujita
# SPDX-License-Identifier: BSD-3-Clause1

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
}

cleanup() {
  echo "[TEST] cleanup"
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

echo "[TEST] launch start"
ros2 launch exp_harness demo.launch.py > /tmp/exp_harness_launch.log 2>&1 &
LAUNCH_PID=$!

echo "[TEST] wait for nodes/services"
# 最大20秒だけ待つ（無限待ち禁止）
for i in $(seq 1 20); do
  # nodeが見えて、serviceが見えればOK
  if ros2 node list 2>/dev/null | grep -q "/experiment_server" \
    && ros2 service list 2>/dev/null | grep -q "^/experiment/run$"; then
    break
  fi
  sleep 1
done

# ここで見えなかったら落とす（無限に進まない）
ros2 node list | grep -q "/experiment_server"
ros2 service list | grep -q "^/experiment/run$"

echo "[TEST] topic sanity (list only)"
# echoは重い/詰まりやすいので「存在確認だけ」にする（軽くて確実に終わる）
ros2 topic list | grep -q "^/metric$"
ros2 topic list | grep -q "^/exp/report$"

echo "[TEST] call service (timeout + accepted check)"
# ここが “一生返らない” の主犯になりやすいので timeout 必須
timeout 10 ros2 service call /experiment/run exp_harness_interfaces/srv/RunExperiment "{
  experiment_id: 'ci_demo',
  metric_topic: '/metric',
  target_node: '/metric_pub',
  param_name: 'gain',
  a_value: 0.5,
  b_value: 1.5,
  pre_duration_sec: 0.2,
  post_duration_sec: 0.2
}" > /tmp/exp_harness_call.log 2>&1

# accepted の判定（true/True 両対応）
grep -Eiq 'accepted:\s*(true|True)' /tmp/exp_harness_call.log

echo "[TEST] PASS"


