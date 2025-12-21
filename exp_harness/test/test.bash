#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2025 Yuzuki Fujita
# SPDX-License-Identifier: BSD-3-Clause

set -euo pipefail

on_err() {
  echo "[TEST] ERROR: dumping debug info..."
  ros2 node list || true
  ros2 service list || true
  ros2 topic list || true
  echo "[TEST] ---- launch log ----"
  [ -f /tmp/exp_harness_launch.log ] && tail -n 200 /tmp/exp_harness_launch.log || true
}
trap on_err ERR

set +u
source /opt/ros/humble/setup.bash
set -u

if [ -f "install/setup.bash" ]; then
  set +u
  source install/setup.bash
  set -u
fi

echo "[TEST] launch start"
ros2 launch exp_harness demo.launch.py > /tmp/exp_harness_launch.log 2>&1 &
LAUNCH_PID=$!

sleep 2
echo "[TEST] launch log (head)"
head -n 50 /tmp/exp_harness_launch.log || true

echo "[TEST] wait service ready"
READY=0
for i in $(seq 1 10); do
  if ros2 service list 2>/dev/null | grep -q "^/experiment/run$"; then
    if ros2 service type /experiment/run >/dev/null 2>&1; then
      READY=1
      break
    fi
  fi
  sleep 0.5
done

if [ "$READY" -ne 1 ]; then
  echo "[TEST] ERROR: service not ready"
  tail -n 200 /tmp/exp_harness_launch.log || true
  exit 1
fi

echo "[TEST] service ready"
echo "[TEST] call service (wait for accepted)"

ACCEPTED=0
for i in $(seq 1 10); do
  echo "[TEST] service call try ${i}"

  ros2 service call /experiment/run exp_harness_interfaces/srv/RunExperiment "{
    experiment_id: 'ci_demo',
    metric_topic: '/metric',
    target_node: '/metric_pub',
    param_name: 'gain',
    a_value: 0.5,
    b_value: 1.5,
    pre_duration_sec: 1.2,
    post_duration_sec: 1.2
  }" > /tmp/exp_harness_call.log 2>&1 || true

  if grep -Eiq 'accepted:\s*(true|True)' /tmp/exp_harness_call.log; then
    ACCEPTED=1
    break
  fi

  tail -n 30 /tmp/exp_harness_call.log || true
  sleep 1
done

if [ "$ACCEPTED" -ne 1 ]; then
  echo "[TEST] ERROR: service not accepted"
  cat /tmp/exp_harness_call.log || true
  exit 1
fi

echo "[TEST] service accepted"
sleep 2
echo "[TEST] report printed?"

echo "[TEST] cleanup"
kill "${LAUNCH_PID}"
wait "${LAUNCH_PID}" 2>/dev/null || true

echo "[TEST] SUCCESS"
