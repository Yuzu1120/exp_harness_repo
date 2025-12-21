#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2025 Yuzuki Fujita
# SPDX-License-Identifier: BSD-3-Clause

set -eo pipefail

LAUNCH_PID=""

cleanup() {
  if [ -n "${LAUNCH_PID}" ] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "[TEST] cleanup: kill launch ${LAUNCH_PID}"
    kill "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

on_err() {
  echo "[TEST] ERROR: dumping debug info..."
  ros2 node list || true
  ros2 service list || true
  ros2 topic list || true
  echo "[TEST] ---- launch log (tail) ----"
  [ -f /tmp/exp_harness_launch.log ] && tail -n 200 /tmp/exp_harness_launch.log || true
}
trap on_err ERR

source /opt/ros/humble/setup.bash

if [ -f "install/setup.bash" ]; then
  source install/setup.bash
fi

# CI で discovery が詰まることがあるので daemon を再起動
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

echo "[TEST] launch start"
ros2 launch exp_harness demo.launch.py > /tmp/exp_harness_launch.log 2>&1 &
LAUNCH_PID=$!

sleep 2
echo "[TEST] launch log (head)"
head -n 80 /tmp/exp_harness_launch.log || true

# ここから全体タイムアウト（90秒で必ず終わる）
DEADLINE=$((SECONDS + 90))

echo "[TEST] wait service ready"
READY=0
while [ "${SECONDS}" -lt "${DEADLINE}" ]; do
  # /experiment/run が出るか確認（完全一致にすると外すので contains にする）
  if ros2 service list 2>/dev/null | grep -q "/experiment/run"; then
    # type が取れれば ready とみなす
    if ros2 service type /experiment/run >/dev/null 2>&1; then
      READY=1
      break
    fi
  fi

  # 途中経過（5秒ごと）
  if [ $((SECONDS % 5)) -eq 0 ]; then
    echo "[TEST] still waiting... (services containing experiment)"
    ros2 service list 2>/dev/null | grep experiment || true
    tail -n 30 /tmp/exp_harness_launch.log || true
  fi
  sleep 0.5
done

if [ "${READY}" -ne 1 ]; then
  echo "[TEST] ERROR: service not ready (timeout)"
  exit 1
fi

echo "[TEST] service ready"
echo "[TEST] call service (wait for accepted)"

ACCEPTED=0
for i in $(seq 1 8); do
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

if [ "${ACCEPTED}" -ne 1 ]; then
  echo "[TEST] ERROR: service not accepted"
  cat /tmp/exp_harness_call.log || true
  exit 1
fi

echo "[TEST] service accepted"
sleep 2
echo "[TEST] report printed?"
echo "[TEST] SUCCESS"
