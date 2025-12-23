#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2025 Yuzuki Fujita
# SPDX-License-Identifier: BSD-3-Clause1

set -euo pipefail

LOG_LAUNCH="/tmp/exp_harness_launch.log"
LOG_CALL="/tmp/exp_harness_call.log"
LAUNCH_PID=""

cleanup() {
  echo "[TEST] cleanup"
  if [ -n "${LAUNCH_PID}" ] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill "${LAUNCH_PID}" 2>/dev/null || true
    # しつこいlaunchでも終わるように保険
    sleep 0.5
    kill -9 "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

on_err() {
  echo "[TEST] ERROR: dumping debug info..."
  ros2 node list || true
  ros2 service list || true
  ros2 topic list || true
  echo "[TEST] ---- launch log (tail) ----"
  [ -f "${LOG_LAUNCH}" ] && tail -n 200 "${LOG_LAUNCH}" || true
  echo "[TEST] ---- call log ----"
  [ -f "${LOG_CALL}" ] && tail -n 200 "${LOG_CALL}" || true
}
trap on_err ERR

# bash -u で setup.bash がコケることがあるので、source周りだけ緩める
set +u
source /opt/ros/humble/setup.bash
set -u

# ワークスペース直下で呼ばれる/呼ばれない両方に対応
if [ -f "install/setup.bash" ]; then
  set +u
  source install/setup.bash
  set -u
fi

echo "[TEST] build result check"
# パイプで BrokenPipe を踏みやすいので、あえて pipe しない
ros2 pkg prefix exp_harness >/dev/null
ros2 pkg prefix exp_harness_interfaces >/dev/null

echo "[TEST] launch start"
: > "${LOG_LAUNCH}"
ros2 launch exp_harness demo.launch.py > "${LOG_LAUNCH}" 2>&1 &
LAUNCH_PID=$!

echo "[TEST] wait for nodes/services"
# ノード・サービスが立ち上がるまで待つ（CI保険）
timeout 30 bash -c '
  until ros2 node list 2>/dev/null | grep -q "/metric_pub"; do sleep 0.2; done
  until ros2 service list 2>/dev/null | grep -q "/experiment/run"; do sleep 0.2; done
'

echo "[TEST] topic sanity (receive 1 msg)"
# トピック通信してる証拠：1件だけ受信できればOK（重くない）
timeout 10 ros2 topic echo -n 1 /metric >/dev/null

echo "[TEST] call service (wait for accepted)"
ACCEPTED=0
for i in $(seq 1 12); do
  echo "[TEST] service call try ${i}"
  ros2 service call /experiment/run exp_harness_interfaces/srv/RunExperiment "{
    experiment_id: 'ci_demo',
    metric_topic: '/metric',
    target_node: '/metric_pub',
    param_name: 'gain',
    a_value: 0.5,
    b_value: 1.5,
    pre_duration_sec: 0.4,
    post_duration_sec: 0.4
  }" > "${LOG_CALL}" 2>&1 || true

  if grep -Eiq "accepted:\s*(true|True)" "${LOG_CALL}"; then
    ACCEPTED=1
    break
  fi

  # 失敗時に短くログ表示（CIで助かる）
  tail -n 30 "${LOG_CALL}" || true
  sleep 0.5
done

if [ "${ACCEPTED}" -ne 1 ]; then
  echo "[TEST] ERROR: service not accepted"
  exit 1
fi
echo "[TEST] service accepted"

# レポート発行までちょい待つ（短く）
sleep 1.0

echo "[TEST] done"

