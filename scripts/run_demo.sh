#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-correct}"
case "$MODE" in
  correct) CODEWORD="OPEN123" ;;
  wrong)   CODEWORD="WRONG123" ;;
  *)
    echo "Usage: $0 {correct|wrong}"
    exit 2
    ;;
esac

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAP_FILE="$ROOT_DIR/maps/synthetic_warehouse.yaml"
RVIZ_FILE="$ROOT_DIR/rviz/wachroboter_sim.rviz"
MISSION_FILE="$ROOT_DIR/wachroboter/config/mission.yaml"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy was not found at /opt/ros/jazzy/setup.bash"
  exit 1
fi

source /opt/ros/jazzy/setup.bash

# If the repository is cloned under <workspace>/src/wachroboter-ros2,
# automatically source that workspace after it has been built.
WORKSPACE_SETUP="$ROOT_DIR/../../install/setup.bash"
if [[ -f "$WORKSPACE_SETUP" ]]; then
  source "$WORKSPACE_SETUP"
elif [[ -f "$HOME/ros2_ws/install/setup.bash" ]]; then
  source "$HOME/ros2_ws/install/setup.bash"
fi

if ! ros2 pkg prefix wachroboter >/dev/null 2>&1; then
  echo "The wachroboter package is not built/sourced."
  echo "See the README 'Build' section first."
  exit 1
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS2CLI_DISABLE_DAEMON=1

STAMP="$(date +"%Y-%m-%d_%H-%M-%S")"
RUN_DIR="$ROOT_DIR/sim_runs/${STAMP}_${MODE}"
mkdir -p "$RUN_DIR"

PIDS=()
cleanup() {
  set +e
  for pid in "${PIDS[@]:-}"; do
    kill "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM

start_bg() {
  local logfile="$1"
  shift
  "$@" >"$logfile" 2>&1 &
  PIDS+=("$!")
}

echo "=========================================="
echo " WACHROBOTER PORTABLE SIMULATION"
echo " Scenario: $MODE"
echo " Visitor codeword: $CODEWORD"
echo " ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo " Logs: $RUN_DIR"
echo "=========================================="

# Fake Guard and Visitor Action Servers.
start_bg "$RUN_DIR/guard_sim.log" \
  python3 "$ROOT_DIR/simulation/fake_robot_sim.py" \
    --ros-args \
    -r __ns:=/guard \
    -p robot_name:=guard \
    -p x0:=-4.800 \
    -p y0:=-0.714 \
    -p yaw0:=0.0

start_bg "$RUN_DIR/visitor_sim.log" \
  python3 "$ROOT_DIR/simulation/fake_robot_sim.py" \
    --ros-args \
    -r __ns:=/visitor \
    -p robot_name:=visitor \
    -p x0:=-4.902 \
    -p y0:=0.084 \
    -p yaw0:=0.0

sleep 1

start_bg "$RUN_DIR/labels.log" \
  python3 "$ROOT_DIR/simulation/rviz_labels.py"

start_bg "$RUN_DIR/topic_exchange.log" \
  python3 -u "$ROOT_DIR/simulation/topic_monitor.py"

# Map server for the synthetic, non-labor map.
start_bg "$RUN_DIR/map_server.log" \
  ros2 run nav2_map_server map_server \
    --ros-args -p yaml_filename:="$MAP_FILE"

sleep 1
ros2 lifecycle set /map_server configure >/dev/null
ros2 lifecycle set /map_server activate >/dev/null

if [[ "${NO_RVIZ:-0}" != "1" ]]; then
  start_bg "$RUN_DIR/rviz.log" rviz2 -d "$RVIZ_FILE"
fi

sleep 1

# The same C++ mission node runs once as Visitor and once as Guard.
start_bg "$RUN_DIR/visitor_mission.log" \
  ros2 run wachroboter wachroboter_node \
    --ros-args \
    --params-file "$MISSION_FILE" \
    -r __node:=visitor_mission \
    -r /navigate_to_pose:=/visitor/navigate_to_pose \
    -p role:=guest \
    -p codeword:="$CODEWORD"

sleep 2

start_bg "$RUN_DIR/guard_mission.log" \
  ros2 run wachroboter wachroboter_node \
    --ros-args \
    --params-file "$MISSION_FILE" \
    -r __node:=guard_mission \
    -r /navigate_to_pose:=/guard/navigate_to_pose \
    -p role:=guard \
    -p expected_codeword:=OPEN123

echo
echo "Demo running. Press Ctrl+C to stop."
echo "Mission communication:"
echo

tail -n +1 -F \
  "$RUN_DIR/topic_exchange.log" \
  "$RUN_DIR/guard_mission.log" \
  "$RUN_DIR/visitor_mission.log"
