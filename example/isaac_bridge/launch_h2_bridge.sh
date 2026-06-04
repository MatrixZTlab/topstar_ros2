#!/usr/bin/env bash
# launch_h2_bridge.sh — Start Isaac Sim + ROS2 bridge for H2 robot.
#
# Usage:
#   bash ~/topstar_ros2/example/isaac_bridge/launch_h2_bridge.sh           # GUI
#   bash ~/topstar_ros2/example/isaac_bridge/launch_h2_bridge.sh --headless
#
# ZMQ sockets (localhost only):
#   :15557  Isaac Sim → bridge  (state,    PUSH/PULL)
#   :15558  bridge → Isaac Sim  (commands, PUSH/PULL)

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

CONDA_PYTHON="${HOME}/miniforge3/envs/env_isaaclab/bin/python"
ROS2_SETUP="/opt/ros/humble/setup.bash"
# Message bindings live in the synced repo — no separate build step needed on this machine.
ROS2_WS="${HOME}/topstar_ros2/cyclonedds_ws"

# ── 1. Isaac Sim process ──────────────────────────────────────────────────
echo "[launch] Starting Isaac Sim (H2) ..."
"${CONDA_PYTHON}" "${SCRIPT_DIR}/h2_isaac_sim.py" "$@" &
ISAAC_PID=$!
echo "[launch] Isaac Sim PID=${ISAAC_PID}"

# Isaac Sim takes ~15-30 s to initialise and load the scene
echo "[launch] Waiting 25 s for Isaac Sim to start ..."
sleep 25

# ── 2. ROS2 bridge process ────────────────────────────────────────────────
RMW_SO=$(find /opt/ros/humble/lib -name "librmw_cyclonedds_cpp.so" 2>/dev/null | head -1)
if [[ -z "${RMW_SO}" ]]; then
    echo "[launch] ERROR: ros-humble-rmw-cyclonedds-cpp is not installed on this machine."
    echo "[launch]        Fix: sudo apt install ros-humble-rmw-cyclonedds-cpp"
    kill "${ISAAC_PID}" 2>/dev/null || true
    exit 1
fi

# Auto-detect the LAN interface that reaches 192.168.1.0/24.
LAN_IFACE=$(ip route get 192.168.1.1 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="dev") print $(i+1)}' | head -1)
LAN_IFACE="${LAN_IFACE:-eth0}"
echo "[launch] Starting ROS2 bridge (CycloneDDS, iface=${LAN_IFACE}) ..."
bash -c "
  source '${ROS2_SETUP}'
  source '${ROS2_WS}/install/local_setup.bash'
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${LAN_IFACE}\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>'
  python3 '${SCRIPT_DIR}/h2_isaac_ros2_bridge.py'
" &
BRIDGE_PID=$!
echo "[launch] ROS2 bridge PID=${BRIDGE_PID}"

# ── Trap Ctrl-C ───────────────────────────────────────────────────────────
cleanup() {
    echo "[launch] Shutting down ..."
    kill "${BRIDGE_PID}" 2>/dev/null || true
    kill "${ISAAC_PID}"  2>/dev/null || true
    wait 2>/dev/null || true
}
trap cleanup INT TERM EXIT

echo "[launch] Both processes running. Ctrl-C to stop."
echo "[launch] Topics: /lowstate (pub), /lowcmd (sub)"
wait "${ISAAC_PID}"
