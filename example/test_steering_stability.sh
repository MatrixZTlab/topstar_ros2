#!/usr/bin/env bash
# test_steering_stability.sh — Test the steering stabilization fix
# 
# This script will:
# 1. Source the environment with stabilization parameters
# 2. Launch the H1 simulator with viewer
# 3. Send a command sequence that previously caused tipover
#
# Expected behavior with the fix:
# - Robot smoothly transitions from idle → lateral motion (vy=0.3)
# - Wheel velocities oscillate less severely (peak ~12-15 rad/s vs previous 32-48)
# - Roll stays within ±0.35 rad throughout the transition
# - No "unsafe attitude" warnings

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EXAMPLE_DIR="$REPO_DIR/example"

echo "═══════════════════════════════════════════════════════════════"
echo "H1 Steering Stabilization Test"
echo "═══════════════════════════════════════════════════════════════"
echo ""
echo "Test parameters:"
echo "  steer_kp: 120 → 60 N·m/rad"
echo "  steer_kd: 70 → 40 N·m·s/rad"
echo "  align_drive_damping: 18 → 40 N·m·s/rad (PRIMARY FIX)"
echo "  large_steer_brake_damping: 15 → 35 N·m·s/rad"
echo "  max_steer_rate_cos: 0.3 → 0.15 rad/s"
echo "  max_steer_wait_torque: 45 → 30 Nm"
echo "  hard_overspeed_rate: 10 → 12 rad/s"
echo ""

# Source the tuning environment
source "$EXAMPLE_DIR/h1_tune_env.sh"

# Source ROS2
source /opt/ros/humble/setup.bash
source "$REPO_DIR/cyclonedds_ws/install/setup.bash"
source "$REPO_DIR/example/install/setup.bash"

echo "Starting H1 simulator (viewer enabled)..."
echo ""
echo "STEP 1: Simulator will start. Wait for '❌ ROS2 node ready' message."
echo "        Then proceed to STEP 2 in another terminal."
echo ""

ros2 launch topstar_ros2_example h1_sim.launch.py viewer:=true
