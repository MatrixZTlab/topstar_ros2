#!/usr/bin/env bash
# test_jog_commands.sh — Send test velocity commands via h1_upper_body_jog
#
# Run this in a second terminal after the simulator is fully loaded.

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Source the same environment as the simulator
source "$REPO_DIR/example/h1_tune_env.sh"
source /opt/ros/humble/setup.bash
source "$REPO_DIR/cyclonedds_ws/install/setup.bash"
source "$REPO_DIR/example/install/setup.bash"

echo "═══════════════════════════════════════════════════════════════"
echo "H1 Steering Stability Test — Jog Commands"
echo "═══════════════════════════════════════════════════════════════"
echo ""
echo "This test sends commands that previously caused tipover:"
echo "  1. Idle hold at vy=0.02 (stabilize)"
echo "  2. Transition to vy=0.3 (90° steering, large angle)"
echo "  3. Monitor for tipover (roll > 0.35 rad / pitch > 0.40 rad)"
echo ""
echo "Run this in a SECOND terminal while the simulator is running."
echo ""

sleep 2

echo "STEP 2: Starting jog node..."
ros2 run topstar_ros2_example h1_upper_body_jog

# The jog node is interactive. Use these commands:
# w - increase vx
# s - decrease vx  
# a - decrease vy (turn left/negative-y)
# d - increase vy (turn right/positive-y)
# x - stop (vx=0, vy=0)
# space - park (enter idle hold mode)
# ESC - exit
#
# Test sequence:
# 1. Press SPACE to enter idle hold (vx≈0, vy≈0.02 for stability)
# 2. Press D multiple times rapidly to reach vy≈0.3 (lateral motion, 90° steering)
# 3. Observe the simulator viewer:
#    - Roll should stay within ±0.35 rad
#    - Pitch should stay within ±0.40 rad
#    - Drive wheel velocities should not exceed ~15 rad/s sustained
# 4. If unstable, press X or ESC to return to safe state
