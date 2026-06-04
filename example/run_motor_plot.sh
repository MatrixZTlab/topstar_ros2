#!/bin/bash
# Launch the H2 joint motor visualizer.
# All arguments are forwarded to h2_motor_plot.py.
#
# Usage examples:
#   ./run_motor_plot.sh
#   ./run_motor_plot.sh --joints left_leg --mode torque
#   ./run_motor_plot.sh --joints 0 1 2 3 --mode both --window 5
#   ./run_motor_plot.sh --joints legs --cols 4 --window 15
#   ./run_motor_plot.sh --joints left_leg right_arm --mode pos

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"

source /opt/ros/humble/setup.bash
source "$ROOT_DIR/cyclonedds_ws/install/setup.bash"
if [ -f "$ROOT_DIR/example/install/setup.bash" ]; then
    source "$ROOT_DIR/example/install/setup.bash"
fi
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="eno1" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'

exec python3 "$SCRIPT_DIR/h2_motor_plot.py" "$@"
