#!/bin/bash
echo "Setup topstar ros2 environment (Robot 1, domain 1, wired)"
source /opt/ros/humble/setup.bash
source $HOME/topstar_ros2/cyclonedds_ws/install/setup.bash
if [ -f "$HOME/topstar_ros2/example/install/setup.bash" ]; then
    source $HOME/topstar_ros2/example/install/setup.bash
fi
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=1
# Restrict to wired interface, cap MTU for B's lan1 (MTU 1466), list explicit peers.
# Multicast does not cross the subnet-36/37 boundary, so peers are required for
# discovering Computer A (only reachable via routing through B).
# IMPORTANT: keep this on one line — CycloneDDS fails to parse multiline env vars.
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enp131s0" priority="default" multicast="default"/></Interfaces></General><Internal><MaxMessageSize>1438B</MaxMessageSize></Internal><Discovery><Peers><Peer Address="192.168.37.10"/><Peer Address="192.168.37.11"/><Peer Address="192.168.36.10"/><Peer Address="192.168.36.40"/></Peers></Discovery></Domain></CycloneDDS>'
# Kill stale daemons that may have started with wrong rmw/domain
pkill -f 'ros2-daemon.*rmw-implementation rmw_fastrtps' 2>/dev/null || true
