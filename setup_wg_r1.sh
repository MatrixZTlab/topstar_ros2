#!/bin/bash
echo "Setup topstar ros2 environment (Robot 1, domain 1, WireGuard)"
source /opt/ros/humble/setup.bash
source $HOME/topstar_ros2/cyclonedds_ws/install/setup.bash
if [ -f "$HOME/topstar_ros2/example/install/setup.bash" ]; then
    source $HOME/topstar_ros2/example/install/setup.bash
fi
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=1
# WireGuard path: wg0 puts dev PC (10.0.0.1) and Computer A (10.0.0.2) on same virtual subnet.
# No cross-subnet routing needed — DDS discovery works reliably.
# IMPORTANT: keep CYCLONEDDS_URI on one line — CycloneDDS fails to parse multiline env vars.
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="wg0" priority="default" multicast="default"/></Interfaces><MaxMessageSize>1386B</MaxMessageSize></General><Discovery><Peers><Peer Address="10.0.0.2"/></Peers></Discovery></Domain></CycloneDDS>'
# Stop any stale ros2 daemon (graceful stop cleans socket files; pkill leaves them and causes !rclpy.ok() errors)
timeout 5 ros2 daemon stop 2>/dev/null || true
