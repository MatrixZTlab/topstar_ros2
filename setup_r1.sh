#!/bin/bash
echo "Setup topstar ros2 environment (Robot 1, domain 1)"
source /opt/ros/humble/setup.bash
source $HOME/topstar_ros2/cyclonedds_ws/install/setup.bash
if [ -f "$HOME/topstar_ros2/example/install/setup.bash" ]; then
    source $HOME/topstar_ros2/example/install/setup.bash
fi
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=1
# IMPORTANT: keep CYCLONEDDS_URI on one line — CycloneDDS fails to parse multiline env vars.
# WiFi path: reach A via wlp4s0 (192.168.1.11) on the same WiFi subnet — no cross-subnet routing needed.
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="wlp132s0f0" priority="default" multicast="default"/></Interfaces><MaxMessageSize>1438B</MaxMessageSize></General><Discovery><Peers><Peer Address="192.168.1.11"/></Peers></Discovery></Domain></CycloneDDS>'
# Stop any stale ros2 daemon (graceful stop cleans socket files; pkill leaves them and causes !rclpy.ok() errors)
timeout 5 ros2 daemon stop 2>/dev/null || true
