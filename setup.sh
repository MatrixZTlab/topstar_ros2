#!/bin/bash
echo "Setup topstar ros2 environment"
source /opt/ros/humble/setup.bash
source $HOME/topstar_ros2/cyclonedds_ws/install/setup.bash
if [ -f "$HOME/topstar_ros2/example/install/setup.bash" ]; then
    source $HOME/topstar_ros2/example/install/setup.bash
fi
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="wlp132s0f0" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
