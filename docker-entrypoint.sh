#!/bin/bash
# Docker entrypoint for SITL/HIL

set -e

# Source ROS 2
source /opt/ros/humble/setup.bash

# Source workspace if built
if [ -f /workspace/src/companion/install/setup.bash ]; then
    source /workspace/src/companion/install/setup.bash
fi

# Source PX4 if built
if [ -f /workspace/Firmware/build/px4_sitl_default/bin/px4 ]; then
    export PX4_SIM_MODEL=gazebo-classic
    export PX4_SIM_SPEED_FACTOR=1
fi

# Set Gazebo paths
export GAZEBO_MODEL_PATH=/workspace/src/simulation/sitl/models:${GAZEBO_MODEL_PATH}
export GAZEBO_PLUGIN_PATH=/workspace/src/simulation/sitl/plugins:${GAZEBO_PLUGIN_PATH}

# Execute command
exec "$@"