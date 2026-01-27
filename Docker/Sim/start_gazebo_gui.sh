#!/bin/bash
export GAZEBO_MODEL_PATH=/root/tfg/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:${GAZEBO_MODEL_PATH}
export GAZEBO_PLUGIN_PATH=/root/tfg/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic:${GAZEBO_PLUGIN_PATH}
export LD_LIBRARY_PATH=/root/tfg/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic:${LD_LIBRARY_PATH}

if [ -z "$DISPLAY" ]; then
  echo "ERROR: DISPLAY not set. Please set DISPLAY environment variable."
  echo "Example: export DISPLAY=:0"
  exit 1
fi

echo "Starting Gazebo Client (GUI)..."
gzclient
