#!/bin/bash
export GAZEBO_MODEL_PATH=/root/tfg/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:${GAZEBO_MODEL_PATH}
export GAZEBO_PLUGIN_PATH=/root/tfg/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic:${GAZEBO_PLUGIN_PATH}
export LD_LIBRARY_PATH=/root/tfg/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic:${LD_LIBRARY_PATH}

# GPS home coordinates from DRONE_0
if [ ! -z "$DRONE_0_HOME_LAT" ]; then
  export PX4_HOME_LAT=$DRONE_0_HOME_LAT
  export PX4_HOME_LON=$DRONE_0_HOME_LON
  export PX4_HOME_ALT=$DRONE_0_HOME_ALT
fi

# Use empty world (standard PX4 approach)
WORLD_FILE="/root/tfg/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world"

echo "Starting Gazebo Classic for multi-drone simulation..."
gzserver --verbose ${WORLD_FILE} &
GZSERVER_PID=$!
echo "Gazebo server started. PID: $GZSERVER_PID"

# Wait for Gazebo to be ready
echo "Waiting for Gazebo server to be ready..."
timeout 30 bash -c "until nc -z localhost 11345 2>/dev/null; do sleep 0.5; done"
if [ $? -eq 0 ]; then
  echo "Gazebo server ready"
else
  echo "Warning: Gazebo port check timed out"
fi

# Start GUI if DISPLAY is set
if [ ! -z "$DISPLAY" ]; then
  echo "Starting Gazebo GUI..."
  gzclient &
  echo "Gazebo GUI started"
fi

echo "Gazebo ready for drone connections"
wait $GZSERVER_PID
