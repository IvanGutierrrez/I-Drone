#!/bin/bash
export GAZEBO_MODEL_PATH=/root/tfg/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:${GAZEBO_MODEL_PATH}
export GAZEBO_PLUGIN_PATH=/root/tfg/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic:${GAZEBO_PLUGIN_PATH}
export LD_LIBRARY_PATH=/root/tfg/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic:${LD_LIBRARY_PATH}

# GPS home coordinates will be read by gazebo_gps_plugin from environment variables
# Use DRONE_0 coordinates as the global reference for Gazebo
if [ ! -z "$DRONE_0_HOME_LAT" ]; then
  export PX4_HOME_LAT=$DRONE_0_HOME_LAT
  export PX4_HOME_LON=$DRONE_0_HOME_LON
  export PX4_HOME_ALT=$DRONE_0_HOME_ALT
fi

echo "Starting Gazebo Classic for multi-drone simulation..."
gzserver --verbose -u /opt/I-Drone/multi_drone.world &
GZSERVER_PID=$!
echo "Gazebo server started. PID: $GZSERVER_PID"

# Wait for Gazebo server to be ready
echo "Waiting for Gazebo server to be ready..."
timeout 30 bash -c "until nc -z localhost 11345 2>/dev/null; do sleep 0.5; done"
if [ $? -eq 0 ]; then
  echo "Gazebo server ready"
else
  echo "Warning: Gazebo port check timed out, but server may still be running"
fi

# Start Gazebo client (GUI) if DISPLAY is set
if [ ! -z "$DISPLAY" ]; then
  echo "Starting Gazebo GUI..."
  gzclient &
  echo "Gazebo GUI started"
fi

echo "Gazebo ready for drone connections"
wait $GZSERVER_PID
