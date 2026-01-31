#!/bin/bash

# Script to playback Gazebo log files with proper model paths

if [ -z "$1" ]; then
    echo "Usage: $0 <path_to_state.log>"
    echo "Example: $0 /opt/I-Drone/data/recordings/state.log"
    exit 1
fi

LOG_FILE="$1"

if [ ! -f "$LOG_FILE" ]; then
    echo "Error: Log file not found: $LOG_FILE"
    exit 1
fi

# Set up model and plugin paths
export GAZEBO_MODEL_PATH=/root/tfg/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:${GAZEBO_MODEL_PATH}
export GAZEBO_PLUGIN_PATH=/root/tfg/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic:${GAZEBO_PLUGIN_PATH}

echo "Starting Gazebo playback for: $LOG_FILE"
echo "Loading models from: $GAZEBO_MODEL_PATH"

# Start server with log playback
gzserver -p "$LOG_FILE" &
SERVER_PID=$!

# Wait for server to be ready
echo "Waiting for Gazebo server..."
timeout 30 bash -c "until nc -z localhost 11345 2>/dev/null; do sleep 0.5; done"

if [ $? -eq 0 ]; then
    echo "Server ready, starting GUI..."
    gzclient
    echo "GUI closed. Server still running (PID: $SERVER_PID)"
    echo "To stop the server, run: kill $SERVER_PID"
else
    echo "Error: Gazebo server failed to start"
    kill $SERVER_PID 2>/dev/null
    exit 1
fi
