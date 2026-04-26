#!/bin/bash
export GAZEBO_MODEL_PATH=/root/tfg/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:${GAZEBO_MODEL_PATH}
export GAZEBO_PLUGIN_PATH=/root/tfg/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic:${GAZEBO_PLUGIN_PATH}
export LD_LIBRARY_PATH=/root/tfg/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic:${LD_LIBRARY_PATH}

# GPS home coordinates from DRONE_0
if [[ ! -z "$DRONE_0_HOME_LAT" ]]; then
  export PX4_HOME_LAT=$DRONE_0_HOME_LAT
  export PX4_HOME_LON=$DRONE_0_HOME_LON
  export PX4_HOME_ALT=$DRONE_0_HOME_ALT
fi

# Use multi_drone world with camera tracking
BASE_WORLD_FILE="/opt/I-Drone/multi_drone.world"
WORLD_FILE="${BASE_WORLD_FILE}"
PID_FILE="/opt/I-Drone/data/simulation_processes.pid"

# Initialize simulation processes tracking file
mkdir -p /opt/I-Drone/data
umask 077
echo "# Simulation processes for cleanup" > "${PID_FILE}"

# Configure recording path for Gazebo
RECORDING_DIR="/opt/I-Drone/data/recordings"
mkdir -p ${RECORDING_DIR}

# Try to build a terrain world from a GeoTIFF (if available)
GENERATED_WORLD="/opt/I-Drone/data/generated/multi_drone_terrain.world"
GENERATED_HEIGHTMAP="/opt/I-Drone/data/generated/terrain_heightmap.png"
HEIGHTMAP_DIM="${DRONE_HEIGHTMAP_DIM:-2049}"
ALBEDO_DIM="${DRONE_TERRAIN_ALBEDO_DIM:-native}"
USE_TERRAIN="${DRONE_USE_TERRAIN:-1}"
ALBEDO_TIF="${DRONE_TERRAIN_ALBEDO_TIF:-}"
mkdir -p /opt/I-Drone/data/generated

if [[ "${USE_TERRAIN}" == "1" || "${USE_TERRAIN}" == "true" ]]; then
  TERRAIN_TIF=""
  if [[ -n "${DRONE_TERRAIN_TIF}" && -f "${DRONE_TERRAIN_TIF}" ]]; then
    TERRAIN_TIF="${DRONE_TERRAIN_TIF}"
  fi

  if [[ -z "${TERRAIN_TIF}" && -d "/workspace" ]]; then
    if [[ -d "/workspace/Docker/Sim" ]]; then
      TERRAIN_TIF=$(find /workspace/Docker/Sim -maxdepth 1 -type f -name "*.tif" | head -n 1)
    fi
    if [[ -z "${TERRAIN_TIF}" ]]; then
      TERRAIN_TIF=$(find /workspace -maxdepth 4 -type f -name "*.tif" | head -n 1)
    fi
  fi

  if [[ -z "${TERRAIN_TIF}" && -d "/opt/I-Drone/data/terrain" ]]; then
    TERRAIN_TIF=$(find /opt/I-Drone/data/terrain -maxdepth 1 -type f -name "*.tif" | head -n 1)
  fi

  if [[ -z "${ALBEDO_TIF}" && -d "/workspace/Docker/Sim" ]]; then
    ALBEDO_TIF=$(find /workspace/Docker/Sim -maxdepth 1 -type f \( -name "PNOA*.tif" -o -name "*.cog.tif" -o -name "*.cog" \) | head -n 1)
  fi

  if [[ -n "${ALBEDO_TIF}" && ! -f "${ALBEDO_TIF}" ]]; then
    echo "Warning: Albedo GeoTIFF not found: ${ALBEDO_TIF}. Falling back to terrain grayscale."
    ALBEDO_TIF=""
  fi

  if [[ -n "${TERRAIN_TIF}" ]]; then
    echo "Terrain GeoTIFF found: ${TERRAIN_TIF}"
    if command -v gdalinfo >/dev/null 2>&1 && command -v gdal_translate >/dev/null 2>&1; then
      TERRAIN_CMD=(
        python3 /opt/I-Drone/prepare_terrain_world.py
        --base-world "${BASE_WORLD_FILE}"
        --terrain-tif "${TERRAIN_TIF}"
        --heightmap-png "${GENERATED_HEIGHTMAP}"
        --output-world "${GENERATED_WORLD}"
        --heightmap-dim "${HEIGHTMAP_DIM}"
        --albedo-dim "${ALBEDO_DIM}"
      )
      if [[ -n "${ALBEDO_TIF}" ]]; then
        echo "Terrain albedo GeoTIFF found: ${ALBEDO_TIF}"
        TERRAIN_CMD+=(--albedo-tif "${ALBEDO_TIF}")
      fi

      if "${TERRAIN_CMD[@]}"; then
        WORLD_FILE="${GENERATED_WORLD}"
        echo "Using generated terrain world: ${WORLD_FILE}"
      else
        echo "Warning: Failed to generate terrain world. Falling back to base world."
      fi
    else
      echo "Warning: GDAL tools not available in container. Falling back to base world."
    fi
  else
    echo "No terrain GeoTIFF found. Using base world."
  fi
else
  echo "Terrain disabled by DRONE_USE_TERRAIN=${USE_TERRAIN}. Using base world."
fi

echo "Starting Gazebo Classic for multi-drone simulation with recording..."
gzserver --verbose -r --record_path ${RECORDING_DIR} ${WORLD_FILE} &
GZSERVER_PID=$!
echo "Gazebo server started with recording enabled. PID: $GZSERVER_PID"
echo "Recordings will be saved to: ${RECORDING_DIR}/<timestamp>/gzserver/state.log"

# Register process for cleanup
echo "gzserver:$GZSERVER_PID" >> "${PID_FILE}"

# Wait for Gazebo to be ready
echo "Waiting for Gazebo server to be ready..."
timeout 300 bash -c "until nc -z localhost 11345 2>/dev/null; do sleep 0.5; done"
if [[ $? -eq 0 ]]; then
  echo "Gazebo server ready and recording"
else
  echo "Warning: Gazebo port check timed out"
fi

# Start GUI if DISPLAY is set
if [[ ! -z "$DISPLAY" ]]; then
  echo "Starting Gazebo GUI..."
  gzclient &
  GZCLIENT_PID=$!
  echo "gzclient:$GZCLIENT_PID" >> "${PID_FILE}"
  echo "Gazebo GUI started"
fi

echo "Gazebo ready for drone connections"
wait $GZSERVER_PID
