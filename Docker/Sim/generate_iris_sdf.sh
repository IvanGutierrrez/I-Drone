#!/bin/bash
# Generate iris SDF using PX4's official jinja template
# Usage: ./generate_iris_sdf.sh <instance_id> <output_file>

INSTANCE=$1
OUTPUT=$2

PX4_DIR="/root/tfg/PX4-Autopilot"
MODEL="iris"

# Calculate unique ports per instance
TCP_PORT=$((4560 + INSTANCE))
UDP_PORT=$((14560 + INSTANCE))
MAVLINK_ID=$((1 + INSTANCE))
GST_UDP_PORT=$((5600 + INSTANCE))
CAM_UDP_PORT=$((14530 + INSTANCE))

# Use PX4's official jinja generator (same as sitl_multiple_run.sh)
python3 ${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/scripts/jinja_gen.py \
    ${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/${MODEL}/${MODEL}.sdf.jinja \
    ${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic \
    --mavlink_tcp_port ${TCP_PORT} \
    --mavlink_udp_port ${UDP_PORT} \
    --mavlink_id ${MAVLINK_ID} \
    --gst_udp_port ${GST_UDP_PORT} \
    --video_uri ${GST_UDP_PORT} \
    --mavlink_cam_udp_port ${CAM_UDP_PORT} \
    --output-file ${OUTPUT}

echo "Generated base SDF for instance ${INSTANCE}: TCP=${TCP_PORT}, UDP=${UDP_PORT}, ID=${MAVLINK_ID}"

# Add camera sensor using Python XML parser
TEMP_SDF="${OUTPUT}.tmp"
mv ${OUTPUT} ${TEMP_SDF}
python3 /root/tfg/add_camera_to_sdf.py ${TEMP_SDF} ${OUTPUT} ${INSTANCE}
rm ${TEMP_SDF}

echo "Added camera to ${OUTPUT}"


