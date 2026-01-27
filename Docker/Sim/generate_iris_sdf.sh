#!/bin/bash
# Generate iris SDF with custom TCP/UDP ports for multi-drone setup
# Usage: ./generate_iris_sdf.sh <instance_id> <output_file>

INSTANCE=$1
OUTPUT=$2
BASE_SDF="/root/tfg/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf"

# Calculate ports for lockstep: 4560 + instance, 14560 + instance
TCP_PORT=$((4560 + INSTANCE))
UDP_PORT=$((14560 + INSTANCE))

# Copy base SDF
cp "$BASE_SDF" "$OUTPUT"

# Add tcp_port in mavlink_interface plugin if not present
if ! grep -q "<tcp_port>" "$OUTPUT"; then
  sed -i "s|<plugin name='mavlink_interface' filename='libgazebo_mavlink_interface.so'>|<plugin name='mavlink_interface' filename='libgazebo_mavlink_interface.so'>\n      <tcp_port>${TCP_PORT}</tcp_port>|" "$OUTPUT"
fi

# Update existing mavlink_tcp_port and mavlink_udp_port
sed -i "s|<mavlink_tcp_port>[0-9]*</mavlink_tcp_port>|<mavlink_tcp_port>${TCP_PORT}</mavlink_tcp_port>|g" "$OUTPUT"
sed -i "s|<mavlink_udp_port>[0-9]*</mavlink_udp_port>|<mavlink_udp_port>${UDP_PORT}</mavlink_udp_port>|g" "$OUTPUT"

# Enable lockstep only for first drone (instance 0) to drive Gazebo time
if [ "$INSTANCE" -eq 0 ]; then
  sed -i "s|<enable_lockstep>0</enable_lockstep>|<enable_lockstep>1</enable_lockstep>|g" "$OUTPUT"
  sed -i "s|<enable_lockstep>false</enable_lockstep>|<enable_lockstep>true</enable_lockstep>|g" "$OUTPUT"
  echo "Generated SDF for instance $INSTANCE: TCP=$TCP_PORT, UDP=$UDP_PORT (lockstep ENABLED - master)"
else
  sed -i "s|<enable_lockstep>1</enable_lockstep>|<enable_lockstep>0</enable_lockstep>|g" "$OUTPUT"
  sed -i "s|<enable_lockstep>true</enable_lockstep>|<enable_lockstep>false</enable_lockstep>|g" "$OUTPUT"
  echo "Generated SDF for instance $INSTANCE: TCP=$TCP_PORT, UDP=$UDP_PORT (lockstep disabled - slave)"
fi

# Increase sensor update rates
sed -i "s|<update_rate>[0-9]*</update_rate>|<update_rate>250</update_rate>|g" "$OUTPUT"
