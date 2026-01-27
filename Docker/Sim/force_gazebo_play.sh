#!/bin/bash
# Force Gazebo to play by sending step commands continuously
while true; do
  gz world -s 0.001 2>/dev/null
  sleep 0.001
done
