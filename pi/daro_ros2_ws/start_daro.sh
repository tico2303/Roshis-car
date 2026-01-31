#!/usr/bin/env bash
set -e

# ---- Environment ----
source /opt/ros/jazzy/setup.bash
source /home/pi/Code/Roshis-car/pi/daro_ros2_ws/install/setup.bash

sleep 5

# ---- Launch DARO ----
exec ros2 launch daro_bringup daro.launch.py