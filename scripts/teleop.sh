#!/bin/bash
# Gamepad teleop for the Wave Rover, run on a laptop/host PC.
# Sources local ROS, configures DDS, and starts the joy + teleop_twist_joy nodes.
# Start the robot driver first; ROS_DOMAIN_ID must match the driver.

set -eo pipefail

# Local ROS install (Jazzy). ROS setup scripts reference unbound vars, so
# nounset stays off while sourcing.
source /opt/ros/jazzy/setup.bash

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
CONFIG="${SCRIPT_DIR}/../config/teleop_joy.yaml"

ros2 run joy_linux joy_linux_node --ros-args --params-file "$CONFIG" &
JOY_PID=$!
trap 'kill "$JOY_PID" 2>/dev/null' EXIT
ros2 run teleop_twist_joy teleop_node --ros-args --params-file "$CONFIG"
