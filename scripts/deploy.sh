#!/bin/bash
# Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear
#
# Deploy the wave_rover_controller + teleop stack from the shared overlay via xdeploy.

set -euo pipefail

THIS_SCRIPT=$(readlink -f "${BASH_SOURCE[0]}")
PKG_DIR=$(dirname "$(dirname "${THIS_SCRIPT}")")
# SDK root = the dir containing xcompile/; walk up so this works at any nesting depth.
SDK_DIR=$(dirname "${THIS_SCRIPT}")
while [ "${SDK_DIR}" != "/" ] && [ ! -f "${SDK_DIR}/xcompile/cross-env.sh" ]; do
    SDK_DIR=$(dirname "${SDK_DIR}")
done

"${SDK_DIR}/xcompile/xdeploy" \
    --packages "wave_rover_controller joy_linux teleop_twist_joy" "$@"

cat <<'EOF'

On the RB3 (after install):
  source /usr/share/qirp-setup.sh

  # Driver
  ros2 launch wave_rover_controller wave_rover_launch.py

  # Gamepad (separate terminal, loads teleop_joy.yaml)
  CONFIG=$(ros2 pkg prefix wave_rover_controller)/share/wave_rover_controller/config/teleop_joy.yaml
  ros2 run joy_linux joy_linux_node --ros-args --params-file "$CONFIG"
  ros2 run teleop_twist_joy teleop_node --ros-args --params-file "$CONFIG"
EOF
