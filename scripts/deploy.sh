#!/bin/bash
# Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

set -euo pipefail

THIS_SCRIPT=$(readlink -f "${BASH_SOURCE[0]}")
PKG_DIR=$(dirname "$(dirname "${THIS_SCRIPT}")")
INSTALL_DIR="${PKG_DIR}/install"
TARBALL="wave_rover_controller.tar.gz"

RB3_USER="${RB3_USER:-root}"

if [ $# -ge 1 ]; then
    TARGET="$1"
elif [ -n "${RB3_IP:-}" ]; then
    TARGET="${RB3_USER}@${RB3_IP}"
else
    echo "Usage: $0 [${RB3_USER}@]<rb3-ip>"
    echo "  or:  RB3_IP=<rb3-ip> $0"
    echo "  optional: RB3_USER (default: root)"
    exit 1
fi

if [[ "${TARGET}" != *@* ]]; then
    TARGET="${RB3_USER}@${TARGET}"
fi
BINARY="${INSTALL_DIR}/lib/wave_rover_controller/wave_rover_controller"

if [ ! -f "${BINARY}" ]; then
    echo "ERROR: Run ./scripts/build.sh first."
    exit 1
fi

if ! file "${BINARY}" | grep -qE 'ARM aarch64|aarch64'; then
    echo "ERROR: Refusing to deploy non-aarch64 binary."
    file "${BINARY}"
    exit 1
fi

cd "${INSTALL_DIR}"
tar -czvf "${TARBALL}" lib share
scp "${TARBALL}" "${TARGET}:/opt/"

echo ""
echo "On the RB3:"
echo "  mount -o remount,rw /usr"
echo "  tar --no-overwrite-dir --no-same-owner -zxf /opt/${TARBALL} -C /usr/"
echo "  source /usr/share/qirp-setup.sh"
echo ""
echo "  # Driver"
echo "  ros2 launch wave_rover_controller wave_rover_launch.py"
echo ""
echo "  # Gamepad (separate terminal, loads teleop_joy.yaml)"
echo '  CONFIG=$(ros2 pkg prefix wave_rover_controller)/share/wave_rover_controller/config/teleop_joy.yaml'
echo '  ros2 run joy_linux joy_linux_node --ros-args --params-file "$CONFIG"'
echo '  ros2 run teleop_twist_joy teleop_node --ros-args --params-file "$CONFIG"'
echo ""
echo "Config:"
echo "  .../config/wave_rover_controller.yaml  # driver / diff-drive"
echo "  .../config/teleop_joy.yaml             # gamepad"
