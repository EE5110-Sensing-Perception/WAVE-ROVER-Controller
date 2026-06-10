#!/bin/bash
# Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

set -euo pipefail

THIS_SCRIPT=$(readlink -f "${BASH_SOURCE[0]}")
PKG_DIR=$(dirname "$(dirname "${THIS_SCRIPT}")")

if [ -z "${OECORE_TARGET_SYSROOT:-}" ]; then
    echo "ERROR: QIR SDK cross-compile environment is not set."
    echo "  source <qirp-sdk>/setup.sh"
    exit 1
fi

cd "${PKG_DIR}"

"${PKG_DIR}/scripts/fetch_teleop_deps.sh"

# Cross-compile hygiene — host ROS and LD_LIBRARY_PATH break the Yocto toolchain.
if [ -n "${LD_LIBRARY_PATH:-}" ]; then
    echo "NOTE: Unsetting LD_LIBRARY_PATH for cross-compile (required by QIR SDK)."
    unset LD_LIBRARY_PATH
fi

if [[ "${CMAKE_PREFIX_PATH:-}" == *"/opt/ros"* ]] \
        || [[ "${AMENT_PREFIX_PATH:-}" == *"/opt/ros"* ]] \
        || [[ "${COLCON_PREFIX_PATH:-}" == *"/opt/ros"* ]]; then
    echo "NOTE: Clearing host ROS (/opt/ros) prefix paths for cross-compile."
    unset CMAKE_PREFIX_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH
fi

# Host build tools — SDK PATH may expose ARM sysroot binaries to env/python3.
HOST_PYTHON="${OECORE_NATIVE_SYSROOT}/usr/bin/python3"
export PATH="${OECORE_NATIVE_SYSROOT}/usr/bin:/usr/local/bin:/usr/bin:/bin:${PATH}"

export AMENT_PREFIX_PATH="${OECORE_NATIVE_SYSROOT}/usr:${OECORE_TARGET_SYSROOT}/usr"
export PYTHONPATH="${OECORE_NATIVE_SYSROOT}/usr/lib/python3.12/site-packages/:${OECORE_TARGET_SYSROOT}/usr/lib/python3.12/site-packages/"

echo "Cross-compiling wave_rover_controller + teleop (joy_linux, teleop_twist_joy)..."
echo "  Compiler: $(command -v aarch64-qcom-linux-gcc || echo 'NOT FOUND')"
echo "  Python:   ${HOST_PYTHON}"

"${HOST_PYTHON}" -m colcon build --merge-install \
    --paths \
        "${PKG_DIR}" \
        "${PKG_DIR}/deps/joystick_drivers/joy_linux" \
        "${PKG_DIR}/deps/teleop_twist_joy" \
    --cmake-args ${CMAKE_ARGS} \
        -DCMAKE_TOOLCHAIN_FILE="${CMAKE_TOOLCHAIN_FILE}" \
        -DCMAKE_PREFIX_PATH="${OECORE_TARGET_SYSROOT}/usr" \
        -DCMAKE_FIND_ROOT_PATH="${OECORE_TARGET_SYSROOT}" \
        -DPython3_EXECUTABLE="${OECORE_NATIVE_SYSROOT}/usr/bin/python3" \
        -DPython3_FIND_STRATEGY=LOCATION

BINARY="${PKG_DIR}/install/lib/wave_rover_controller/wave_rover_controller"
JOY_BINARY="${PKG_DIR}/install/lib/joy_linux/joy_linux_node"
TELEOP_BINARY="${PKG_DIR}/install/lib/teleop_twist_joy/teleop_node"

file "${BINARY}"

if ! file "${BINARY}" | grep -qE 'ARM aarch64|aarch64'; then
    echo "ERROR: binary is not aarch64"
    exit 1
fi

for artifact in "${JOY_BINARY}" "${TELEOP_BINARY}"; do
    if [ ! -f "${artifact}" ]; then
        echo "ERROR: missing teleop artifact: ${artifact}"
        exit 1
    fi
    if ! file "${artifact}" | grep -qE 'ARM aarch64|aarch64'; then
        echo "ERROR: teleop binary is not aarch64: ${artifact}"
        file "${artifact}"
        exit 1
    fi
done

echo "Cross-compile succeeded:"
echo "  ${BINARY}"
echo "  ${JOY_BINARY}"
echo "  ${TELEOP_BINARY}"
