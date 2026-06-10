#!/bin/bash
# Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

set -euo pipefail

THIS_SCRIPT=$(readlink -f "${BASH_SOURCE[0]}")
PKG_DIR=$(dirname "$(dirname "${THIS_SCRIPT}")")
DEPS_DIR="${PKG_DIR}/deps"
TELEOP_BRANCH="${TELEOP_BRANCH:-rolling}"

# QIR SDK setup puts the ARM sysroot on PATH; use host git for fetching sources.
GIT="$(command -v -p git)"
if [ -z "${GIT}" ]; then
    echo "ERROR: host git not found"
    exit 1
fi

clone_or_update() {
    local url="$1"
    local dest="$2"
    local branch="$3"

    if [ -d "${dest}/.git" ]; then
        "${GIT}" -C "${dest}" fetch --depth 1 origin "${branch}"
        "${GIT}" -C "${dest}" checkout "${branch}"
        "${GIT}" -C "${dest}" pull --ff-only origin "${branch}" || true
    else
        "${GIT}" clone --depth 1 --branch "${branch}" "${url}" "${dest}"
    fi
}

mkdir -p "${DEPS_DIR}"

echo "Fetching ROS teleop dependencies (teleop branch: ${TELEOP_BRANCH})..."
clone_or_update \
    https://github.com/ros-drivers/joystick_drivers.git \
    "${DEPS_DIR}/joystick_drivers" \
    "ros2"
clone_or_update \
    https://github.com/ros2/teleop_twist_joy.git \
    "${DEPS_DIR}/teleop_twist_joy" \
    "${TELEOP_BRANCH}"

echo "Teleop sources ready under ${DEPS_DIR}/"
