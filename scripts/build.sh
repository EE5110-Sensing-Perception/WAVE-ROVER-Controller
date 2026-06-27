#!/bin/bash
# Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear
#
# Cross-compile wave_rover_controller + teleop stack via the shared xcompile toolkit.

set -euo pipefail

THIS_SCRIPT=$(readlink -f "${BASH_SOURCE[0]}")
PKG_DIR=$(dirname "$(dirname "${THIS_SCRIPT}")")
# SDK root = the dir containing xcompile/; walk up so this works at any nesting depth.
SDK_DIR=$(dirname "${THIS_SCRIPT}")
while [ "${SDK_DIR}" != "/" ] && [ ! -f "${SDK_DIR}/xcompile/cross-env.sh" ]; do
    SDK_DIR=$(dirname "${SDK_DIR}")
done

# Fetch the teleop dependency sources (joy_linux, teleop_twist_joy) into deps/.
"${PKG_DIR}/scripts/fetch_teleop_deps.sh"

# Build only the packages we need from this tree into the shared overlay.
#
# Discovery: this package lives at PKG_DIR root with vendored teleop sources under
# deps/, so colcon's recursive crawl would only ever find wave_rover_controller (it
# stops at the package root). Pass explicit '--paths' for exactly the package dirs
# we build. This deliberately does NOT crawl all of deps/, which would also pull in
# joy/sdl2_vendor — teleop_twist_joy only needs them at runtime (exec_depend) and we
# drive the joystick via joy_linux, so building them is unnecessary.
exec "${SDK_DIR}/xcompile/xbuild" "${PKG_DIR}" \
    --select wave_rover_controller \
    --select joy_linux \
    --select teleop_twist_joy \
    "$@" \
    -- --paths \
        "${PKG_DIR}" \
        "${PKG_DIR}/deps/joystick_drivers/joy_linux" \
        "${PKG_DIR}/deps/teleop_twist_joy"
