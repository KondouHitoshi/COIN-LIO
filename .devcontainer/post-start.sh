#!/usr/bin/env bash
set -euo pipefail

runtime_dir="${XDG_RUNTIME_DIR:-/tmp/runtime-$(id -un)}"
mkdir -p "${runtime_dir}"
chmod 700 "${runtime_dir}" 2>/dev/null || true

source /opt/ros/noetic/setup.bash

cd "${CATKIN_WS:-${HOME}/catkin_ws}"
if [ ! -x devel/lib/coin_lio/coin_lio_mapping ]; then
  catkin_make -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}" -j"${CATKIN_BUILD_JOBS:-$(nproc)}"
fi
