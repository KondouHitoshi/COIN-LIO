#!/usr/bin/env bash
set -e

source /opt/ros/noetic/setup.bash

if [ -n "${XDG_RUNTIME_DIR:-}" ]; then
  mkdir -p "${XDG_RUNTIME_DIR}"
  chmod 700 "${XDG_RUNTIME_DIR}" 2>/dev/null || true
fi

if [ "${COIN_LIO_AUTO_BUILD:-1}" = "1" ] \
  && [ -n "${CATKIN_WS:-}" ] \
  && [ -f "${CATKIN_WS}/src/COIN-LIO/package.xml" ]; then
  if [ "${COIN_LIO_FORCE_BUILD:-0}" = "1" ] \
    || [ ! -x "${CATKIN_WS}/devel/lib/coin_lio/coin_lio_mapping" ]; then
    cd "${CATKIN_WS}"
    catkin_make -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}" -j"${CATKIN_BUILD_JOBS:-$(nproc)}"
  fi
fi

if [ -n "${CATKIN_WS:-}" ] && [ -f "${CATKIN_WS}/devel/setup.bash" ]; then
  source "${CATKIN_WS}/devel/setup.bash"
fi

exec "$@"
