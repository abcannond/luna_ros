#!/usr/bin/env bash
# Reliable colcon build for this workspace (bind-mounted at /ros2_ws in Docker).
#
# Default: copy install (no --symlink-install). That avoids intermittent
# [Errno 17] File exists errors from setuptools when symlinks in install/
# disagree with a new build (common for ament_python packages like luna_nav).
#
# Fast iteration: ./build_workspace.sh --symlink  (same as colcon --symlink-install)
#
# Usage:
#   ./build_workspace.sh
#   ./build_workspace.sh --symlink --packages-select luna_nav
#   ./build_workspace.sh --cmake-args -DCMAKE_BUILD_TYPE=Release

set -euo pipefail

usage() {
  sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'
}

SYMLINK=0
if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi
if [[ "${1:-}" == "--symlink" || "${1:-}" == "-s" ]]; then
  SYMLINK=1
  shift
fi

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${WS_ROOT}"

if [[ ! -d src ]]; then
  echo "build_workspace.sh: expected ${WS_ROOT}/src (ROS 2 workspace root)." >&2
  exit 1
fi

if ! command -v colcon >/dev/null 2>&1; then
  echo "build_workspace.sh: colcon not in PATH; source /opt/ros/jazzy/setup.bash" >&2
  exit 1
fi

args=()
if [[ "${SYMLINK}" -eq 1 ]]; then
  args+=(--symlink-install)
fi

exec colcon build "${args[@]}" "$@"
