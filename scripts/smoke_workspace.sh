#!/usr/bin/env bash
# Non-interactive smoke checks: ROS packages and launch --show-args for key entrypoints.
# Usage: from repo root, with workspace built and sourced, e.g.:
#   cd /ros2_ws && source install/setup.bash && cd /path/to/luna_ros && ./scripts/smoke_workspace.sh
#
# If the host has no ROS (ros2 not in PATH), use the project image and mount repo + workspace, e.g.:
#   docker run --rm -v "$PWD/ros2_ws:/ros2_ws:rw" -v "$PWD:/repo:ro" luna_ros:latest \
#     bash -lc 'source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && /repo/scripts/smoke_workspace.sh'

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "smoke_workspace: ros2 not found. Source /opt/ros/jazzy/setup.bash and ros2_ws/install/setup.bash" >&2
  exit 1
fi

echo "smoke_workspace: ros2 prefix OK"

required_pkgs=(
  lunabot_2425
  luna_nav
  luna_mapping
  luna_control
  fiducial_localizer
)

for pkg in "${required_pkgs[@]}"; do
  if ! ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
    echo "smoke_workspace: missing package $pkg (build and source workspace)" >&2
    exit 1
  fi
  echo "smoke_workspace: found package $pkg"
done

launches=(
  "lunabot_2425 competition_sim.launch.py"
  "lunabot_2425 gz_bringup.launch.py"
  "luna_mapping rtabmap_nav2_sim.launch.py"
  "luna_mapping rtabmap_nav2_hardware.launch.py"
)

for spec in "${launches[@]}"; do
  # shellcheck disable=SC2086
  set -- $spec
  pkg="$1"
  launch="$2"
  echo "smoke_workspace: ros2 launch $pkg $launch --show-args"
  ros2 launch "$pkg" "$launch" --show-args >/dev/null
done

echo "smoke_workspace: OK"
