#!/usr/bin/env bash
set -euo pipefail
AMR_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
AMR_ROS_SETUP="${AMR_ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
if [[ ! -r "$AMR_ROS_SETUP" ]]; then
  printf 'ROS 2 Jazzy setup not found: %s\nSee scripts/install_ros_deps.sh and docs/ROS_TOPICS.md.\n' "$AMR_ROS_SETUP" >&2
  exit 2
fi
if [[ -n "${CONDA_PREFIX:-}" || -n "${VIRTUAL_ENV:-}" ]]; then
  printf 'Open a plain system terminal; deactivate Conda/virtualenv before using Ubuntu ROS Jazzy.\n' >&2
  exit 2
fi
# ROS shell setup scripts may reference unset variables.
set +u
source "$AMR_ROS_SETUP"
set -u
if [[ "${ROS_DISTRO:-}" != "jazzy" ]]; then
  printf 'This tool workspace targets ROS_DISTRO=jazzy, got %s\n' "${ROS_DISTRO:-unset}" >&2
  exit 2
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
AMR_COMMAND="${1:-monitor}"
if [[ $# -gt 0 ]]; then shift; fi
if [[ "$AMR_COMMAND" == "build" ]]; then
  cd "$AMR_ROOT/ros2_ws"
  if [[ ! -x /usr/bin/colcon ]]; then
    printf 'System colcon is missing. Run scripts/install_ros_deps.sh --install.\n' >&2
    exit 2
  fi
  exec /usr/bin/python3 /usr/bin/colcon build --symlink-install --packages-select amr_tools "$@"
fi
if [[ ! -r "$AMR_ROOT/ros2_ws/install/setup.bash" ]]; then
  printf 'Build the ROS tools first: ./scripts/run_ros_tools.sh build\n' >&2
  exit 2
fi
set +u
source "$AMR_ROOT/ros2_ws/install/setup.bash"
set -u
case "$AMR_COMMAND" in
  monitor|teleop|smoke) exec ros2 run amr_tools "$AMR_COMMAND" "$@" ;;
  rviz) exec ros2 launch amr_tools view.launch.py "$@" ;;
  *) printf 'Usage: %s {build|monitor|teleop|smoke|rviz} [arguments]\n' "$0" >&2; exit 2 ;;
esac
