#!/usr/bin/env bash
set -euo pipefail
if [[ ! -r /etc/os-release ]]; then
  printf 'This script targets Ubuntu 24.04.\n' >&2
  exit 2
fi
source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "24.04" ]]; then
  printf 'Expected Ubuntu 24.04; found %s %s.\n' "${ID:-unknown}" "${VERSION_ID:-unknown}" >&2
  exit 2
fi
if [[ ! -r /opt/ros/jazzy/setup.bash ]]; then
  printf '%s\n' \
    'Install ROS 2 Jazzy from the official Ubuntu instructions first:' \
    'https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html' \
    'Choose ros-jazzy-desktop (includes RViz), then rerun this script.' >&2
  exit 2
fi
AMR_PACKAGES=(python3-colcon-common-extensions python3-setuptools ros-jazzy-rclpy \
  ros-jazzy-geometry-msgs ros-jazzy-nav-msgs ros-jazzy-sensor-msgs ros-jazzy-rosgraph-msgs \
  ros-jazzy-tf2-msgs ros-jazzy-launch-ros ros-jazzy-ament-index-python \
  ros-jazzy-rviz2 ros-jazzy-rmw-fastrtps-cpp)
printf 'Dependency install command (no changes without --install):\nsudo apt-get install'
printf ' %q' "${AMR_PACKAGES[@]}"
printf '\n'
if [[ "${1:-}" == "--install" ]]; then
  sudo apt-get update
  sudo apt-get install "${AMR_PACKAGES[@]}"
elif [[ $# -ne 0 ]]; then
  printf 'Usage: %s [--install]\n' "$0" >&2
  exit 2
fi
