#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
isaac_root="${ISAAC_SIM_PATH:-${HOME}/isaacsim}"
if [[ ! -x "$isaac_root/python.sh" ]]; then
    echo "Isaac Sim 5.1.0 python.sh not found: $isaac_root/python.sh" >&2
    echo "Set ISAAC_SIM_PATH to the extracted Isaac Sim 5.1.0 directory." >&2
    exit 2
fi
# Ubuntu Jazzy binaries use Python 3.12; Isaac Sim 5.1 embeds Python 3.11.
# A clean process is required; do not mix their binary extension paths.
for ros_path in "${PYTHONPATH:-}" "${LD_LIBRARY_PATH:-}" "${AMENT_PREFIX_PATH:-}" "${CMAKE_PREFIX_PATH:-}"; do
    if [[ "$ros_path" == *"/opt/ros/"* ]]; then
        echo "Use a fresh terminal without sourcing /opt/ros/jazzy/setup.bash for Isaac Sim." >&2
        echo "The external ROS tools run in a separate Jazzy-sourced terminal." >&2
        exit 2
    fi
done
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+${LD_LIBRARY_PATH}:}${isaac_root}/exts/isaacsim.ros2.bridge/jazzy/lib"
cd "$project_root"
exec "$isaac_root/python.sh" "$project_root/sim/run_sim.py" "$@"
