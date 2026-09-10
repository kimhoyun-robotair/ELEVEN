#!/usr/bin/env bash
set -euo pipefail
amr_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
isaac_root="${ISAAC_SIM_PATH:-${HOME}/isaacsim}"
if [[ ! -x "$isaac_root/python.sh" ]]; then
  printf 'Set ISAAC_SIM_PATH to your Isaac Sim 5.1.0 installation.\n' >&2
  exit 2
fi
cd "$amr_root"
exec "$isaac_root/python.sh" model/build_assets.py "$@"
