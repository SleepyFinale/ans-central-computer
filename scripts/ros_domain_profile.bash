#!/usr/bin/env bash
#
# Set central ROS domain used by start_central.sh in bridged mode.
# Usage:
#   source scripts/ros_domain_profile.bash

_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_workspace_dir="$(dirname "$_script_dir")"
_domain_map="${_workspace_dir}/config/fleet_domain_map.yaml"

if [ -f "${_domain_map}" ]; then
  _central_domain="$(python3 - <<'PY' "${_domain_map}"
import sys, yaml
with open(sys.argv[1], "r", encoding="utf-8") as f:
    data = yaml.safe_load(f) or {}
print(data.get("fleet_domain_map", {}).get("central_domain_id", 90))
PY
)"
  export ROS_DOMAIN_ID="${_central_domain}"
else
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-90}"
fi
echo "Central ROS domain profile loaded: ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
