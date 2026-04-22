#!/usr/bin/env bash
#
# Set ROS domain profile:
# - fleet robot host/arg: use mapped robot domain from fleet_domain_map.yaml
# - non-fleet host/arg: default to central/non-fleet domain 244
# Usage:
#   source scripts/ros_domain_profile.bash

_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_workspace_dir="$(dirname "$_script_dir")"
_domain_map="${_workspace_dir}/config/fleet_domain_map.yaml"
_default_domain="244"
_target_robot="${1:-${HOSTNAME:-}}"

if [ -f "${_domain_map}" ]; then
  _resolved_domain="$(python3 - <<'PY' "${_domain_map}" "${_target_robot}" "${_default_domain}"
import sys, yaml
domain_map_path, target_robot, default_domain = sys.argv[1], (sys.argv[2] or "").strip().lower(), int(sys.argv[3])
with open(domain_map_path, "r", encoding="utf-8") as f:
    data = yaml.safe_load(f) or {}
robot_map = data.get("fleet_domain_map", {}).get("robot_domain_ids", {}) or {}
if target_robot and target_robot in robot_map:
    print(int(robot_map[target_robot]))
else:
    print(default_domain)
PY
)"
  export ROS_DOMAIN_ID="${_resolved_domain}"
else
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-${_default_domain}}"
fi
echo "ROS domain profile loaded: target='${_target_robot:-unknown}', ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
