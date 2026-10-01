#!/usr/bin/env bash
# Start the fleet from the central PC: Zenoh, robot bringup, SLAM, robot
# Zenoh, the central stack, and RViz.
#
# Usage:
#   ./scripts/core/start_fleet.sh                       # -bpic, RViz on, host reverie
#   ./scripts/core/start_fleet.sh -ic                   # inky + clyde
#   ./scripts/core/start_fleet.sh -c --no-rviz
#   ./scripts/core/start_fleet.sh -ic --central-host othername
#   ./scripts/core/start_fleet.sh --verbose -c
#
# Robot filter letters match start_central.sh: b=blinky, p=pinky, i=inky, c=clyde.
# The filter always defaults to -bpic and is passed through to the central scripts.
#
# By default only warnings and errors from the child processes are printed.
# Pass --verbose to print every line. Full logs are always written to disk.
# The script stays in the foreground. Ctrl+C stops every process it started.
# Robot SSH uses sshpass and ROBOT_SSH_PASSWORD (default: ubuntu).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"

TIMEOUT_ZENOH_CENTRAL="${TIMEOUT_ZENOH_CENTRAL:-90}"
TIMEOUT_BRINGUP="${TIMEOUT_BRINGUP:-180}"
TIMEOUT_SLAM="${TIMEOUT_SLAM:-300}"
TIMEOUT_ZENOH_ROBOT="${TIMEOUT_ZENOH_ROBOT:-90}"
TIMEOUT_CENTRAL="${TIMEOUT_CENTRAL:-180}"
TIMEOUT_RVIZ="${TIMEOUT_RVIZ:-60}"

ALL_ROBOTS=(blinky pinky inky clyde)
declare -A LETTER_TO_ROBOT=(
  [b]="blinky"
  [p]="pinky"
  [i]="inky"
  [c]="clyde"
)

SELECTION=""
CENTRAL_HOST="reverie"
DRY_RUN=0
WITH_RVIZ=1
VERBOSE=0

PIDS=()
TAIL_PIDS=()
TMP_FILES=()
declare -A SSH_TARGET=()
STOPPING=0
LAST_PID=""

usage() {
  cat <<'EOF'
Usage:
  ./scripts/core/start_fleet.sh [robot_filter] [--central-host <name>] [--no-rviz] [--verbose] [--dry-run]

Robot filter (same letters as start_central.sh):
  b=blinky  p=pinky  i=inky  c=clyde
  Default: -bpic

Options:
  --central-host <name>   Tailscale name of this PC (default: reverie)
  --no-rviz               Do not start start_rviz_central.sh
  --verbose               Print every line from every process
  --dry-run               Print the stages and SSH targets, then exit
  -h, --help              Show this help

By default the terminal shows stage progress plus warnings and errors.
Full logs are still written under the Logs directory.

Environment:
  ROBOT_SSH_PASSWORD      SSH password for the robots (default: ubuntu)
  TIMEOUT_ZENOH_CENTRAL   Seconds to wait for central Zenoh (default: 90)
  TIMEOUT_BRINGUP         Seconds to wait for robot bringup (default: 180)
  TIMEOUT_SLAM            Seconds to wait for startup map seeding (default: 300)
  TIMEOUT_ZENOH_ROBOT     Seconds to wait for robot Zenoh (default: 90)
  TIMEOUT_CENTRAL         Seconds to wait for start_central.sh (default: 180)
  TIMEOUT_RVIZ            Seconds to wait for RViz to start (default: 60)

Requires sshpass (sudo apt install sshpass). The script stays in the
foreground until Ctrl+C, which stops every process it started.

Examples:
  ./scripts/core/start_fleet.sh
  ./scripts/core/start_fleet.sh -ic
  ./scripts/core/start_fleet.sh -c --no-rviz
  ./scripts/core/start_fleet.sh --verbose -c
  ./scripts/core/start_fleet.sh --dry-run -ic
EOF
}

while (($# > 0)); do
  case "$1" in
    -[bpic]*)
      SELECTION="${1#-}"
      ;;
    --central-host)
      shift
      if (($# == 0)); then
        echo "ERROR: --central-host requires a Tailscale name." >&2
        exit 1
      fi
      CENTRAL_HOST="$1"
      ;;
    --no-rviz)
      WITH_RVIZ=0
      ;;
    --verbose)
      VERBOSE=1
      ;;
    --dry-run)
      DRY_RUN=1
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: unknown argument '$1'" >&2
      echo "Use a robot filter like -c or -bpic, or --help." >&2
      exit 1
      ;;
  esac
  shift
done

if [[ -z "$SELECTION" ]]; then
  SELECTION="bpic"
fi

if [[ ! "$CENTRAL_HOST" =~ ^[A-Za-z0-9]([A-Za-z0-9.-]*[A-Za-z0-9])?$ ]]; then
  echo "ERROR: invalid central Tailscale name '${CENTRAL_HOST}'." >&2
  exit 1
fi

declare -A SELECTED_ROBOT_SET=()
for ((idx=0; idx<${#SELECTION}; idx++)); do
  ch="${SELECTION:$idx:1}"
  if [[ -z "${LETTER_TO_ROBOT[$ch]:-}" ]]; then
    echo "ERROR: invalid robot selector '${ch}' in '-${SELECTION}'" >&2
    echo "Use any combination of: b=blinky, p=pinky, i=inky, c=clyde" >&2
    exit 1
  fi
  SELECTED_ROBOT_SET["${LETTER_TO_ROBOT[$ch]}"]=1
done

ROBOTS=()
for robot in "${ALL_ROBOTS[@]}"; do
  [[ -n "${SELECTED_ROBOT_SET[$robot]:-}" ]] && ROBOTS+=("$robot")
done
if [[ ${#ROBOTS[@]} -eq 0 ]]; then
  echo "ERROR: no robots selected" >&2
  exit 1
fi

if (( ! DRY_RUN )); then
  if ! command -v sshpass >/dev/null 2>&1; then
    echo "ERROR: sshpass is required to log into the robots." >&2
    echo "  sudo apt install sshpass" >&2
    exit 1
  fi
  if (( WITH_RVIZ )) && [[ -z "${DISPLAY:-}" ]]; then
    echo "ERROR: DISPLAY is not set, so RViz cannot open." >&2
    echo "  Run this from a desktop terminal, or pass --no-rviz." >&2
    exit 1
  fi
  : "${ROBOT_SSH_PASSWORD:=ubuntu}"
  export SSHPASS="$ROBOT_SSH_PASSWORD"
fi

resolve_ssh_target() {
  local robot="$1"
  local target
  target="$(
    set -e
    # shellcheck disable=SC1091
    source "${WORKSPACE_DIR}/scripts/env/set_robot_env.sh" "$robot" >&2
    printf '%s' "${ROBOT_SSH:-}"
  )" || {
    echo "ERROR: could not resolve SSH target for ${robot}." >&2
    return 1
  }
  if [[ -z "$target" ]]; then
    echo "ERROR: empty SSH target for ${robot}." >&2
    return 1
  fi
  printf '%s' "$target"
}

for robot in "${ROBOTS[@]}"; do
  SSH_TARGET["$robot"]="$(resolve_ssh_target "$robot")"
done

ready_zenoh_central() {
  local robot="$1"
  printf 'Route Subscriber (Zenoh:%s/tf_zenoh -> ROS:/%s/tf_zenoh) created' "$robot" "$robot"
}

ready_bringup() {
  local robot="$1"
  printf '[%s.diff_drive_controller]: Run!' "$robot"
}

ready_slam() {
  local robot="$1"
  printf '[%s.startup_map_seeder]: Startup map seeding complete' "$robot"
}

ready_zenoh_robot() {
  local robot="$1"
  printf 'TF zenoh publish: /%s/tf -> /%s/tf_zenoh and /tf (best-effort in, merged, 10 Hz)' "$robot" "$robot"
}

print_plan() {
  local robot
  echo "Fleet bringup"
  echo "  robots:       ${ROBOTS[*]}"
  echo "  filter:       -${SELECTION}"
  echo "  central host: ${CENTRAL_HOST}"
  if (( WITH_RVIZ )); then
    echo "  rviz:         on"
  else
    echo "  rviz:         off"
  fi
  if (( VERBOSE )); then
    echo "  output:       all"
  else
    echo "  output:       warnings and errors"
  fi
  echo "  ssh:"
  for robot in "${ROBOTS[@]}"; do
    echo "    ${robot} -> ${SSH_TARGET[$robot]}"
  done
  echo "Stages:"
  echo "  1. ./scripts/comms/start_zenoh_central.sh -${SELECTION}"
  echo "     wait for each robot: $(ready_zenoh_central '<robot>')"
  echo "  2. on each robot, in parallel:"
  echo "       source scripts/env/ros_robot_env.bash"
  echo "       export TURTLEBOT3_MODEL=burger"
  echo "       export LDS_MODEL=LDS-02"
  echo "       ros2 launch turtlebot3_bringup robot.launch.py"
  echo "     wait for: $(ready_bringup '<robot>')"
  echo "  3. on each robot, in parallel:"
  echo "       ros2 launch turtlebot3_navigation2 navigation2_slam.launch.py"
  echo "     wait for: $(ready_slam '<robot>')"
  echo "  4. on each robot, in parallel:"
  echo "       ./scripts/comms/start_zenoh_robot.sh ${CENTRAL_HOST}"
  echo "     wait for: $(ready_zenoh_robot '<robot>')"
  echo "  5. ./scripts/core/start_central.sh -${SELECTION}"
  echo "     wait for: All services running.  Press Ctrl+C to stop."
  if (( WITH_RVIZ )); then
    echo "  6. ./scripts/core/start_rviz_central.sh"
    echo "     wait for: Starting RViz"
  fi
}

if (( DRY_RUN )); then
  print_plan
  exit 0
fi

RUN_DIR="${FLEET_LOG_DIR:-${XDG_RUNTIME_DIR:-/tmp}/fleet-bringup}/$(date +%Y%m%d-%H%M%S)-$$"
mkdir -p "$RUN_DIR"

kill_pid() {
  local pid="$1" sig="$2"
  local child
  if ! kill -0 "$pid" 2>/dev/null; then
    return 0
  fi
  # script/sshpass children sit in their own session. Signal those too so a
  # local launch or an SSH session actually stops.
  for child in $(pgrep -P "$pid" 2>/dev/null || true); do
    kill "-${sig}" -- "-${child}" 2>/dev/null || kill "-${sig}" "$child" 2>/dev/null || true
  done
  kill "-${sig}" -- "-${pid}" 2>/dev/null || kill "-${sig}" "$pid" 2>/dev/null || true
}

cleanup_processes() {
  local pid
  if (( STOPPING )); then
    return 0
  fi
  STOPPING=1
  echo ""
  echo "Stopping fleet..."
  for pid in "${PIDS[@]+"${PIDS[@]}"}"; do
    kill_pid "$pid" TERM
  done
  local i alive
  for i in 1 2 3 4 5 6 7 8; do
    alive=0
    for pid in "${PIDS[@]+"${PIDS[@]}"}"; do
      if kill -0 "$pid" 2>/dev/null; then
        alive=1
        break
      fi
    done
    (( alive == 0 )) && break
    sleep 1
  done
  for pid in "${PIDS[@]+"${PIDS[@]}"}"; do
    kill_pid "$pid" KILL
  done
  for pid in "${TAIL_PIDS[@]+"${TAIL_PIDS[@]}"}"; do
    kill_pid "$pid" TERM
  done
  local tmp
  for tmp in "${TMP_FILES[@]+"${TMP_FILES[@]}"}"; do
    rm -f "$tmp"
  done
}

on_signal() {
  cleanup_processes
  exit 130
}

trap cleanup_processes EXIT
trap on_signal INT TERM

prefix_follow() {
  local tag="$1"
  local file="$2"
  # ROS uses [ERROR]/[WARN]. Zenoh tracing uses a space-padded ERROR/WARN level.
  local alert_re='\[(ERROR|WARN|WARNING|FATAL)\]|(^|[[:space:]])(ERROR|WARN|WARNING|FATAL)([[:space:]]|:)'
  if (( VERBOSE )); then
    setsid bash -c '
      file=$1
      tag=$2
      while [[ ! -f "$file" ]]; do
        sleep 0.2
      done
      tail -n +1 -F "$file" | sed -u "s/\r\$//; s/^/[${tag}] /"
    ' _ "$file" "$tag" </dev/null &
  else
    setsid bash -c '
      file=$1
      tag=$2
      alert_re=$3
      while [[ ! -f "$file" ]]; do
        sleep 0.2
      done
      tail -n +1 -F "$file" | sed -u "s/\r\$//" | grep -iE --line-buffered "$alert_re" | sed -u "s/^/[${tag}] /"
    ' _ "$file" "$tag" "$alert_re" </dev/null &
  fi
  TAIL_PIDS+=("$!")
}

run_logged_bash() {
  local tag="$1"
  local logfile="$2"
  local body="$3"
  local payload runner
  payload="$(printf '%s\n' "$body" | base64 -w0)"
  runner="$(mktemp "/tmp/fleet-bringup.XXXXXX")"
  TMP_FILES+=("$runner")
  # script(1) runs the command with /bin/sh -c. Keep that string dash-safe.
  # A pty makes child logs flush so the ready line is visible in the typescript.
  # script copies the session to the terminal and to the typescript. Drop the
  # terminal copy so the prefixed follower is the only output.
  setsid script -q -f -e -c "echo ${payload} | base64 -d > ${runner} && bash ${runner}" "$logfile" </dev/null >/dev/null 2>&1 &
  LAST_PID=$!
  PIDS+=("$LAST_PID")
  prefix_follow "$tag" "$logfile"
}

central_body() {
  local command="$1"
  cat <<EOF
set +u
cd $(printf '%q' "$WORKSPACE_DIR")
source $(printf '%q' "${WORKSPACE_DIR}/scripts/env/ros_domain_profile.bash")
source $(printf '%q' "${WORKSPACE_DIR}/scripts/env/ros_robot_env.bash")
exec ${command}
EOF
}

start_central_stage() {
  local tag="$1"
  local logfile="$2"
  local command="$3"
  echo "=== ${tag} ==="
  run_logged_bash "$tag" "$logfile" "$(central_body "$command")"
}

remote_body() {
  local command="$1"
  local remote_path="$2"
  cat <<EOF
set +u
cd "\$HOME/turtlebot3"
source scripts/env/ros_robot_env.bash
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=$(printf '%q' "${LDS_MODEL:-LDS-02}")
export PATH="\$HOME/.local/bin:\$PATH"
shopt -s huponexit
cleanup_remote() {
  trap - EXIT INT TERM HUP
  if [[ -n "\${CHILD_PID:-}" ]] && kill -0 "\$CHILD_PID" 2>/dev/null; then
    pkill -TERM -P "\$CHILD_PID" 2>/dev/null || true
    kill -TERM "\$CHILD_PID" 2>/dev/null || true
    sleep 2
    pkill -KILL -P "\$CHILD_PID" 2>/dev/null || true
    kill -KILL "\$CHILD_PID" 2>/dev/null || true
  fi
  rm -f $(printf '%q' "$remote_path")
}
trap cleanup_remote EXIT INT TERM HUP
${command} &
CHILD_PID=\$!
wait "\$CHILD_PID"
EOF
}

start_robot_stage() {
  local robot="$1"
  local stage="$2"
  local command="$3"
  local logfile="${RUN_DIR}/${robot}-${stage}.log"
  local payload remote_name remote_path target
  remote_name="fleet-bringup-${robot}-${stage}.sh"
  remote_path="/tmp/${remote_name}"
  payload="$(printf '%s\n' "$(remote_body "$command" "$remote_path")" | base64 -w0)"
  target="${SSH_TARGET[$robot]}"
  setsid sshpass -e ssh -tt \
    -o StrictHostKeyChecking=accept-new \
    -o PreferredAuthentications=password \
    -o PubkeyAuthentication=no \
    -o NumberOfPasswordPrompts=1 \
    -o ConnectTimeout=20 \
    -o ServerAliveInterval=15 \
    -o ServerAliveCountMax=4 \
    "$target" \
    "echo ${payload} | base64 -d > /tmp/${remote_name} && bash /tmp/${remote_name}" \
    >"$logfile" 2>&1 &
  LAST_PID=$!
  PIDS+=("$LAST_PID")
  prefix_follow "${robot}:${stage}" "$logfile"
}

wait_ready() {
  local pid="$1"
  local logfile="$2"
  local timeout="$3"
  local label="$4"
  shift 4
  local -a patterns=("$@")
  local deadline=$((SECONDS + timeout))
  local pat found
  while (( SECONDS < deadline )); do
    found=1
    for pat in "${patterns[@]}"; do
      if ! grep -qF -- "$pat" "$logfile" 2>/dev/null; then
        found=0
        break
      fi
    done
    if (( found == 1 )); then
      echo "Ready: ${label}"
      return 0
    fi
    if ! kill -0 "$pid" 2>/dev/null; then
      echo "ERROR: ${label} exited before it was ready." >&2
      echo "Last log lines (${logfile}):" >&2
      tail -n 40 "$logfile" >&2 || true
      return 1
    fi
    sleep 0.5
  done
  echo "ERROR: timed out after ${timeout}s waiting for ${label}." >&2
  echo "Still waiting for:" >&2
  for pat in "${patterns[@]}"; do
    if ! grep -qF -- "$pat" "$logfile" 2>/dev/null; then
      echo "  ${pat}" >&2
    fi
  done
  echo "Last log lines (${logfile}):" >&2
  tail -n 40 "$logfile" >&2 || true
  return 1
}

echo "Logs: ${RUN_DIR}"
print_plan
echo ""

zenoh_central_log="${RUN_DIR}/zenoh-central.log"
start_central_stage "zenoh-central" "$zenoh_central_log" \
  "./scripts/comms/start_zenoh_central.sh -${SELECTION}"
zenoh_central_pid="$LAST_PID"
zenoh_patterns=()
for robot in "${ROBOTS[@]}"; do
  zenoh_patterns+=("$(ready_zenoh_central "$robot")")
done
wait_ready "$zenoh_central_pid" "$zenoh_central_log" "$TIMEOUT_ZENOH_CENTRAL" \
  "central Zenoh" "${zenoh_patterns[@]}"

declare -A STAGE_PID=()
for robot in "${ROBOTS[@]}"; do
  echo "=== ${robot}: bringup ==="
  start_robot_stage "$robot" "bringup" \
    "ros2 launch turtlebot3_bringup robot.launch.py"
  STAGE_PID["$robot"]="$LAST_PID"
done
for robot in "${ROBOTS[@]}"; do
  wait_ready "${STAGE_PID[$robot]}" "${RUN_DIR}/${robot}-bringup.log" \
    "$TIMEOUT_BRINGUP" "${robot} bringup" "$(ready_bringup "$robot")"
done

for robot in "${ROBOTS[@]}"; do
  echo "=== ${robot}: slam ==="
  start_robot_stage "$robot" "slam" \
    "ros2 launch turtlebot3_navigation2 navigation2_slam.launch.py"
  STAGE_PID["$robot"]="$LAST_PID"
done
for robot in "${ROBOTS[@]}"; do
  wait_ready "${STAGE_PID[$robot]}" "${RUN_DIR}/${robot}-slam.log" \
    "$TIMEOUT_SLAM" "${robot} slam" "$(ready_slam "$robot")"
done

for robot in "${ROBOTS[@]}"; do
  echo "=== ${robot}: zenoh ==="
  start_robot_stage "$robot" "zenoh" \
    "./scripts/comms/start_zenoh_robot.sh ${CENTRAL_HOST}"
  STAGE_PID["$robot"]="$LAST_PID"
done
for robot in "${ROBOTS[@]}"; do
  wait_ready "${STAGE_PID[$robot]}" "${RUN_DIR}/${robot}-zenoh.log" \
    "$TIMEOUT_ZENOH_ROBOT" "${robot} zenoh" "$(ready_zenoh_robot "$robot")"
done

central_log="${RUN_DIR}/central.log"
start_central_stage "central" "$central_log" \
  "./scripts/core/start_central.sh -${SELECTION}"
wait_ready "$LAST_PID" "$central_log" "$TIMEOUT_CENTRAL" \
  "central stack" "All services running.  Press Ctrl+C to stop."

if (( WITH_RVIZ )); then
  rviz_log="${RUN_DIR}/rviz.log"
  start_central_stage "rviz" "$rviz_log" \
    "./scripts/core/start_rviz_central.sh"
  wait_ready "$LAST_PID" "$rviz_log" "$TIMEOUT_RVIZ" \
    "RViz" "Starting RViz"
fi

echo ""
echo "Fleet is up. Press Ctrl+C to stop."

while true; do
  for pid in "${PIDS[@]}"; do
    if ! kill -0 "$pid" 2>/dev/null; then
      echo "ERROR: a fleet process exited (pid ${pid})." >&2
      exit 1
    fi
  done
  sleep 1
done
