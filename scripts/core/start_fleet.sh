#!/usr/bin/env bash
# Start the fleet from the central PC: Zenoh, robot bringup, SLAM, robot
# Zenoh, the central stack, and RViz.
#
# Usage:
#   ./scripts/core/start_fleet.sh                       # -bpic, RViz on, host = this PC
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
# Robot stages run detached on the robots (/tmp/fleet-stages/), so a robot that
# drops off Wi-Fi keeps its stack and drives itself home
# (scripts/comms/fleet_link_watchdog.py on the robot). The fleet keeps running,
# re-attaches when the robot is back, and stops robot stages over SSH.
# Robot SSH uses sshpass and ROBOT_SSH_PASSWORD (default: ubuntu).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../env/fleet_common.bash
source "${SCRIPT_DIR}/../env/fleet_common.bash"
WORKSPACE_DIR="$FLEET_WORKSPACE_DIR"

TIMEOUT_ZENOH_CENTRAL="${TIMEOUT_ZENOH_CENTRAL:-90}"
TIMEOUT_BRINGUP="${TIMEOUT_BRINGUP:-180}"
TIMEOUT_SLAM="${TIMEOUT_SLAM:-300}"
TIMEOUT_ZENOH_ROBOT="${TIMEOUT_ZENOH_ROBOT:-90}"
TIMEOUT_ZENOH_ACTIONS="${TIMEOUT_ZENOH_ACTIONS:-30}"
TIMEOUT_CENTRAL="${TIMEOUT_CENTRAL:-180}"
TIMEOUT_RVIZ="${TIMEOUT_RVIZ:-60}"

SELECTION=""
# Robots dial this PC by name. Use the short hostname unless --central-host
# overrides it. Tailscale names are lowercase.
CENTRAL_HOST="$(hostname -s 2>/dev/null | tr '[:upper:]' '[:lower:]')"
DRY_RUN=0
WITH_RVIZ=1
VERBOSE=0

PIDS=()
TAIL_PIDS=()
TMP_FILES=()
declare -A SSH_TARGET=()
declare -A PID_LABEL=()
ROBOT_STAGE_ORDER=()
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
  --central-host <name>   Name passed to start_zenoh_robot.sh
                          (default: this computer's hostname)
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
  TIMEOUT_ZENOH_ACTIONS   Seconds for the robot bridge to find Nav2's action
                          servers before that stage is restarted once (default: 30)
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
      SELECTION="$1"
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

fleet_select_robots "$SELECTION"
SELECTION="$(for r in "${ROBOTS[@]}"; do printf '%s' "${r:0:1}"; done)"

if [[ -z "$CENTRAL_HOST" ]]; then
  echo "ERROR: could not read this computer's hostname. Pass --central-host <name>." >&2
  exit 1
fi
if [[ ! "$CENTRAL_HOST" =~ ^[A-Za-z0-9]([A-Za-z0-9.-]*[A-Za-z0-9])?$ ]]; then
  echo "ERROR: invalid central Tailscale name '${CENTRAL_HOST}'." >&2
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
  printf 'TF zenoh publish: /%s/tf -> /%s/tf_zenoh' "$robot" "$robot"
}

# The robot bridge must also have discovered both Nav2 action servers. On
# 2026-11-03 it missed the planner's (a DDS discovery race at bridge start):
# the explorer then waited for compute_path_to_pose forever and the robot
# never moved. One line per action, printed by the bridge on discovery.
ready_zenoh_actions() {
  local robot="$1"
  printf 'declares Action Server /%s/navigate_to_pose\n' "$robot"
  printf 'declares Action Server /%s/compute_path_to_pose\n' "$robot"
}

# A central bridge that is still shutting down holds the port, and a new one
# then exits at once with "Address already in use".
wait_zenoh_port_free() {
  local waited=0
  while ss -tln 2>/dev/null | grep -q ':7447 ' && ((waited < 15)); do
    sleep 1
    waited=$((waited + 1))
  done
}

forget_pid() {
  local gone="$1" pid
  local -a kept=()
  for pid in "${PIDS[@]}"; do
    [[ "$pid" == "$gone" ]] || kept+=("$pid")
  done
  PIDS=("${kept[@]}")
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
  echo "     and both Nav2 action servers discovered (restarted once if not)"
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
  # Robot stages outlive their SSH sessions, so stop them on the robots.
  stop_robot_stages
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
  # A Python node that crashes prints neither; its traceback header and the
  # final "SomethingError: ..." line are matched too.
  local alert_re='\[(ERROR|WARN|WARNING|FATAL)\]|(^|[[:space:]])(ERROR|WARN|WARNING|FATAL)([[:space:]]|:)|Traceback \(most recent call last\)|(^|[[:space:]])[A-Za-z_][A-Za-z0-9_.]*(Error|Exception): '
  # Known-harmless startup lines, hidden from the terminal (the log keeps
  # them): v4l2_camera looks for a per-camera calibration file and probes
  # controls the Arducams refuse (stereo uses config/stereo/<robot>.yaml)
  # and notes its YUYV -> mono8 conversion; Nav2 notes once that it uses its
  # only goal checker; RViz prints an empty GLSL link log and ignores Wayland.
  local noise_re='camera_calibration_parsers|[Cc]amera calibration file|Failed getting value for control|Control type not currently supported|possibly slow conversion|No goal checker was specified|GLSL link result|Ignoring XDG_SESSION_TYPE=wayland'
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
      noise_re=$4
      while [[ ! -f "$file" ]]; do
        sleep 0.2
      done
      tail -n +1 -F "$file" | sed -u "s/\r\$//" | grep -iE --line-buffered "$alert_re" \
        | grep -vE --line-buffered "$noise_re" | sed -u "s/^/[${tag}] /"
    ' _ "$file" "$tag" "$alert_re" "$noise_re" </dev/null &
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
  PID_LABEL["$LAST_PID"]="$tag"
  prefix_follow "$tag" "$logfile"
}

central_body() {
  local command="$1"
  cat <<EOF
set +u
cd $(printf '%q' "$WORKSPACE_DIR")
source $(printf '%q' "${WORKSPACE_DIR}/scripts/env/central_env.bash")
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

# Where a robot stage keeps its log and process-group id on the robot.
remote_stage_paths() {
  local robot="$1" stage="$2"
  cat <<EOF
STAGE_DIR=/tmp/fleet-stages
LOG_FILE=/tmp/fleet-stages/${robot}-${stage}.log
PID_FILE=/tmp/fleet-stages/${robot}-${stage}.pid
EOF
}

remote_body() {
  local command="$1" robot="$2" stage="$3"
  cat <<EOF
set +u
cd "\$HOME/turtlebot3"
source scripts/env/ros_robot_env.bash
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=$(printf '%q' "${LDS_MODEL:-LDS-02}")
export PATH="\$HOME/.local/bin:\$PATH"
# The stage writes to a file, not to this SSH pty: keep its output line-buffered.
export PYTHONUNBUFFERED=1
export RCUTILS_LOGGING_BUFFERED_STREAM=0
$(remote_stage_paths "$robot" "$stage")
mkdir -p "\$STAGE_DIR"
# Left over from a run that could not stop it (the robot was off the network).
if [[ -f "\$PID_FILE" ]]; then
  old=\$(cat "\$PID_FILE")
  if kill -0 -- "-\$old" 2>/dev/null; then
    echo "Stopping ${stage} left over from an earlier run (process group \$old)"
    kill -TERM -- "-\$old" 2>/dev/null
    for _ in \$(seq 1 20); do kill -0 -- "-\$old" 2>/dev/null || break; sleep 0.5; done
    kill -KILL -- "-\$old" 2>/dev/null
  fi
fi
# The stage runs in its own session and writes to a file, so it outlives this
# SSH connection: a robot that loses Wi-Fi keeps its stack running, including
# the failsafe that drives it home. The fleet stops it with stop_robot_stage,
# not by hanging up.
: > "\$LOG_FILE"
setsid stdbuf -oL -eL bash -c $(printf '%q' "$command") > "\$LOG_FILE" 2>&1 < /dev/null &
CHILD_PID=\$!
echo "\$CHILD_PID" > "\$PID_FILE"
tail -n +1 -F --pid="\$CHILD_PID" "\$LOG_FILE" 2>/dev/null
wait "\$CHILD_PID"
EOF
}

# One-off command on a robot (no pty). Exit 255 = robot unreachable.
robot_ssh() {
  local robot="$1"
  shift
  sshpass -e ssh \
    -o StrictHostKeyChecking=accept-new \
    -o PreferredAuthentications=password \
    -o PubkeyAuthentication=no \
    -o NumberOfPasswordPrompts=1 \
    -o ConnectTimeout=8 \
    "${SSH_TARGET[$robot]}" "$@" </dev/null
}

# alive | dead | unreachable
robot_stage_status() {
  local robot="$1" stage="$2" rc=0
  robot_ssh "$robot" "$(remote_stage_paths "$robot" "$stage")
[[ -f \"\$PID_FILE\" ]] && kill -0 -- \"-\$(cat \"\$PID_FILE\")\" 2>/dev/null" >/dev/null 2>&1 || rc=$?
  if (( rc == 0 )); then
    echo alive
  elif (( rc == 255 )); then
    echo unreachable
  else
    echo dead
  fi
}

# Stop a robot stage and everything it started. Returns 255 if the robot
# cannot be reached.
stop_robot_stage() {
  local robot="$1" stage="$2"
  robot_ssh "$robot" "$(remote_stage_paths "$robot" "$stage")
[[ -f \"\$PID_FILE\" ]] || exit 0
p=\$(cat \"\$PID_FILE\")
kill -TERM -- \"-\$p\" 2>/dev/null
for _ in \$(seq 1 20); do kill -0 -- \"-\$p\" 2>/dev/null || break; sleep 0.5; done
kill -KILL -- \"-\$p\" 2>/dev/null
rm -f \"\$PID_FILE\"
exit 0" >/dev/null 2>&1
}

# Stop every robot stage, newest first, robots in parallel.
stop_robot_stages() {
  local robot i key
  local -a jobs=()
  for robot in "${ROBOTS[@]+"${ROBOTS[@]}"}"; do
    (
      if ! robot_ssh "$robot" true >/dev/null 2>&1; then
        echo "WARNING: cannot reach ${robot}; its stack is still running there." >&2
        echo "  It is stopped at the next fleet start, or by hand: ssh ${SSH_TARGET[$robot]} 'for f in /tmp/fleet-stages/*.pid; do kill -TERM -- -\$(cat \$f); done'" >&2
        exit 0
      fi
      for (( i=${#ROBOT_STAGE_ORDER[@]}-1; i>=0; i-- )); do
        key="${ROBOT_STAGE_ORDER[$i]}"
        [[ "$key" == "${robot}:"* ]] || continue
        stop_robot_stage "$robot" "${key#*:}" || true
      done
    ) &
    jobs+=("$!")
  done
  if (( ${#jobs[@]} )); then
    wait "${jobs[@]}"
  fi
}

robot_ssh_tt() {
  local robot="$1" remote_command="$2" logfile="$3"
  setsid sshpass -e ssh -tt \
    -o StrictHostKeyChecking=accept-new \
    -o PreferredAuthentications=password \
    -o PubkeyAuthentication=no \
    -o NumberOfPasswordPrompts=1 \
    -o ConnectTimeout=20 \
    -o ServerAliveInterval=15 \
    -o ServerAliveCountMax=4 \
    "${SSH_TARGET[$robot]}" "$remote_command" \
    >>"$logfile" 2>&1 &
  LAST_PID=$!
}

start_robot_stage() {
  local robot="$1"
  local stage="$2"
  local command="$3"
  local logfile="${RUN_DIR}/${robot}-${stage}.log"
  local payload remote_name
  remote_name="fleet-bringup-${robot}-${stage}.sh"
  payload="$(printf '%s\n' "$(remote_body "$command" "$robot" "$stage")" | base64 -w0)"
  : > "$logfile"
  robot_ssh_tt "$robot" \
    "echo ${payload} | base64 -d > /tmp/${remote_name} && bash /tmp/${remote_name}" \
    "$logfile"
  PIDS+=("$LAST_PID")
  PID_LABEL["$LAST_PID"]="${robot}:${stage}"
  ROBOT_STAGE_ORDER+=("${robot}:${stage}")
  prefix_follow "${robot}:${stage}" "$logfile"
}

# Follow a stage that is still running on the robot after its SSH session
# dropped. Lines written while out of contact stay in the robot's log file.
attach_robot_stage() {
  local robot="$1"
  local stage="$2"
  local logfile="${RUN_DIR}/${robot}-${stage}.log"
  echo "--- re-attached $(date +%H:%M:%S); lines from while out of contact are in ${robot}:/tmp/fleet-stages/${robot}-${stage}.log ---" >>"$logfile"
  robot_ssh_tt "$robot" \
    "$(remote_stage_paths "$robot" "$stage"); [[ -f \"\$PID_FILE\" ]] && tail -n 0 -F --pid=\"\$(cat \"\$PID_FILE\")\" \"\$LOG_FILE\"" \
    "$logfile"
  PID_LABEL["$LAST_PID"]="${robot}:${stage}"
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

wait_zenoh_port_free
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
  readarray -t action_patterns < <(ready_zenoh_actions "$robot")
  if ! wait_ready "${STAGE_PID[$robot]}" "${RUN_DIR}/${robot}-zenoh.log" \
      "$TIMEOUT_ZENOH_ACTIONS" "${robot} Nav2 actions over Zenoh" "${action_patterns[@]}"; then
    echo "WARNING: ${robot}'s Zenoh bridge did not find both Nav2 action servers; restarting it and the central bridge once." >&2
    stop_robot_stage "$robot" zenoh || true
    kill_pid "${STAGE_PID[$robot]}" TERM
    forget_pid "${STAGE_PID[$robot]}"
    # The central bridge has to restart too. It keeps a second DDS reader for
    # each action route the old robot bridge announced, so every goal reaches
    # Nav2 twice, and Nav2 dies on the duplicate: "goal ID already exists"
    # (crashed 3 of 3 times when forced). Other robots' bridges reconnect.
    kill_pid "$zenoh_central_pid" TERM
    forget_pid "$zenoh_central_pid"
    sleep 3
    wait_zenoh_port_free
    zenoh_central_log="${RUN_DIR}/zenoh-central-retry.log"
    start_central_stage "zenoh-central-retry" "$zenoh_central_log" \
      "./scripts/comms/start_zenoh_central.sh -${SELECTION}"
    zenoh_central_pid="$LAST_PID"
    wait_ready "$zenoh_central_pid" "$zenoh_central_log" "$TIMEOUT_ZENOH_CENTRAL" \
      "central Zenoh (retry)" "${zenoh_patterns[@]}"
    start_robot_stage "$robot" "zenoh-retry" \
      "./scripts/comms/start_zenoh_robot.sh ${CENTRAL_HOST}"
    STAGE_PID["$robot"]="$LAST_PID"
    wait_ready "${STAGE_PID[$robot]}" "${RUN_DIR}/${robot}-zenoh-retry.log" \
      "$TIMEOUT_ZENOH_ROBOT" "${robot} zenoh (retry)" \
      "$(ready_zenoh_robot "$robot")" "${action_patterns[@]}"
  fi
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

# A robot's SSH session can drop while its stage keeps running there (Wi-Fi
# out of reach). Then the fleet keeps going: the robot drives itself home
# (fleet_link_watchdog.py) and is followed again once it can be reached.
declare -A DETACHED=()   # robot:stage -> time of the next contact attempt
declare -A ROBOT_LOST=()

robot_stage_dropped() {
  local key="$1" robot="${1%%:*}" stage="${1#*:}"
  case "$(robot_stage_status "$robot" "$stage")" in
    alive)
      echo "WARNING: SSH session for ${key} dropped; the stage is still running, re-attached." >&2
      attach_robot_stage "$robot" "$stage"
      NEW_PIDS+=("$LAST_PID")
      ;;
    unreachable)
      DETACHED["$key"]=$((SECONDS + 10))
      if [[ -z "${ROBOT_LOST[$robot]:-}" ]]; then
        ROBOT_LOST["$robot"]=1
        echo "WARNING: lost contact with ${robot}. Its stack keeps running and drives it home if the link stays down; retrying every 10 s." >&2
      fi
      ;;
    *)
      return 1
      ;;
  esac
}

while true; do
  alive=()
  NEW_PIDS=()
  for pid in "${PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      alive+=("$pid")
      continue
    fi
    label="${PID_LABEL[$pid]:-unknown}"
    if [[ "$label" == "rviz" ]]; then
      echo "WARNING: RViz exited. The rest of the fleet is still running." >&2
      continue
    fi
    if [[ "$label" == *:* && -n "${SSH_TARGET[${label%%:*}]:-}" ]] \
        && robot_stage_dropped "$label"; then
      continue
    fi
    echo "ERROR: ${label} exited (pid ${pid})." >&2
    exit 1
  done
  for key in "${!DETACHED[@]}"; do
    [[ $SECONDS -ge ${DETACHED[$key]} ]] || continue
    robot="${key%%:*}"
    case "$(robot_stage_status "$robot" "${key#*:}")" in
      alive)
        unset "DETACHED[$key]"
        if [[ -n "${ROBOT_LOST[$robot]:-}" ]]; then
          unset "ROBOT_LOST[$robot]"
          echo "Contact with ${robot} restored." >&2
        fi
        attach_robot_stage "$robot" "${key#*:}"
        NEW_PIDS+=("$LAST_PID")
        ;;
      unreachable)
        DETACHED["$key"]=$((SECONDS + 10))
        ;;
      *)
        echo "ERROR: ${key} stopped while ${robot} was out of contact." >&2
        exit 1
        ;;
    esac
  done
  PIDS=("${alive[@]+"${alive[@]}"}" "${NEW_PIDS[@]+"${NEW_PIDS[@]}"}")
  sleep 1
done
