#!/usr/bin/env bash
set -euo pipefail

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'EOF'
Usage:
  ./scripts/validation/run_fleet_validation.sh <scenario> <log_file>
  ./scripts/validation/run_fleet_validation.sh <scenario> <baseline_log> <candidate_log>

Scenarios:
  one_robot   - single robot + central
  two_robot   - two robots + central
  full_fleet  - all robots + central

This script reports KPI counts from captured ROS logs.
When two logs are provided, it prints candidate-minus-baseline deltas.
EOF
  exit 0
fi

if [[ $# -ne 2 && $# -ne 3 ]]; then
  echo "ERROR: expected 2 or 3 args: <scenario> <log_file> [candidate_log]"
  exit 1
fi

SCENARIO="$1"
LOG_FILE="$2"
CANDIDATE_LOG="${3:-}"

case "$SCENARIO" in
  one_robot|two_robot|full_fleet) ;;
  *)
    echo "ERROR: invalid scenario '$SCENARIO' (use one_robot|two_robot|full_fleet)"
    exit 1
    ;;
esac

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "Scenario: $SCENARIO"
if [[ -z "$CANDIDATE_LOG" ]]; then
  python3 "${SCRIPT_DIR}/fleet_kpi_report.py" "$LOG_FILE"
else
  echo "Baseline log: $LOG_FILE"
  echo "Candidate log: $CANDIDATE_LOG"
  python3 "${SCRIPT_DIR}/fleet_kpi_report.py" "$LOG_FILE" --compare-other "$CANDIDATE_LOG"
fi
