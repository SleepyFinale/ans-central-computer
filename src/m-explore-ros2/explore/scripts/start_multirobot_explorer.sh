#!/bin/bash
# Compatibility shim for a legacy launcher path.
# The canonical central startup command is scripts/core/start_central.sh.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"

echo "DEPRECATED: start_multirobot_explorer.sh is now a compatibility shim."
echo "Use './scripts/core/start_central.sh' for normal central startup."
echo ""

exec "${WORKSPACE_DIR}/scripts/core/start_central.sh" "$@"
