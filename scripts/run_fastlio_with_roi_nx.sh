#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export FASTLIO_ENV_SCRIPT="${SCRIPT_DIR}/env_fastlio_nx.sh"
exec "${SCRIPT_DIR}/run_fastlio_with_roi.sh" "$@"
