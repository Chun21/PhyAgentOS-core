#!/usr/bin/env bash
# Start an explicitly supervised arm session on the robot host.
set -euo pipefail
: "${CONDA_PREFIX:?Activate the phyagent conda environment first}"
: "${PAOS_SKILL_ROOT:?Set PAOS_SKILL_ROOT to the extracted G1_D bundle}"
export CYCLONEDDS_HOME="${CONDA_PREFIX}/g1d-dds-0.10.2"
export LD_LIBRARY_PATH="${CYCLONEDDS_HOME}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
unset PYTHONPATH
exec "${CONDA_PREFIX}/bin/python" "${PAOS_SKILL_ROOT}/artifacts/g1d-runtime" \
  --host "${PAOS_G1D_HOST:-127.0.0.1}" --port "${PAOS_G1D_PORT:-19083}" \
  --control-profile "${PAOS_SKILL_ROOT}/profiles/real-g1d/control.json" \
  --journal "${PAOS_G1D_JOURNAL:-${PAOS_SKILL_ROOT}/run/actions.sqlite}" "$@"
