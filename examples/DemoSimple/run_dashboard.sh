#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-jazzy}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS setup file not found: ${ROS_SETUP}" >&2
  echo "Set ROS_DISTRO, or source ROS manually and run raftros_dynamic_dashboard.py directly." >&2
  exit 1
fi

# ROS setup scripts may reference unset variables internally. Keep nounset for
# this wrapper, but relax it while sourcing the generated ROS environment.
set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
set -u

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-${SCRIPT_DIR}/logs}"
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset FASTDDS_DEFAULT_PROFILES_FILE
mkdir -p "${ROS_LOG_DIR}"

if [[ "${ROS_DISTRO_NAME}" == "humble" ]]; then
  export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
else
  unset ROS_LOCALHOST_ONLY || true
fi

exec python3 -u "${SCRIPT_DIR}/raftros_dynamic_dashboard.py" "$@"
