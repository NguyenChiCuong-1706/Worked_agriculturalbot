#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP_DIR="${WORKSPACE_DIR}/src/voice_nav_web"
VENV_PYTHON="${APP_DIR}/.venv/bin/python"
ROSBRIDGE_LOG="/tmp/voice_nav_rosbridge.log"

VOICE_NAV_CERT="${VOICE_NAV_CERT:-${APP_DIR}/cert.pem}"
VOICE_NAV_KEY="${VOICE_NAV_KEY:-${APP_DIR}/key.pem}"
ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9090}"
DEFAULT_TLS_CERT="${APP_DIR}/cert.pem"
DEFAULT_TLS_KEY="${APP_DIR}/key.pem"

if [[ "${VOICE_NAV_CERT}" != /* ]]; then
  VOICE_NAV_CERT="${APP_DIR}/${VOICE_NAV_CERT}"
fi
if [[ "${VOICE_NAV_KEY}" != /* ]]; then
  VOICE_NAV_KEY="${APP_DIR}/${VOICE_NAV_KEY}"
fi

# Replace only the bundled development certificate when it is missing a
# Subject Alternative Name. User-provided certificates are never modified.
if [ "${VOICE_NAV_CERT}" = "${DEFAULT_TLS_CERT}" ] && \
   [ "${VOICE_NAV_KEY}" = "${DEFAULT_TLS_KEY}" ] && \
   { [ ! -f "${VOICE_NAV_CERT}" ] || \
     ! openssl x509 -in "${VOICE_NAV_CERT}" -noout -text 2>/dev/null \
       | grep -q "X509v3 Subject Alternative Name"; }; then
  echo "Generating a private-network TLS certificate with hostname/IP SANs..."
  VOICE_NAV_CERT="${VOICE_NAV_CERT}" \
  VOICE_NAV_KEY="${VOICE_NAV_KEY}" \
    "${APP_DIR}/generate_tls_cert.sh"
fi

if [ ! -r "${VOICE_NAV_CERT}" ]; then
  echo "Missing or unreadable TLS certificate: ${VOICE_NAV_CERT}" >&2
  exit 1
fi
if [ ! -r "${VOICE_NAV_KEY}" ]; then
  echo "Missing or unreadable TLS key: ${VOICE_NAV_KEY}" >&2
  exit 1
fi

export ROS_WS_SETUP="${ROS_WS_SETUP:-${WORKSPACE_DIR}/install/setup.bash}"
export ROBOT_TASK_CONFIG="${ROBOT_TASK_CONFIG:-${APP_DIR}/navigation_config.json}"
export NAV2_PARAMS_FILE="${NAV2_PARAMS_FILE:-${WORKSPACE_DIR}/src/Carbot/fl_nav2_helper/config/nav2_params.yaml}"
export VOICE_NAV_CERT
export VOICE_NAV_KEY
export ROSBRIDGE_PORT
export ROSBRIDGE_SCHEME="${ROSBRIDGE_SCHEME:-wss}"

set +u
source /opt/ros/kilted/setup.bash
source "${ROS_WS_SETUP}"
set -u

ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
  ssl:=true \
  certfile:="${VOICE_NAV_CERT}" \
  keyfile:="${VOICE_NAV_KEY}" \
  port:="${ROSBRIDGE_PORT}" \
  > "${ROSBRIDGE_LOG}" 2>&1 &
ROSBRIDGE_PID=$!

cleanup() {
  if kill -0 "${ROSBRIDGE_PID}" 2>/dev/null; then
    kill "${ROSBRIDGE_PID}" 2>/dev/null || true
    wait "${ROSBRIDGE_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "secure rosbridge started on wss://0.0.0.0:${ROSBRIDGE_PORT}"
echo "rosbridge log: ${ROSBRIDGE_LOG}"

cd "${APP_DIR}"

if [ -x "${VENV_PYTHON}" ]; then
  "${VENV_PYTHON}" server.py
else
  python3 server.py
fi
