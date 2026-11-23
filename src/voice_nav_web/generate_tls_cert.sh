#!/usr/bin/env bash
set -euo pipefail

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CERT_FILE="${VOICE_NAV_CERT:-${APP_DIR}/cert.pem}"
KEY_FILE="${VOICE_NAV_KEY:-${APP_DIR}/key.pem}"
CERT_DAYS="${VOICE_NAV_CERT_DAYS:-825}"
HOST_NAME="${VOICE_NAV_HOSTNAME:-$(hostname)}"

san_entries="DNS:${HOST_NAME},DNS:${HOST_NAME}.local,DNS:voice-nav-local,DNS:localhost,IP:127.0.0.1"

for address in $(hostname -I 2>/dev/null || true); do
  if [[ "${address}" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]]; then
    san_entries+=",IP:${address}"
  fi
done

openssl req \
  -x509 \
  -newkey rsa:2048 \
  -sha256 \
  -nodes \
  -days "${CERT_DAYS}" \
  -subj "/CN=${HOST_NAME}" \
  -addext "subjectAltName=${san_entries}" \
  -keyout "${KEY_FILE}" \
  -out "${CERT_FILE}"

chmod 600 "${KEY_FILE}"
chmod 644 "${CERT_FILE}"

echo "Generated certificate: ${CERT_FILE}"
echo "Generated private key: ${KEY_FILE}"
echo "Subject Alternative Names: ${san_entries}"
