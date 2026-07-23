#!/usr/bin/env bash
set -u

CAN_NAME="${1:-can0}"
BITRATE="${2:-1000000}"
USB_ADDRESS="${3:-}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${ROOT}/.ros/log"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/setup_can0_$(date +%Y%m%d_%H%M%S).log"

exec > >(tee -a "${LOG_FILE}") 2>&1

if [ "$(id -u)" -eq 0 ]; then
  SUDO=""
else
  SUDO="sudo"
fi

fail() {
  echo
  echo "ERROR: $*"
  echo "Log: ${LOG_FILE}"
  exit 1
}

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || fail "Missing command '$1'. Install it first."
}

echo "=== Piper CAN setup ==="
echo "Target interface: ${CAN_NAME}"
echo "Bitrate: ${BITRATE}"
echo "USB address filter: ${USB_ADDRESS:-<none>}"
echo "Log: ${LOG_FILE}"
echo

need_cmd ip
need_cmd awk
need_cmd grep
need_cmd ethtool

echo "[1/4] Loading gs_usb driver if available..."
${SUDO} modprobe gs_usb 2>/dev/null || echo "WARN: modprobe gs_usb failed or driver already unavailable."

echo "[2/4] Scanning CAN interfaces..."
CAN_LIST="$(ip -br link show type can 2>&1)"
SCAN_STATUS=$?
if [ "${SCAN_STATUS}" -ne 0 ]; then
  echo "${CAN_LIST}"
  fail "Cannot read CAN interfaces. If you are in a restricted container, run this on the host OS. Otherwise try: sudo -E bash scripts/setup_can0.sh ${CAN_NAME} ${BITRATE}"
fi

echo "${CAN_LIST:-<none>}"
mapfile -t IFACES < <(printf '%s\n' "${CAN_LIST}" | awk 'NF {print $1}')

if [ "${#IFACES[@]}" -eq 0 ]; then
  fail "No CAN interface detected. Plug in the USB-CAN adapter, check cable/driver, then rerun this script."
fi

SELECTED=""
if [ -n "${USB_ADDRESS}" ]; then
  for iface in "${IFACES[@]}"; do
    BUS_INFO="$(${SUDO} ethtool -i "${iface}" 2>/dev/null | awk '/bus-info:/ {print $2}')"
    echo "Interface ${iface}: bus-info=${BUS_INFO:-unknown}"
    if [ "${BUS_INFO}" = "${USB_ADDRESS}" ]; then
      SELECTED="${iface}"
      break
    fi
  done
  [ -n "${SELECTED}" ] || fail "No CAN interface matched USB address '${USB_ADDRESS}'."
else
  if [ "${#IFACES[@]}" -gt 1 ]; then
    echo
    echo "Multiple CAN interfaces detected:"
    for iface in "${IFACES[@]}"; do
      BUS_INFO="$(${SUDO} ethtool -i "${iface}" 2>/dev/null | awk '/bus-info:/ {print $2}')"
      echo "  ${iface}: bus-info=${BUS_INFO:-unknown}"
    done
    fail "Pass the USB address explicitly, for example: bash scripts/setup_can0.sh ${CAN_NAME} ${BITRATE} \"1-2:1.0\""
  fi
  SELECTED="${IFACES[0]}"
fi

echo "[3/4] Configuring ${SELECTED} -> ${CAN_NAME} at ${BITRATE}..."
${SUDO} ip link set "${SELECTED}" down || fail "Failed to bring ${SELECTED} down."
${SUDO} ip link set "${SELECTED}" type can bitrate "${BITRATE}" || fail "Failed to set CAN bitrate."

if [ "${SELECTED}" != "${CAN_NAME}" ]; then
  ${SUDO} ip link set "${SELECTED}" name "${CAN_NAME}" || fail "Failed to rename ${SELECTED} to ${CAN_NAME}."
fi

${SUDO} ip link set "${CAN_NAME}" up || fail "Failed to bring ${CAN_NAME} up."

echo "[4/4] Final interface state:"
ip -details link show "${CAN_NAME}" || fail "Configured interface exists but could not be displayed."

echo
echo "OK: ${CAN_NAME} is ready."
echo "Log: ${LOG_FILE}"
