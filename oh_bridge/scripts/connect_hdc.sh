#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HDC_DIR="${HDC_DIR:-$HOME/ohos-robot-toolchain/linux/toolchains}"
OH_PORT="${OH_PORT:-55555}"

if [[ ! -x "$HDC_DIR/hdc" ]]; then
    echo "Error: hdc not found at:"
    echo "  $HDC_DIR/hdc"
    exit 1
fi

export PATH="$HDC_DIR:$PATH"

if [[ -n "${OH_IP:-}" ]]; then
    TARGET_IP="$OH_IP"
else
    TARGET_IP="$("$SCRIPT_DIR/get_oh_ip.sh")"
fi

echo "Connecting to OpenHarmony at ${TARGET_IP}:${OH_PORT}..."
hdc tconn "${TARGET_IP}:${OH_PORT}"

echo
echo "Connected targets:"
hdc list targets -v

echo
echo "Opening OpenHarmony shell..."
hdc shell
