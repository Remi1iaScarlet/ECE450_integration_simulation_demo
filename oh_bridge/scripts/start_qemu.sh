#!/usr/bin/env bash

set -Eeuo pipefail

OH_DIR="${OH_DIR:-$HOME/OpenHarmony}"
QEMU_SCRIPT="$OH_DIR/qemu_run_client.sh"
KERNEL_LOG="$OH_DIR/kernel.log"

if [[ ! -f "$QEMU_SCRIPT" ]]; then
    echo "Error: QEMU startup script not found:"
    echo "  $QEMU_SCRIPT"
    exit 1
fi

if [[ ! -x "$QEMU_SCRIPT" ]]; then
    chmod +x "$QEMU_SCRIPT"
fi

echo "Starting OpenHarmony QEMU..."
echo "Working directory: $OH_DIR"
echo "Kernel log: $KERNEL_LOG"
echo
echo "Keep this terminal open while QEMU is running."
echo

cd "$OH_DIR"
sudo "$QEMU_SCRIPT" 2>&1 | tee "$KERNEL_LOG"
