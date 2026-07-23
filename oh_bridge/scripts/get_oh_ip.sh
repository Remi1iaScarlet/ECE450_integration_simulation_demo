#!/usr/bin/env bash

set -Eeuo pipefail

KERNEL_LOG="${KERNEL_LOG:-$HOME/OpenHarmony/kernel.log}"

if [[ ! -f "$KERNEL_LOG" ]]; then
    echo "Error: kernel log not found:"
    echo "  $KERNEL_LOG"
    exit 1
fi

OH_IP="$(
    grep "my address is" "$KERNEL_LOG" \
    | tail -n 1 \
    | sed -E 's/.*my address is ([0-9.]+).*/\1/'
)"

if [[ -z "$OH_IP" ]]; then
    echo "OpenHarmony IP has not been found yet."
    echo "Make sure QEMU has completed booting."
    exit 1
fi

echo "$OH_IP"
