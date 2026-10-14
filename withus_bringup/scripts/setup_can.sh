#!/usr/bin/env bash
set -euo pipefail

if (( $# > 2 )); then
    echo "Usage: $0 [interface] [bitrate]" >&2
    exit 1
fi

can_interface="${1:-can0}"
can_bitrate="${2:-500000}"

if [[ ! "$can_bitrate" =~ ^[1-9][0-9]*$ ]]; then
    echo "Bitrate must be a positive integer." >&2
    exit 1
fi

# 설정 전에 장치의 존재 여부와 CAN 인터페이스인지 확인
interface_info=$(ip -details link show dev "$can_interface")
if [[ "$interface_info" != *"link/can"* ]]; then
    echo "Not a CAN interface: $can_interface" >&2
    exit 1
fi

printf 'Configuring %s at %s bit/s...\n' "$can_interface" "$can_bitrate"

sudo ip link set dev "$can_interface" down
sudo ip link set dev "$can_interface" type can bitrate "$can_bitrate"
sudo ip link set dev "$can_interface" up

ip -details -statistics link show dev "$can_interface"