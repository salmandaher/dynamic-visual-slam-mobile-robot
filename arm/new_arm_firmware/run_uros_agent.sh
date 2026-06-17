#!/usr/bin/env bash
# Run the micro-ROS agent for the ESPMax espmax_passthrough firmware on this machine
# WITHOUT sudo / snap confinement.
#
# Why this exists: micro-ros-agent is installed as a STRICT-confined snap (eprosima),
# and this is a CLASSIC desktop with no serial-port slot, so the confined snap cannot
# open /dev/ttyUSB0 (errno 13). Re-running under sudo does NOT help (AppArmor denies
# the device regardless of uid). Instead we invoke the snap's INNER binary directly
# (no snap-confine), which inherits the user's dialout access. The snap is Foxy/core20
# based, so we add its bundled ROS+app libs, and shim the two Focal-only libs
# (libssl/libcrypto 1.1) that jammy lacks -- WITHOUT putting core20's libc on the path
# (that would shadow the host glibc and break everything).
#
# Usage:  new_arm_firmware/run_uros_agent.sh [serial_device]   (default /dev/ttyUSB0)
set -euo pipefail

DEV="${1:-/dev/ttyUSB0}"
BAUD="${BAUD:-115200}"
SNAP=/snap/micro-ros-agent/current
CORE=/snap/core20/current
BIN="$SNAP/opt/ros/snap/lib/micro_ros_agent/micro_ros_agent"

[ -e "$BIN" ] || { echo "agent binary not found: $BIN (is the micro-ros-agent snap installed?)" >&2; exit 1; }
[ -e "$DEV" ] || { echo "serial device not found: $DEV (is the ESP32 plugged in?)" >&2; exit 1; }

# Persistent shim dir (on root fs so it survives DataDrive unmount); symlinks point at
# core20's Focal libssl/libcrypto 1.1, which are always mounted with the base snap.
SHIM="$HOME/.local/lib/uros_compat"
mkdir -p "$SHIM"
ln -sf "$CORE/usr/lib/x86_64-linux-gnu/libssl.so.1.1"    "$SHIM/"
ln -sf "$CORE/usr/lib/x86_64-linux-gnu/libcrypto.so.1.1" "$SHIM/"

export LD_LIBRARY_PATH="$SNAP/opt/ros/snap/lib:$SNAP/opt/ros/foxy/lib:$SNAP/usr/lib/x86_64-linux-gnu:$SHIM"
export FASTRTPS_DEFAULT_PROFILES_FILE="$SNAP/usr/share/fastdds_no_shared_memory.xml"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export AMENT_PREFIX_PATH="$SNAP/opt/ros/foxy"

echo "starting micro-ROS agent: serial $DEV @ ${BAUD} (Foxy snap binary, unconfined)"
exec "$BIN" serial --dev "$DEV" -b "$BAUD" "${@:2}"
