#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

source /opt/ros/jazzy/setup.bash
source "$WORKSPACE_ROOT/install/setup.bash"

BASE="$SCRIPT_DIR"

echo "======================================"
echo "1. VENT 30 s"
echo "======================================"

python3 "$BASE/vent_to_zero.py"

echo
echo "======================================"
echo "2. PRESSURE ZERO CALIBRATION"
echo "======================================"

python3 "$BASE/calibrate_pressure.py"

echo
echo "======================================"
echo "3. FINAL SENSOR CHECK"
echo "======================================"

ros2 topic echo /pico_bridge/sensor_frame --once
