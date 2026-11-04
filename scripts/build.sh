#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
colcon build --packages-select construction_interfaces construction_autonomy
