#!/usr/bin/env bash
set -e
ui_project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ui_project_root"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
exec ros2 launch nexarm_description angle_panel.launch.py
