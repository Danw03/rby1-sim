#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
required=(
  package.xml
  setup.py
  setup.cfg
  resource/rby1_control_ui
  config/default.yaml
  launch/control_ui.launch.py
  rby1_control_ui/__init__.py
  rby1_control_ui/qt_compat.py
  rby1_control_ui/ros_backend.py
  rby1_control_ui/main_window.py
  rby1_control_ui/main.py
)

missing=0
for item in "${required[@]}"; do
  if [[ ! -f "$ROOT/$item" ]]; then
    echo "MISSING: $item"
    missing=1
  else
    echo "OK: $item"
  fi
done

if [[ $missing -ne 0 ]]; then
  echo "File-layout check failed." >&2
  exit 1
fi

echo "File-layout check passed. No ROS node or GUI was executed."
