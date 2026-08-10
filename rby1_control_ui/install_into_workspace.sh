#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 <rby1_ros2_ws_root>"
  echo "Example: $0 ~/rby1_ros2_ws"
}

if [[ $# -ne 1 ]]; then
  usage
  exit 2
fi

WORKSPACE_ROOT="$(realpath -m "$1")"
SOURCE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARENT_DIR="$WORKSPACE_ROOT/src/rby1-ros2"
TARGET_DIR="$PARENT_DIR/rby1_control_ui"

if [[ ! -d "$WORKSPACE_ROOT/src" ]]; then
  echo "ERROR: Workspace src directory not found: $WORKSPACE_ROOT/src" >&2
  exit 1
fi

if [[ ! -d "$PARENT_DIR" ]]; then
  echo "ERROR: Expected RB-Y1 source directory not found: $PARENT_DIR" >&2
  echo "Open the workspace shown in VS Code and pass its root directory." >&2
  exit 1
fi

if [[ -e "$TARGET_DIR" ]]; then
  echo "ERROR: Target already exists: $TARGET_DIR" >&2
  echo "Nothing was overwritten. Rename or remove the existing folder after reviewing it." >&2
  exit 1
fi

mkdir -p "$PARENT_DIR"
cp -a "$SOURCE_DIR" "$TARGET_DIR"
rm -f "$TARGET_DIR/install_into_workspace.sh"

cat <<MSG
Package files copied successfully.

Target:
  $TARGET_DIR

No build or UI execution was performed.
MSG
