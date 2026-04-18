#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_ROOT="$REPO_ROOT/backend/catkin_ws"
PACKAGE_NAME="manual_loop_closure_backend"

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "[manual-loop-closure] /opt/ros/noetic/setup.bash not found." >&2
  echo "[manual-loop-closure] Install ROS Noetic first." >&2
  exit 1
fi

source /opt/ros/noetic/setup.bash

echo "[manual-loop-closure] Workspace: $WS_ROOT"

mkdir -p "$WS_ROOT/src"

catkin config \
  --workspace "$WS_ROOT" \
  --extend /opt/ros/noetic \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release

catkin build --workspace "$WS_ROOT" "$PACKAGE_NAME"

echo
echo "[manual-loop-closure] Build complete."
echo "[manual-loop-closure] Binary:"
echo "  $WS_ROOT/devel/lib/$PACKAGE_NAME/manual_loop_optimize"
