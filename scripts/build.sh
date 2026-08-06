#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname -- "${SCRIPT_DIR}")"

source /opt/ros/humble/setup.bash
set -u

cd "${WORKSPACE_DIR}"
colcon build --symlink-install "$@"

echo
echo "Build completed. Source the workspace environment with:"
echo "source ${WORKSPACE_DIR}/install/setup.bash"
