#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname -- "${SCRIPT_DIR}")"

source /opt/ros/humble/setup.bash
set -u

cd "${WORKSPACE_DIR}"
colcon build --symlink-install "$@"

echo
echo "编译完成。使用下面的命令加载工作空间环境："
echo "source ${WORKSPACE_DIR}/install/setup.bash"
