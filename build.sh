#!/bin/bash
set -Eeuo pipefail
echo "building!"
colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
#gnome-terminal -- bash -c "colcon build --symlink-install"
