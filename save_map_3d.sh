#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/install/setup.bash"

ros2 topic pub --once /save_dir std_msgs/msg/String "{data: '${SCRIPT_DIR}/maps'}"
