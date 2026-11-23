#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/install/setup.bash"

CMD_VEL_TOPIC="/cmd_vel"
if ros2 node list 2>/dev/null | grep -Fxq "/twist_mux"; then
  CMD_VEL_TOPIC="/cmd_vel_teleop"
fi

echo "Publishing teleop commands to ${CMD_VEL_TOPIC}"

exec ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p speed:=0.3 -p turn:=1.0 \
  -p stamped:=true \
  -r cmd_vel:="${CMD_VEL_TOPIC}"
