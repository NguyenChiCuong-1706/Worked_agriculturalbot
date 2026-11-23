#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/install/setup.bash"

ros2 launch carbot full_mapping_3d_nuc.launch.py \
  use_sim_time:=false \
  fast_lio_config_file:=mid360.yaml \
  sam_config_file:="${SCRIPT_DIR}/src/3D_SLAM/FAST-LIO-SAM-QN2/config/config.yaml"
