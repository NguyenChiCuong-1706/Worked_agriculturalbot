#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/install/setup.bash"

NAV2_PARAMS_FILE="${NAV2_PARAMS_FILE:-${SCRIPT_DIR}/src/Carbot/fl_nav2_helper/config/nav2_params.yaml}"

TEASERPP_LIB_DIR="${SCRIPT_DIR}/teaserpp_install/lib"
if [ -d "${TEASERPP_LIB_DIR}" ]; then
  export LD_LIBRARY_PATH="${TEASERPP_LIB_DIR}:${LD_LIBRARY_PATH:-}"
fi

ros2 launch carbot full_navigation_3d_nuc.launch.py \
  use_sim_time:=false \
  rviz:=true \
  fast_lio_config_file:=mid360.yaml \
  localization_config_file:="${SCRIPT_DIR}/src/3D_SLAM/FAST-LIO-Localization-SC-QN2/config/config.yaml" \
  saved_map_bag:="${SCRIPT_DIR}/maps/greenhouse_2026_10_06-14_22_13/result.bag" \
  map_yaml_file:="${SCRIPT_DIR}/maps/map_grid_filtered.yaml" \
  nav2_params_file:="${NAV2_PARAMS_FILE}" \
  collision_monitor_params_file:="${SCRIPT_DIR}/src/Carbot/fl_nav2_helper/config/collision_monitor.yaml" \
  pointcloud_to_laserscan_params_file:="${SCRIPT_DIR}/src/Carbot/fl_nav2_helper/config/pointcloud_to_laserscan.yaml"
