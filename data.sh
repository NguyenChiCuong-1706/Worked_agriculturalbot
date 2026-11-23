#!/usr/bin/env bash

# Record the inputs and outputs of navigation_3d.sh for offline localization
# evaluation and RViz replay.
set -Eeo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/install/setup.bash"
set -u

# Usage:
#   ./data.sh [run_name]
#
# Optional environment variables:
#   BAG_DIR=/path/to/output       Parent directory (default: ./bags)
#   GROUND_TRUTH_TOPIC=/mocap/... Pose/odometry reference for accuracy metrics
#   EXTRA_TOPICS="/foo /bar"      Additional space-separated topics
RUN_NAME="${1:-navigation_3d}"
BAG_DIR="${BAG_DIR:-${SCRIPT_DIR}/bags}"
STAMP="$(date +%Y%m%d_%H%M%S)"
BAG_PATH="${BAG_DIR}/${RUN_NAME}_${STAMP}"

mkdir -p "${BAG_DIR}"

topics=(
  # Time and coordinate frames. These are essential for RViz replay.
  /clock
  /tf
  /tf_static

  # Raw sensors and FAST-LIO inputs/outputs.
  /Odometry
  /cloud_registered
  /cloud_registered_body
  /scan

  # FAST-LIO localization outputs: raw vs map-corrected trajectory and clouds.
  /pose_stamped
  /ori_odom
  /ori_path
  /corrected_odom
  /corrected_path
  /corrected_current_pcd
  /saved_map
  /map_match
  /src
  /dst
  /coarse_aligned_quatro
  /fine_aligned_nano_gicp

  # Nav2 map, planner/controller output, and costmaps displayed by RViz.
  /map
  /map_updates
  /plan
  /local_plan
  /global_costmap/costmap
  /global_costmap/costmap_updates
  /global_costmap/published_footprint
  /local_costmap/costmap
  /local_costmap/costmap_updates
  /local_costmap/published_footprint
  /collision_monitor/collision_points_marker
  /polygon_stop
  /polygon_slowdown

  # Navigation requests and RViz annotations.
  /goal_pose
  /initialpose
  /clicked_point
  /waypoints
  /route_graph
  /following_path
  /goal_pose_array
  /controller_selector
  /goal_checker_selector
  /planner_selector
  /progress_checker_selector
  /smoother_selector

  # Original camera streams, useful for visually inspecting failures.
  /camera/camera/color/image_raw
  /camera/camera/color/camera_info
  /camera/camera/depth/image_rect_raw
  /camera/camera/depth/camera_info

  # Diagnostics useful when correlating lifecycle/configuration changes.
  /parameter_events
  /rosout
)

if [[ -n "${GROUND_TRUTH_TOPIC:-}" ]]; then
  topics+=("${GROUND_TRUTH_TOPIC}")
fi

if [[ -n "${EXTRA_TOPICS:-}" ]]; then
  # Intentional word splitting: EXTRA_TOPICS is a space-separated topic list.
  read -r -a extra_topics <<< "${EXTRA_TOPICS}"
  topics+=("${extra_topics[@]}")
fi

echo "Recording navigation/localization data to: ${BAG_PATH}"
if [[ -z "${GROUND_TRUTH_TOPIC:-}" ]]; then
  echo "Note: no independent ground truth configured; set GROUND_TRUTH_TOPIC for absolute accuracy evaluation."
fi
echo "Stop cleanly with Ctrl-C so the bag index is finalized."

exec ros2 bag record \
  --storage mcap \
  --output "${BAG_PATH}" \
  --include-unpublished-topics \
  --topics "${topics[@]}" \
  --actions \
    /navigate_to_pose \
    /navigate_through_poses \
    /follow_waypoints \
    /compute_path_to_pose \
    /compute_path_through_poses \
    /smooth_path \
    /follow_path \
  --custom-data "run_name=${RUN_NAME}" "stack=navigation_3d"
