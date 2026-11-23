#!/bin/bash

BAG_NAME="nav_quantitive_data_$(date +'%Y%m%d_%H%M%S')"

ros2 bag record \
  -o "$BAG_NAME" \
  /tf \
  /tf_static \
  /plan \
  /plan_smoothed \
  /goal_pose \
  /Odometry \
  /ori_odom \
  /ori_path \
  /corrected_odom \
  /route_graph
