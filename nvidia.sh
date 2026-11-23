#!/usr/bin/env bash
BAG_NAME="nvidia_data_$(date +'%Y%m%d_%H%M%S')"

ros2 bag record \
  -o "$BAG_NAME" \
  /clock \
  /tf \
  /tf_static \
  /Odometry \
  /cloud_registered \
  /cloud_registered_body \
  /camera/camera/color/image_raw \
  /camera/camera/color/camera_info \
  /camera/camera/depth/image_rect_raw \
  /camera/camera/depth/camera_info \



