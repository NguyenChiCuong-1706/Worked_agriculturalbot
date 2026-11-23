#!/bin/bash

ros2 launch livox_ros_driver2 msg_MID360_launch.py &
sleep 2

ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=true \
  align_depth.enable:=true \
  pointcloud.enable:=true \
  enable_gyro:=false \
  enable_accel:=false\
  rgb_camera.color_profile:=640x480x10 \
  depth_module.depth_profile:=640x480x10
  

