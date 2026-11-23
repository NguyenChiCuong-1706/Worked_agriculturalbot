#!/bin/bash

ros2 launch carbot full_mapping_nuc.launch.py \
  use_sim_time:=false \
  fast_lio_config_file:=mid360.yaml \
  slam_params_file:=/home/beinglab-kilted/carbot_ws/src/Carbot/fl_nav2_helper/config/mapper_params_online_async.yaml \
  pointcloud_to_laserscan_params_file:=/home/beinglab-kilted/carbot_ws/src/Carbot/fl_nav2_helper/config/pointcloud_to_laserscan.yaml

