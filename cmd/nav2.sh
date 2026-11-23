#!/bin/bash

ros2 launch nav2_collision_monitor collision_monitor_node.launch.py \
  params_file:=/home/beinglab-kilted/carbot_ws/src/Carbot/fl_nav2_helper/config/collision_monitor.yaml \
  use_composition:=False \
  use_sim_time:=False \
  autostart:=True &
sleep 2

ros2 run twist_mux twist_mux --ros-args \
  --params-file /home/beinglab-kilted/carbot_ws/src/Carbot/carbot/config/twist_mux.yaml \
  -r cmd_vel_out:=/cmd_vel &
sleep 2

ros2 launch carbot navigation_launch.py \
  use_sim_time:=false \
  params_file:=/home/beinglab-kilted/carbot_ws/src/Carbot/fl_nav2_helper/config/nav2_params.yaml
  
