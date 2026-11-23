#!/bin/bash

## Nav2
ros2 launch nav2_collision_monitor collision_monitor_node.launch.py \
  params_file:=/home/beinglab-kilted/carbot_ws/src/Carbot/fl_nav2_helper/config/collision_monitor.yaml \
  use_composition:=False \
  use_sim_time:=False \
  autostart:=True

  
wait
