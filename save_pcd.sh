#!/usr/bin/env bash

ros2 topic pub --once /save_dir std_msgs/msg/String \
  "{data: '/home/beinglab-kilted/carbot_ws/'}"
