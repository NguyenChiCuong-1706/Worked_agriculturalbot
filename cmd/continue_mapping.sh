#!/bin/bash


ros2 service call /slam_toolbox/deserialize_map slam_toolbox/srv/DeserializePoseGraph "{
  filename: '/home/beinglab-kilted/carbot_ws/premap_final_serial',
  match_type: 1,
  initial_pose: {x: 0.0, y: 0.0, theta: 0.0}
}"
