#!/bin/bash
ros2 run tf2_ros static_transform_publisher \
0 0 0 0 0 0 base_link camera_init &
sleep 2

ros2 run tf2_ros static_transform_publisher \
0 0 0 0 0 0 camera_link base_link &
sleep 2
ros2 launch rtabmap_launch rtabmap.launch.py \
  rgb_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/camera/color/camera_info \
  odom_topic:=/Odometry \
  subscribe_scan_cloud:=true \
  scan_cloud_topic:=/cloud_registered_body \
  visual_odometry:=false \
  icp_odometry:=false \
  frame_id:=body \
  approx_sync:=true \
  queue_size:=50 \
  rviz:=false \
  rtabmap_viz:=true \
  rtabmap_args:="\
    --database_path /home/beinglab/.ros/labhall.db \
    --Optimizer/Strategy 1 \
    --Optimizer/GravitySigma 0.3 \
    --RGBD/NeighborLinkRefining true \
    --RGBD/ProximityBySpace true \
    --RGBD/ProximityMaxGraphDepth 50 \
    --RGBD/MaxDepth 3 \
    --Grid/Sensor 2 \
    --Grid/RangeMax 4.0 \
    --Grid/RayTracing true \
    --Grid/CellSize 0.03 \
    --Grid/DepthDecimation 2 \
    --Grid/NormalsSegmentation false \
    --Grid/MinClusterSize 20 \
    --Mem/STMSize 30 \
    --Mem/NotLinkedNodesKept false \
    --Rtabmap/TimeThr 0 \
    --Rtabmap/DetectionRate 0"
    
# rtabmap-databaseViewer /home/beinglab-kilted/carbot_ws/maps/run1.db






