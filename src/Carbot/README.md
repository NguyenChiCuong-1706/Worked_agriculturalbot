Build worksapce steps:
1. **Create workspace**:
   - mkdir carbot_ws && cd carbot_ws
   - mkdir src && cd src
   - git clone https://github.com/NguyenChiCuong-1706/Carbot
2. **Build workspace**:
   - colcon build --symlink -install (_Add new code file need build again_)
   - source install/setup.bash
     
3. **Run workspace**: 
   - cd carbot_ws
   - source install/setup.bash
  
4. **Launch/Node commands**: (ros2 launch <package name> <launch file>)
   
**Launch robot**: (Robot_state_publisher + ros2_control)
  - ros2 launch carbot launch_realrobot.launch.py

**Run teleop node**: 
  - With `full_navigation_3d_nuc.launch.py` running: `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.3 -p turn:=1.0 -p stamped:=true -r cmd_vel:=/cmd_vel_teleop`
