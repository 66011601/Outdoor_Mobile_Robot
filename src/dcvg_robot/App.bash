#!/bin/bash

# 1. Source the base ROS 2 installation (replace 'humble' with your distro)
source ~/.bashrc
sleep 0.5

# 2. Source your local workspace overlay
source /home/thana/internship_project/diff_drive_ws/install/setup.bash
sleep 0.5

xdg-open http://127.0.0.1:5000

# 3. Run the desired ros2 command
ros2 run dcvg_robot osm_route_app
