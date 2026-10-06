# DCVG Autonomous Rover

Read `README.md` first. It is the single source of truth for architecture, calibrated values and debugging.

- ROS 2 Humble, both packages are ament_python. Build with `colcon build` from the workspace root.
- `dcvg_bringup` is the only runnable stack (real hardware). `dcvg_robot` holds the URDF, the web app and the waypoint follower.
- There is no simulation. Changes can only be validated on the robot.
- Never mix conda with ROS 2 in this workspace.
- The code carries no comments by design. Put explanations in `README.md`.
