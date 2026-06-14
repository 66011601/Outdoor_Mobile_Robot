# Task: Fix Odometry Drift in EKF Configuration

## Context — Read CLAUDE.md First
Read CLAUDE.md in the workspace root for full project context. This file describes the specific debugging task.

## The Problem
The fused odometry drifts over time, causing the robot's perceived position to diverge from its actual position. This breaks obstacle avoidance because the costmap becomes misaligned with the world. The drift occurs in simulation, which is diagnostic — it means the problem is in the EKF configuration, not sensor hardware.

## Understanding the Root Cause

### Key insight: GPS does NOT drift, VIO does
- **VIO drift** accumulates because each frame builds on the previous frame's error. Small errors compound over distance.
- **GPS does NOT drift** — each reading is an absolute position from satellites, independent of previous readings. Error is bounded by GPS spec (cm for RTK Fixed, meters for Single fix).

### Why the current setup drifts
If the EKF fuses VIO and GPS but weights VIO too heavily, the drift from VIO pulls the fused estimate away from accurate GPS. The output looks like "everything is drifting" even though GPS itself is perfectly accurate.

### Correct mental model for outdoor robots
- **GPS** = ground truth for global position (when RTK Fixed)
- **VIO** = smooth interpolation between GPS updates (~30Hz fills gaps in ~5Hz GPS)
- **IMU** = orientation, especially yaw

## In Simulation (Current Focus)
Simulated GPS is PERFECT by default unless noise is explicitly configured. If drift is visible in simulation, it's 100% an EKF configuration problem (not sensor noise). Fix it in simulation first; the same fix applies on real hardware with additional noise tuning.

## Files to Modify
```
~/internship_project/diff_drive_ws/src/dcvg_robot/config/
├── dual_ekf_navsat_params.yaml    ← PRIMARY FIX TARGET
└── nav2_no_map_params.yaml         ← May need adjustment if costmap behavior changes
```

## The Fix — Try in This Order

### Fix #1 (Recommended first): GPS-only in ekf_global, VIO-only in ekf_local
Architectural separation — clearest results, easiest to reason about.

**ekf_local** (publishes odom → base_footprint, smooth local frame for Nav2 local costmap):
- Use VIO (`/vo`) as primary odometry source
- Use IMU for orientation/yaw
- Do NOT use GPS here — keep it smooth and high-frequency

**ekf_global** (publishes map → odom, GPS-corrected global frame for Nav2 global costmap):
- Use GPS (`/odometry/gps` from navsat_transform) as the position source
- Use IMU for orientation/yaw
- Do NOT use VIO here — GPS is the absolute truth, no need to mix in drift-prone data

This guarantees zero drift in the global frame because GPS is the only position source. VIO is contained to the local frame where its smoothness is useful and drift between GPS updates is negligible.

### Fix #2: If VIO must be in ekf_global, reduce its weight
If keeping VIO in ekf_global is required for some reason:
- Increase process noise (`process_noise_covariance`) for VIO-related state variables
- Decrease process noise for GPS-related state variables
- This tells the EKF: "trust GPS more, trust VIO less"

### Fix #3: Check pose rejection thresholds
`navsat_transform_node` and EKF have parameters that reject GPS updates if they jump too far from the current estimate:
- If too strict, GPS corrections get rejected when VIO has drifted, and the drift compounds forever
- Loosen these thresholds so GPS can always correct drift, no matter how large

Parameters to check:
- `transform_timeout` in navsat_transform
- Sensor differential mode flags (`odom0_differential`, `odom1_differential`) — if `true`, only velocity changes are fused, not absolute positions, which hides drift

### Fix #4: Verify GPS source is correctly fed to ekf_global
Verify the topic chain:
```
/gps/fix (NavSatFix from Gazebo)
  → navsat_transform_node
  → /odometry/gps (nav_msgs/Odometry)
  → ekf_global as odom1 (or odom2 if VIO is odom1)
```
Confirm `odom1_config` for the GPS source has position fields enabled:
```yaml
odom1_config: [true, true, false,    # x, y position — MUST be true
               false, false, false,
               false, false, false,
               false, false, false,
               false, false, false]
```

## Testing Procedure

### Before any changes
1. Launch simulation: `ros2 launch dcvg_robot gps_waypoint_follower.launch.py`
2. Drive the robot in a long straight line (50-100m) using teleop or a waypoint
3. Record `/odometry/global` and `/gps/fix` to a bag file
4. Plot the trajectory — measure the gap between fused position and GPS truth at end of run
5. This is your baseline drift

### After applying fix
1. Rebuild: `colcon build --packages-select dcvg_robot && source install/setup.bash`
2. Same test: drive 50-100m, record same topics
3. Compare drift — should be near zero with Fix #1 applied
4. Test obstacle avoidance — drive around the world, verify costmap aligns with actual obstacles

## Diagnostic Commands

```bash
# Check current EKF output vs GPS truth
ros2 topic echo /odometry/global --field pose.pose.position
ros2 topic echo /gps/fix --field latitude

# Check what TF the EKFs are publishing
ros2 run tf2_tools view_frames

# Watch covariance values (high covariance = low trust)
ros2 topic echo /odometry/global --field pose.covariance

# Check if VIO is publishing reasonable data
ros2 topic echo /vo --field pose.pose.position
```

## What NOT to Do
- Do NOT try to "reset" odometry periodically — that's treating the symptom, not the cause
- Do NOT shut down and restart the system at each stop — not practical, and a properly tuned EKF doesn't need it
- Do NOT add manual position corrections — the EKF should handle this automatically
- Do NOT remove VIO entirely — it's still needed for ekf_local (smooth motion between GPS updates)
- Do NOT increase Nav2 update frequencies to "compensate" — that wastes compute and doesn't fix the root cause

## Goal
After this fix, in simulation:
- The robot's position estimate should NEVER drift more than ~1m from true GPS position
- Drift should not grow over time or distance
- The map/odom frames should remain stable relative to the world
- Obstacle avoidance should remain effective indefinitely without any system restarts