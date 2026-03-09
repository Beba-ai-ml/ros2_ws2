# SAC Driver (ROS2 Workspace)
Autonomous racing vehicle inference node — runs a trained SAC (Soft Actor-Critic) neural network policy on a Jetson Nano, reading lidar+odom and outputting Ackermann drive commands to a VESC motor controller. Part of an F1TENTH-style racing platform.

## Context Files
- `KNOWLEDGE.md` - Architecture, key files, stack, conventions, how to run, debugging lessons
- `STATE.md` - Current state, recent changes, decisions, known issues, gotchas

## Workspace Structure
- `src/sac_driver/` — SAC inference ROS2 package (main project)
- `src/f1tenth_stack/` — F1TENTH bringup, joy_mode_manager, teleop
- `src/ackermann_mux/` — Ackermann command multiplexer (C++)
- `src/slam_toolbox/` — SLAM
- `src/particle_filter/` — Localization
- `src/pure_pursuit/` — Pure pursuit controller
- `src/stanley_avoidance/` — Stanley controller with obstacle avoidance

## Related Systems
- **ROS2 Control Panel:** `/home/laptop/ros2_panel/` (GTK3 app for starting/stopping nodes)
- **NFS shared folder:** `/home/laptop/shared` ↔ PC `/home/beba/shared` (model weights, logs)
- **PC training code:** `/home/beba/occupancy_racer/Soft_Actor_Critic_2/`

## Documentation
- `README.md` — Professional GitHub-ready README
- `DOCUMENTATION.md` — Full module-level reference documentation

Last updated: 2026-03-09
