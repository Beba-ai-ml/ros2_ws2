# SAC Driver (ROS2 Workspace)

Autonomous racing vehicle inference node — runs a trained SAC (Soft Actor-Critic) neural network policy on an NVIDIA Jetson Orin Nano, reading lidar + odom and outputting Ackermann drive commands to a VESC motor controller. Part of an F1TENTH-style racing platform. The repository is self-contained and portable: clone it on any Jetson with JetPack 5.x, run `./install.sh`, and the car drives.

## Context Files
- `KNOWLEDGE.md` — Architecture, key files, stack, conventions, how to run, debugging lessons
- `STATE.md` — Current state, recent changes, decisions, known issues, gotchas

## Workspace Structure
- `src/sac_driver/` — SAC inference ROS2 package (main project), weights in `weights/`
- `src/f1tenth_stack/` — bringup (`bringup_launch3.py`), `joy_mode_manager`, vesc/joy/mux configs
- `src/ackermann_mux/` — Ackermann command multiplexer (C++)
- `src/sllidar_ros2/` — RPLiDAR driver (vendored upstream)
- `src/slam_toolbox/` — SLAM source, NOT built (apt version is used); its config file is used
- `src/particle_filter/`, `src/range_libc/` — localization (vendored upstream)
- `src/pure_pursuit/`, `src/stanley_avoidance/`, `src/waypoint_generator/` — classical controllers
- `src/gap_follow/`, `src/wall_follow/`, `src/safety_node/`, `src/scan_matching/` — F1TENTH lab code
- `local_python/` — patched `joy_teleop` (PYTHONPATH shim, required by bringup) + `gpio_shutdown.py`
- `ros2_panel/` — GTK3 control panel (moved into the repo from `~/ros2_panel`)
- `scripts/` — `key_drive.sh`, `key_drive.py`, `key_drive_boot.sh`, `key_drive_test.py`, `ledy*.py`
- `tools/` — `vesc/vesc_config_upload.py` + configs/params, `lidar_diag.py`, `lidar_test.py`
- `system/` — `udev/99-f1tenth.rules`, `systemd/*.service.in`, `sudoers.d/f1tenth.in`, `desktop/`
- `config/` — `ackermann_mux.yaml`, `joy_mode_manager.yaml`, `slam_rviz.rviz`, `frames.gv`
- `maps/` — occupancy grids (.pgm + .yaml)
- `docs/` — `SETUP_NEW_JETSON.md`, `TROUBLESHOOTING.md`, `3d-mapper/` (chassis CAD)
- `legacy/system_web/` — old Flask web panel, unmaintained
- `install.sh`, `requirements.txt` — one-shot installer and Python deps

## Related Systems
- **NFS shared folder (optional):** `/home/laptop/shared` ↔ PC `/home/beba/shared` (192.168.1.102) — weights/logs exchange only; inference does not depend on it
- **PC training code:** `/home/beba/occupancy_racer/Soft_Actor_Critic_2/` → https://github.com/Beba-ai-ml/occupancy-racer-sac2
- **Remote desktop:** NoMachine (`nxserver`), `DISPLAY=:1004` — used by the key_drive X11 backend

## Documentation
- `README.md` — human-facing overview, quick start, first drive on a new car
- `DOCUMENTATION.md` — full module-level reference (packages, scripts, tools, system, local_python)
- `AGENTS.md` — guidance for AI coding agents (edit scope, safety rules, conventions)
- `CLAUDE.md` — short version of AGENTS.md
- `docs/SETUP_NEW_JETSON.md` — fresh-machine setup, step by step
- `docs/TROUBLESHOOTING.md` — known failure modes and fixes

Last updated: 2026-09-08
