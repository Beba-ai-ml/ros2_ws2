# CLAUDE.md

**Read [AGENTS.md](AGENTS.md) first.** It has the full repo map, conventions and porting
checklist. This file is the short version.

This is a ROS2 Foxy workspace for a physical F1TENTH racing car (Jetson Orin Nano, JetPack
5.1.5, Python 3.8, torch 1.13.1 CPU). `src/sac_driver/` runs the default
`session_Sesja_mpo2_2_policy.pth` policy on a 60 Hz tick with a decision every 8 ticks;
`src/f1tenth_stack/` brings up the lidar, VESC, joystick and command mux.

The default policy is the policy-only export from
`occupancy_racer/Soft_Actor_Critic_2/runs/session_Sesja_mpo2_2/`, trained on `mpo2`
(7,131 episodes, peak mean_100 195 m). Live lidar parameters and YAML on 2026-09-23 are
`angle_offset_deg=-90`, `angle_direction=-1`; cardboard data supports front raw 0° and
left raw +90°. Fresh user-confirmed front/left/right captures established the frame;
TF yaw is now 0 and Python fallbacks match YAML -90. All 16 offline parity tests pass.
Read `.context/RESEARCH-jetson-20260923.md` and the latest `.context/STATE.md`. After the
earlier Bringup-only stage, the user confirmed raised wheels again and authorized stand
tests. The current trial uses a temporary 0.5 m/s AI limit and the user's RB deadman.
Stand trials confirmed forward wheel motion, RB stop and right steering away from a
left-front box. Initial response to a right-front box was inconsistent; a passive probe
reproduced steering variation from changing scans/max-range
dropouts. The converter now preserves valid endpoints and repairs short bounded gaps
within one scan (`lidar.max_invalid_gap_deg=1.5`); a stand retest is in progress.
Resume timing was fixed and `model.cpu_threads=1` reduced latency. Ground driving is not
authorized; read current state before starting nodes.

## Key commands

```bash
source /opt/ros/foxy/setup.bash && cd ~/ros2_ws && source install/setup.bash

colcon build --packages-select sac_driver          # build only what you touched
python3 -m py_compile src/sac_driver/sac_driver/*.py   # fast syntax check
./install.sh --check                                # environment check, changes nothing

ros2 launch f1tenth_stack bringup_launch3.py       # hardware bringup
ros2 launch sac_driver sac_driver.launch.py        # AI driver (model_path:=... to override)
ros2 service call /sac_driver/enable std_srvs/srv/SetBool "{data: true}"
~/ros2_ws/scripts/key_drive.sh                     # keyboard teleop (starts bringup itself)
python3 scripts/key_drive_test.py                  # direction test, wheels OFF the ground

# clean up orphans after stopping a launch
pkill -f sllidar; pkill -f vesc; pkill -f joy; pkill -f ackermann_mux; pkill -f sac_driver_node
```

## Safety rules

1. **Never publish drive commands** (`/drive`, `/teleop_gated`, `ackermann_cmd`,
   `/commands/motor/*`) unless the user confirmed the wheels are off the ground or the track is
   clear. Ask every time.
2. **One driver per `/dev/vesc`.** Check for a running bringup and for
   `systemctl is-active key_drive.service` before starting anything.
3. **`key_drive.service` autostarts driving at boot** with no deadman switch — a keyboard
   plugged into the Jetson moves the car. Stop it before hardware work.
4. **Do not change calibration without a physical re-test:** `speed_to_erpm_gain`, `speed_min`
   / `speed_max`, servo gain/offset/limits in `src/f1tenth_stack/config/vesc.yaml`, and
   `control.speed_sign` / `control.steer_sign` / `lidar.angle_offset_deg` in
   `src/sac_driver/config/driver_params.yaml`. On this car positive `drive.speed` = REVERSE.
5. **Do not weaken safety logic**: the `/autonomy_lock` deadman, the mux priorities, the
   watchdog, `control.speed_limit_mps` (2.0) or `control.safe_mode`. And no passwords in the
   repo — privileged commands go through `sudo -n` and `/etc/sudoers.d/f1tenth`.

## After every change

Update **`.context/STATE.md`** — what changed, why, what still does not work. It is the
project's living memory and the first thing the next agent reads.
