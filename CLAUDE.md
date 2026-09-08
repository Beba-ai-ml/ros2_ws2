# CLAUDE.md

**Read [AGENTS.md](AGENTS.md) first.** It has the full repo map, conventions and porting
checklist. This file is the short version.

This is a ROS2 Foxy workspace for a physical F1TENTH racing car (Jetson Orin Nano, JetPack
5.1.5, Python 3.8, torch 1.13.1 CPU). `src/sac_driver/` runs a Soft Actor-Critic policy at
30 Hz; `src/f1tenth_stack/` brings up the lidar, VESC, joystick and command mux.

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
