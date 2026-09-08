# SAC Racing — AI-Powered Autonomous Racing on F1TENTH

A neural-network-driven autonomous racing vehicle built on ROS2 Foxy and an NVIDIA Jetson Orin Nano. The core of this project is **SAC Driver** — a real-time inference node that runs a trained **Soft Actor-Critic (SAC)** reinforcement learning policy to drive a 1/10th scale RC car using only lidar and odometry.

The SAC agent was trained in simulation with [occupancy-racer-sac2](https://github.com/Beba-ai-ml/occupancy-racer-sac2) and deployed to physical hardware via a modular inference pipeline. The car processes 450 lidar rays (variable resolution: 0.5° front, 2.0° rear) + speed + steering + acceleration feedback + linear acceleration + yaw rate at 30 Hz, stacks 4 frames into a **1820-dimensional state vector**, and outputs continuous steering and throttle commands through a GaussianPolicy network (~1.2M parameters, ~5-6 ms inference on the Jetson CPU).

This repository is self-contained: clone it on a fresh Jetson, run one installer, and the car drives.

---

## What's in the box

### Hardware

| Component | Model / Notes |
|-----------|---------------|
| Compute | NVIDIA Jetson Orin Nano Super Developer Kit, JetPack 5.1.5 (L4T R35.6.1), Ubuntu 20.04.6, 25 W power mode |
| Lidar | SLAMTEC RPLiDAR S1, 256000 baud, CP210x USB bridge (`10c4:ea60`) → `/dev/rplidar` |
| Motor controller | VESC 6 (HW60, firmware 6.02), STM32 VCP (`0483:5740`) → `/dev/vesc` |
| Gamepad | Logitech F710 (`046d:c219`) or any USB joystick → `/dev/input/js0` |
| Status LEDs | 7x WS2812B on SPI1 MOSI (40-pin header, pin 19) → `/dev/spidev0.0` |
| Shutdown button | Optional GPIO button, BOARD pins 37 (drive) / 38 (sense) — not installed on this car |
| Chassis | F1TENTH 1/10 scale RC car, wheelbase 0.35 m |

CUDA 11.4 is present on the Jetson, but **inference runs on the CPU** (torch 1.13.1 CPU wheel). The policy needs ~5-6 ms per step, well inside the 33 ms budget of the 30 Hz control loop.

### Software

- ROS2 Foxy (apt), Python 3.8.10
- PyTorch 1.13.1 (CPU), NumPy 1.24.4
- C++ nodes: `ackermann_mux`, `pure_pursuit`, `waypoint_generator`
- GTK3 control panel (`ros2_panel/`)

---

## Quick Start

On a Jetson with JetPack 5.x (Ubuntu 20.04) already flashed:

```bash
git clone https://github.com/Beba-ai-ml/ros2_ws2.git ~/ros2_ws && cd ~/ros2_ws && ./install.sh
```

Then reboot so the new group memberships (`dialout`, `gpio`, `input`, `plugdev`, `video`) take effect:

```bash
sudo reboot
```

If ROS2 Foxy is not installed yet, run `./install.sh --with-ros` instead. See
[docs/SETUP_NEW_JETSON.md](docs/SETUP_NEW_JETSON.md) for the full fresh-machine walkthrough.

| `install.sh` flag | Effect |
|---|---|
| *(none)* | apt + pip deps, rosdep, udev, spidev, groups, sudoers, `.bashrc`, build core packages |
| `--check` | verify only, change nothing (PASS/FAIL per item) |
| `--with-ros` | install ROS2 Foxy from apt first if `/opt/ros/foxy` is missing |
| `--full` | also build particle_filter, pure_pursuit, stanley_avoidance, waypoint_generator, labs + range_libc |
| `--no-build` | skip the colcon build |
| `--desktop` | put a ROS2 Control Panel launcher on `~/Desktop` |
| `--gpio-shutdown` | install + enable the GPIO shutdown button service |
| `--bt-pad` | install + enable the Bluetooth gamepad auto-connect service |
| `--key-drive-service` | install + enable keyboard-drive autostart (**car drives right after boot**) |
| `--yes` | non-interactive |

### Drive

```bash
# terminal 1 — hardware bringup (lidar, VESC, joystick, mux)
source /opt/ros/foxy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch f1tenth_stack bringup_launch3.py

# terminal 2 — SAC inference node
source /opt/ros/foxy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch sac_driver sac_driver.launch.py
```

Hold **RB** (button 5) on the gamepad with **LB** released → the car drives autonomously.
Release RB → immediate stop. LB (manual teleop) always overrides the AI.

---

## First drive on a NEW car

**Every car is different.** Motor wiring, servo linkage and lidar mounting all change the
signs and gains below. Work through this checklist before putting the car on the ground.

**1. Devices enumerate**

```bash
ls -l /dev/rplidar /dev/vesc      # both symlinks must exist (udev rules from install.sh)
ls /dev/input/js0                 # joystick
```

If a symlink is missing, plug the device in and check `lsusb` for the vendor:product IDs in
the hardware table; adjust `system/udev/99-f1tenth.rules` if your hardware differs.

**2. Put the car on a stand — wheels off the ground.** Everything below moves the motor.

**3. Upload the VESC configuration** (motor + app config for this chassis):

```bash
python3 tools/vesc/vesc_config_upload.py \
    --motor tools/vesc/configs/motor_config.xml \
    --app   tools/vesc/configs/app_config.xml --verify
```

No VESC Tool GUI is needed. The tool speaks the native VESC binary protocol; parameter
definitions for firmware 6.02 live in `tools/vesc/params/6.02/`. Use `--dry-run` first and
`--check-sig` to confirm the config structure matches your firmware.

**4. Automated direction test** (car still off the ground):

```bash
source /opt/ros/foxy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch f1tenth_stack bringup_launch3.py   # in another terminal
python3 scripts/key_drive_test.py
```

It publishes forward / reverse / left / right on `/teleop_gated` and checks the motor ERPM
from `/sensors/core` and the servo command on `/commands/servo/position`.

**5. Fix the speed sign.** On this car a **positive** `AckermannDrive.speed` spins the motor
**backwards**. That is why:

| Where | Parameter | Value on this car |
|-------|-----------|-------------------|
| `src/sac_driver/config/driver_params.yaml` | `control.speed_sign` | `-1.0` |
| `scripts/key_drive.py` | `SPEED_SIGN` | `-1.0` |

If your car drives forward on a positive speed, flip both to `+1.0`.

**6. Cardboard steering test.** With the car on the stand and the SAC driver enabled, hold a
large piece of cardboard close to **one** side of the lidar. The wheels must steer **away**
from the obstacle. If they steer **into** it, the lidar frame convention is wrong — change
`lidar.angle_offset_deg` (0.0 on this car, `-90.0` for the older 27-ray model) and/or
`control.steer_sign` (`1.0` on this car). Re-test after every change.

**7. Calibrate `src/f1tenth_stack/config/vesc.yaml`** for your motor and servo:

| Key | Value here | Meaning |
|-----|-----------|---------|
| `speed_to_erpm_gain` | `1850.0` | erpm per m/s — depends on motor KV, gearing, wheel size |
| `speed_min` / `speed_max` | `-45250.0` / `45250.0` | erpm clamp in the VESC driver |
| `steering_angle_to_servo_gain` | `-0.9` | servo units per radian; sign follows linkage |
| `steering_angle_to_servo_offset` | `0.5304` | servo value for wheels straight |
| `servo_min` / `servo_max` | `0.05` / `0.95` | mechanical servo limits |
| `vesc_to_odom_node.wheelbase` | `0.35` | metres, front-to-rear axle |

Measure the erpm gain by driving a known distance and comparing `/odom` to reality. Find the
servo offset by nudging `/commands/servo/position` until the wheels point straight.

**8. Only then put the car on the ground**, in a clear space, with `control.speed_limit_mps`
still at the default `2.0`.

---

## Architecture

```
                JOYSTICK (Logitech F710 / any gamepad)
                          |
                          v
+---------------------------------------------------+
|                 joy_mode_manager                   |
|  LB (btn 4) = manual mode (gates teleop)          |
|  RB (btn 5) = autonomy mode (unlocks /drive)      |
|  Publishes /autonomy_lock at 50Hz                  |
+--------------+--------------------+---------------+
               |                    |
       /teleop_gated         /autonomy_lock
       (priority 100)        (Bool: True=locked)
               |                    |
               v                    v
+---------------------------------------------------+
|                ackermann_mux (C++)                 |
|  Joystick: /teleop_gated (priority 100)           |
|  AI:       /drive         (priority 10)           |
|  Lock: /autonomy_lock blocks AI channel           |
|  Output: /ackermann_cmd --> VESC motor controller |
+---------------+-------------------+---------------+
                ^                   ^
        /teleop_gated             /drive
                |                   |
         joy_teleop          sac_driver_node
         (manual)            (AI autonomous)
```

`ros2 launch f1tenth_stack bringup_launch3.py` starts:

| Node | Package | Purpose |
|------|---------|---------|
| `sllidar_node` | `sllidar_ros2` | RPLiDAR S1 driver → `/scan` |
| `vesc_driver_node` | `vesc_driver` | VESC serial link → `/sensors/core` |
| `ackermann_to_vesc_node` | `vesc_ackermann` | `ackermann_cmd` → motor/servo commands |
| `vesc_to_odom_node` | `vesc_ackermann` | VESC telemetry → `/odom` + TF |
| `joy_linux_node` | `joy_linux` | `/dev/input/js0` → `/joy` |
| `joy_teleop` | `joy_teleop` (patched copy) | `/joy` → `/teleop` |
| `joy_mode_manager` | `f1tenth_stack` | deadman gating, `/teleop_gated`, `/autonomy_lock` |
| `ackermann_mux` | `ackermann_mux` | priority mux → `ackermann_cmd` |
| `static_transform_publisher` | `tf2_ros` | `base_link` → `laser` (0.27, 0, 0.11) |

Notes:

- `joy_teleop` is launched with `<ws>/local_python` prepended to `PYTHONPATH`. That directory
  holds a patched copy of the upstream node: commands configured **without** deadman buttons
  are treated as always active, which the shipped `joy_teleop.yaml` relies on. Without the
  patch the config is rejected at startup.
- The mux publishes on `ackermann_cmd`; the `ackermann_cmd_out` → `ackermann_drive` remap in
  the launch file is a no-op, and `ackermann_to_vesc_node` subscribes to `ackermann_cmd`, so
  the chain works as drawn.
- `throttle_interpolator` is present in `vesc.yaml` (`max_acceleration: 9.5`) but the node is
  currently commented out in the launch file.

---

## SAC Driver — AI inference pipeline

A ROS2 node that runs the trained SAC policy in real time on the Jetson CPU.

```
 ROS2 Topics (async)        Timer (30Hz)              VESC
+--------------+         +------------------+    +------------+
| /scan        |--> scan |                  |    |            |
| /odom        |--> speed|  _on_timer():    |    |  /drive    |
|              |--> accel|  1. LidarConv    |--->| (Ackermann |
|              |--> yaw  |  2. StateBuilder |    |  Drive)    |
| /servo       |--> servo|  3. NN inference |    |            |
| /autonomy    |--> lock |  4. ControlMap   |    +------------+
|  _lock       |         |                  |
+--------------+         +------------------+
```

### How it works

1. **Lidar Converter** — extracts 450 angles from the RPLiDAR scan using variable-resolution
   stepping (0.5° front hemisphere, 2.0° rear hemisphere), applies `lidar.angle_offset_deg`
   with wrapping to [-pi, pi), and normalizes distances to [0, 1] (max 20 m). Interpolates
   between adjacent scan indices.

2. **State Builder** — builds a 455-element observation vector per frame:
   - `[0-449]` — 450 lidar rays (distance / 20.0, clipped [0,1])
   - `[450]` — speed (normalized by `state.max_speed_mps`, [0,1])
   - `[451]` — steering position (servo centered to [-1,1])
   - `[452]` — acceleration feedback (previous NN action, raw [-1,1])
   - `[453]` — linear acceleration (derived from odom speed delta, [-1,1])
   - `[454]` — angular velocity (from odom twist, [-1,1])

   Maintains a sliding window of 4 frames → **1820-float state vector**.

3. **Inference Engine** — runs the `GaussianPolicy` network (hidden layers [512, 512, 256])
   on CPU under `torch.no_grad()`. Input 1820 floats → steering in [-1,1], acceleration in
   [0,2]. ~5-6 ms per step.

4. **Control Mapper** — maps the NN output to physical Ackermann commands with rate limiting,
   speed limiting, safe-mode scaling and configurable sign inversion for sim→real transfer.

### Safety features

- **Deadman switch** — hold RB on the gamepad to enable the AI. Release = immediate stop.
- **Manual override** — LB (manual teleop) always wins via the `ackermann_mux` priority system.
- **Safe mode** — configurable speed / steering / acceleration scaling factors.
- **Rate limiting** — smooth steering and acceleration transitions.
- **Watchdog** — stops if no sensor data arrives within `safety.watchdog_timeout_sec` (0.5 s).
- **Speed cap** — `control.speed_limit_mps` (2.0 m/s) clamps the commanded speed.

### Model details

| Property | Value |
|----------|-------|
| Algorithm | Soft Actor-Critic (SAC) |
| Network | GaussianPolicy, hidden [512, 512, 256] (~1.2M params) |
| State dim | 1820 (455 features x 4 stacked frames) |
| Action dim | 2 (steering [-1,1], acceleration [0,2]) |
| Lidar | 450 rays, variable resolution (0.5° front, 2.0° rear) |
| Framework | PyTorch 1.13.1, CPU inference, ~5-6 ms/step |
| Control rate | 30 Hz |
| Active weights | `src/sac_driver/weights/session_Rybnik_02_1.pth` (Rybnik_02 map) |
| Previous weights | `src/sac_driver/weights/session_car_1_3.pth` |

Checkpoints are tracked in git and installed into the package share directory by `setup.py`,
so `model.path` in `driver_params.yaml` is **relative** (`weights/session_Rybnik_02_1.pth`)
and resolves against `<install>/share/sac_driver/`. Absolute paths, `~/...` and
`package://sac_driver/weights/...` also work.

To run a different checkpoint:

```bash
ros2 launch sac_driver sac_driver.launch.py model_path:=/abs/path/to/other.pth
```

`policy_loader.py` auto-detects `state_dim`, `action_dim` and hidden sizes from the weight
shapes, strips a `_orig_mod.` prefix left by `torch.compile`, and aliases `numpy._core` so
checkpoints saved with newer NumPy on the training PC load under NumPy 1.24 on the Jetson.

### Enabling autonomy without a gamepad

```bash
ros2 service call /sac_driver/enable std_srvs/srv/SetBool "{data: true}"
```

---

## Keyboard drive

Drive the car from a keyboard — useful for testing without a gamepad and for the headless
boot service.

```bash
~/ros2_ws/scripts/key_drive.sh                  # bringup + keyboard teleop
~/ros2_ws/scripts/key_drive.sh --speed 0.5      # options are passed through to key_drive.py
KEEP_BRINGUP=1 ~/ros2_ws/scripts/key_drive.sh   # leave bringup running after quitting
```

`key_drive.sh` builds `f1tenth_stack`, starts `bringup_launch3.py` (reusing a running bringup
if there is one), waits for `/sensors/core`, then runs `scripts/key_drive.py`. Pressing `q`
quits and stops the bringup it started.

`key_drive.py` publishes `AckermannDriveStamped` on `/teleop_gated` — mux priority 100, so it
is **not** masked by `/autonomy_lock` and does not need the joystick deadman.

| Key | Action |
|-----|--------|
| Up / W | forward |
| Down / S | reverse |
| Left / A, Right / D | steer |
| Space / Esc | stop |
| `+` / `-` | speed ±0.25 m/s |
| `q` / Ctrl-C | quit |

| Option | Default | Meaning |
|--------|---------|---------|
| `--speed` | `2.0` | m/s for Up/Down |
| `--steer` | `0.3` | rad for Left/Right |
| `--max-speed` | `4.0` | ceiling for `+` |
| `--topic` | `/teleop_gated` | output topic |
| `--evdev` | off | read a keyboard plugged into the Jetson (`/dev/input/event*`), no X needed |
| `--no-grab` | off | evdev: do not grab the keyboard exclusively |
| `--no-quit-key` | off | ignore `q` (used by the boot service) |
| `--no-x` | off | force the terminal fallback |

Input backends are auto-selected: **evdev** (hot-plug safe, sends STOP when the keyboard is
unplugged), **X11** via `XQueryKeymap` (works over NoMachine — this car uses `DISPLAY=:1004`),
and a latched terminal fallback.

---

## Control panel

A native GTK3 desktop application for managing all ROS2 nodes from one window.

```bash
python3 ros2_panel/panel_app.py     # or: ros2_panel/ros2panel
```

| Feature | Description |
|---------|-------------|
| Boot animation | Fade-in / fade-out splash screen |
| Process cards | SETUP, Bringup, SLAM, AI Inference, Localize, Pursuit, Stanley |
| LED indicators | Red (stopped), yellow (starting), green (running), with pulse animation |
| Toggle switches | Animated on/off switches to start/stop each node |
| Battery bar | VESC `voltage_input` from `/sensors/core`, 12 V = 100 %, 9 V = 0 % |
| Debug console | Colour-coded live log output from all processes |
| Zombie killer | Kills orphaned ROS2 nodes when a launch is stopped |

The SLAM card starts `slam_toolbox` (`online_async`, params from
`src/slam_toolbox/config/mapper_params_online_async.yaml`) and, three seconds later, `rviz2`
with `config/slam_rviz.rviz`. All paths are derived from the repository location, so the panel
works from any clone. `install.sh --desktop` writes a launcher to `~/Desktop` from
`system/desktop/ROS2-Panel.desktop.in` (your desktop may ask you to "Allow Launching" the
first time).

`legacy/system_web/` holds an older Flask web panel — unmaintained, kept for reference.

---

## Configuration

### `src/sac_driver/config/driver_params.yaml`

```yaml
model.path: "weights/session_Rybnik_02_1.pth"   # relative to the package share dir
model.device: "cpu"
model.weights_only: false
lidar.front_step_deg: 0.5        # variable-resolution lidar (450 rays)
lidar.rear_step_deg: 2.0
lidar.angle_offset_deg: 0.0      # 0 deg = forward for the Rybnik/car_1_3 models
lidar.max_range_m: 20.0
state.stack_frames: 4
state.max_speed_mps: 6.0
state.servo_norm_offset: -0.535  # servo centered to [-1, 1]
state.servo_norm_divisor: 0.435
control.speed_sign: -1.0         # positive speed = reverse on this car
control.steer_sign: 1.0
control.speed_limit_mps: 2.0
control.max_steering_angle_deg: 20.0
control.wheelbase_m: 0.35
control.safe_mode: true
safety.watchdog_timeout_sec: 0.5
```

`driver_params_27ray.yaml` keeps the configuration of the older 27-ray / 120-dim model.
Full key-by-key reference: [DOCUMENTATION.md](DOCUMENTATION.md).

### Other config files

| File | Contents |
|------|----------|
| `src/f1tenth_stack/config/vesc.yaml` | erpm gain, servo gain/offset, limits, odom wheelbase |
| `src/f1tenth_stack/config/joy_teleop.yaml` | `human_control` deadman `[4]`, speed axis 1 scale `-5.0`, steer axis 3 scale `0.34` |
| `src/f1tenth_stack/config/mux.yaml` | mux channels: `teleop_gated` prio 100, `drive` prio 10, lock `autonomy_lock` |
| `config/ackermann_mux.yaml`, `config/joy_mode_manager.yaml` | standalone copies used outside the launch file |
| `config/slam_rviz.rviz` | RViz layout for SLAM (map + laser + TF, top-down) |

---

## Maps and SLAM

Occupancy grids live in `maps/` as `.pgm` + `.yaml` pairs:

| Map | Note |
|-----|------|
| `Rybnik_01`, `Rybnik_02` | Rybnik track; `Rybnik_02` is the map the active policy was trained on |
| `Rybnk_04`, `Rybnk_05` | newer Rybnik recordings |
| `HW_01`–`HW_03`, `K_01`, `K_02`, `P_01`, `Dom_01`, `mpo`, `mpo2` | older venues |

Record a new map with the panel's SLAM card (or manually), then save it:

```bash
ros2 run nav2_map_server map_saver_cli -f maps/my_track
```

`src/slam_toolbox/` contains the upstream source for reference; the **apt** `slam-toolbox`
package is what actually runs — the source tree is not built.

---

## Project structure

```
ros2_ws/
├── install.sh                   # one-shot installer (see Quick Start)
├── requirements.txt             # Python deps (torch 1.13.1 CPU, numpy 1.24.4, ...)
├── README.md / DOCUMENTATION.md / AGENTS.md / CLAUDE.md
│
├── src/                         # ROS2 packages
│   ├── sac_driver/              #   AI inference node (main project)
│   │   ├── sac_driver/
│   │   │   ├── sac_driver_node.py    # ROS2 node: subscribers, 30Hz timer, pipeline
│   │   │   ├── state_builder.py      # 455-elem frame builder + 4-frame stacking
│   │   │   ├── lidar_converter.py    # 450-angle variable-resolution lidar extraction
│   │   │   ├── inference_engine.py   # GaussianPolicy wrapper
│   │   │   ├── policy_loader.py      # .pth loader with architecture auto-detection
│   │   │   └── control_mapper.py     # NN output -> Ackermann commands
│   │   ├── config/                   # driver_params.yaml, driver_params_27ray.yaml
│   │   ├── launch/sac_driver.launch.py
│   │   └── weights/                  # tracked .pth checkpoints
│   ├── f1tenth_stack/           #   bringup, joy_mode_manager, vesc/joy/mux configs
│   ├── ackermann_mux/           #   priority command multiplexer (C++)
│   ├── sllidar_ros2/            #   SLAMTEC RPLiDAR driver (vendored upstream)
│   ├── particle_filter/         #   Monte Carlo localization (vendored)
│   ├── range_libc/              #   fast ray casting for the particle filter (vendored)
│   ├── pure_pursuit/            #   Pure Pursuit path tracker (C++)
│   ├── stanley_avoidance/       #   Stanley controller with obstacle avoidance
│   ├── waypoint_generator/      #   waypoint recording from odometry (C++)
│   ├── gap_follow/ wall_follow/ safety_node/   # F1TENTH lab templates
│   ├── scan_matching/           #   scan matching experiments
│   ├── slam_toolbox/            #   upstream source, NOT built (apt version is used)
│   ├── f1tenth_system/          #   upstream README only
│   ├── teleop_tools/            #   empty placeholder
│   └── scripts/                 #   docker helper scripts for the upstream f1tenth image
│
├── local_python/                # patched joy_teleop (PYTHONPATH shim) + gpio_shutdown.py
├── ros2_panel/                  # GTK3 control panel
├── scripts/                     # key_drive*, ledy.py, ledy2.py
├── tools/                       # lidar_diag.py, lidar_test.py, vesc/ (config uploader)
├── system/                      # udev rules, systemd/sudoers/desktop templates (*.in)
├── config/                      # ackermann_mux.yaml, joy_mode_manager.yaml, slam_rviz.rviz, frames.gv
├── maps/                        # occupancy grids (.pgm + .yaml)
├── docs/                        # SETUP_NEW_JETSON.md, TROUBLESHOOTING.md, 3d-mapper/ (chassis CAD)
├── legacy/system_web/           # old Flask web panel (unmaintained)
└── .context/                    # machine-readable project state for AI agents
```

---

## Tools

| Tool | Purpose |
|------|---------|
| `tools/vesc/vesc_config_upload.py` | Upload motor/app XML configs to the VESC over serial — no VESC Tool GUI. `--dry-run`, `--verify`, `--check-sig`. Firmware 6.02 parameter definitions in `tools/vesc/params/6.02/`. |
| `tools/vesc/configs/*.xml` | Motor and app configuration for this car |
| `tools/lidar_diag.py` | Lidar diagnostics |
| `tools/lidar_test.py` | Raw lidar read test |
| `scripts/key_drive_test.py` | Automated forward/reverse/left/right check (wheels off the ground) |
| `scripts/ledy.py`, `scripts/ledy2.py` | Drive the 7x WS2812B strip over SPI (violet by default) |
| `scripts/bt_pad_connect.sh` | Bluetooth gamepad auto-connect loop (see Optional services) |
| `ros2_panel/scan_test.py` | Print the 20 closest lidar points — used to verify the lidar frame offset |

The LED scripts need SPI1 enabled on the 40-pin header:

```bash
sudo /opt/nvidia/jetson-io/jetson-io.py    # Configure 40-pin header -> enable spi1 -> save & reboot
```

`install.sh` adds `spidev` to `/etc/modules-load.d/` and puts the user in the `gpio` group;
the jetson-io step is manual because it rewrites the device tree.

---

## Optional services

Both are installed only when you ask for them.

### Keyboard drive on boot

```bash
./install.sh --key-drive-service
```

Installs and enables `system/systemd/key_drive.service.in` as `key_drive.service`
(`User=<you>`, `Restart=always`). On boot it runs `scripts/key_drive_boot.sh`: kills leftover
nodes, waits for `/dev/vesc`, starts the bringup, waits for `/sensors/core`, then runs
`key_drive.py --evdev --no-quit-key`. It supervises the VESC chain
(`vesc_driver_node`, `ackermann_to_vesc_node`, `ackermann_mux`) and exits — so systemd
restarts everything — if any of them dies.

> **SAFETY:** with this service enabled the car reacts to a keyboard plugged into the Jetson
> **immediately after boot**, with no deadman button. Keep the wheels off the ground while
> testing. It is **enabled on this car**. Turn it off with:
>
> ```bash
> sudo systemctl disable --now key_drive.service
> ```

`scripts/key_drive.sh` detects a running `key_drive.service`, stops it for the duration of the
manual session (only one process may own `/dev/vesc`) and restarts it on exit.

### Bluetooth gamepad auto-connect

`scripts/bt_pad_connect.sh` keeps retrying a Bluetooth connection to a DualShock 4 style pad
(MAC in `PAD_MAC`, override with the environment variable). Once connected, joydev creates
`/dev/input/js0` and the bringup's `joy_linux_node` picks it up on its own. Pad controls: hold
L1 (button 4) as deadman, left stick Y = throttle, right stick X = steering.

On this car it runs as `bt_pad.service` (`Restart=always`, log in `log/bt_pad.log`). Install it
on a new car with:

```bash
./install.sh --bt-pad      # edit PAD_MAC in scripts/bt_pad_connect.sh first
```

### GPIO shutdown button

```bash
./install.sh --gpio-shutdown
```

Installs and enables `gpio-shutdown.service`, which runs `local_python/gpio_shutdown.py`: BOARD pin 37 is
driven high, pin 38 is watched, and the Jetson powers off when 38 stays high for 0.2 s (button
or jumper between the two pins). Not installed on this car.

---

## Training link and NFS share

The policy is trained on a separate PC with
[occupancy-racer-sac2](https://github.com/Beba-ai-ml/occupancy-racer-sac2). Finished
checkpoints are copied into `src/sac_driver/weights/` and committed, so **inference has no
runtime dependency on the PC**.

For convenience this car mounts the training PC over NFS (optional, `fstab` entry):

| Jetson | Training PC |
|--------|-------------|
| `/home/laptop/shared` | `/home/beba/shared` (192.168.1.102) |

```bash
mount | grep shared          # check
sudo mount /home/laptop/shared   # remount after a reboot
```

Remote desktop on this car is NoMachine (`nxserver`), which is why the keyboard-drive X11
backend defaults to `DISPLAY=:1004`.

---

## Troubleshooting

See [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) — lidar reconnect failures, zombie
nodes, `/dev/vesc` permissions, QoS drops, checkpoint compatibility, keyboard-drive display
problems.

## For AI agents

If you are an AI coding agent working in this repository, read [AGENTS.md](AGENTS.md) first —
it covers the repo map, what may and may not be edited, build/verify commands, and the
hardware safety rules. [CLAUDE.md](CLAUDE.md) is the short version.

## Additional controllers

This repository also carries classical path-tracking controllers originally developed by
[Steven Gong](https://github.com/CL2-UWaterloo):

- **Pure Pursuit** (`src/pure_pursuit/`) — geometric path tracking over pre-recorded
  waypoints, C++ for low latency, with an RViz waypoint visualizer.
- **Stanley Controller** (`src/stanley_avoidance/`) — front-axle path tracking with reactive
  obstacle avoidance via lidar gap detection, Python, multi-agent sim scenarios included.

Both use the particle-filter localization pipeline and the same waypoint infrastructure, and
can be started from the control panel.

## License

This project is part of an academic/research effort in autonomous racing. The F1TENTH platform
components follow their respective upstream licenses.
