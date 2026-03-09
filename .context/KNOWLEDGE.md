# SAC Driver - Knowledge Base

## Architecture

### Full System Diagram
```
                     JOYSTICK
                        │
                        ▼
┌─────────────────────────────────────────────────┐
│              joy_mode_manager                    │
│  LB (btn 4) → manual mode (gates teleop)        │
│  RB (btn 5) → autonomy mode (unlocks /drive)    │
│  Logic: lock = NOT autonomy_active OR manual     │
│  Publishes at 50Hz to /autonomy_lock             │
└───────────┬───────────────────┬─────────────────┘
            │                   │
    /teleop_gated          /autonomy_lock
    (priority 100)         (Bool: True=locked)
            │                   │
            ▼                   ▼
┌─────────────────────────────────────────────────┐
│              ackermann_mux                       │
│  joystick channel: /teleop_gated (priority 100)  │
│  navigation channel: /drive (priority 10)        │
│  lock: /autonomy_lock (blocks navigation)        │
│  Output: /ackermann_cmd → VESC                   │
└─────────────────────────────────────────────────┘
            ▲                   ▲
    /teleop_gated              /drive
            │                   │
     joy_teleop           sac_driver_node
     (manual)             (autonomous)
```

### SAC Driver Internal Architecture
```
ROS2 Topics (async)          Timer (30Hz)              VESC
┌──────────┐                ┌─────────────────┐
│ /scan    │──→ latest_scan │                 │
│ /odom    │──→ latest_speed│  _on_timer():   │
│          │──→ latest_accel│  1. LidarConv   │     ┌──────────┐
│          │──→ latest_yaw  │  2. StateBuilder│────→│ /drive   │
│ /servo   │──→ latest_servo│  3. NN inference│     │(Ackermann│
│ /autonomy│──→ enabled     │  4. ControlMap  │     └──────────┘
│  _lock   │                └─────────────────┘
└──────────┘
```

- **Async callbacks** store latest sensor values (no queuing, always overwrite)
- **Timer at 30Hz** grabs latest values, builds observation, runs NN, publishes command
- **Frame stacking**: sliding window of 4 frames (deque), each frame = 32 floats
- **Total state**: 32 x 4 = 128 floats fed to GaussianPolicy neural network
- **Lidar is ~8Hz, odom ~50Hz** — between lidar updates, timer reuses last scan but gets fresh speed/accel/yaw

## Observation Vector (per frame = 32 floats)

```
[0-26]  27x lidar rays (dist / 20.0, clipped [0,1])
[27]    collision flag (currently hardcoded 0.0)
[28]    speed (abs(speed_mps) / max_speed_mps, [0,1])
[29]    servo (normalized, [0,1])
[30]    linear_acceleration (accel_mps2 / 4.0, clamped [-1,1])
[31]    angular_velocity (yaw_rate_rad_s / 3.0, clamped [-1,1])
```

## Lidar Angles (27 rays)
```
[-15, 0, 15, 30, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90,
 95, 100, 105, 110, 115, 120, 125, 130, 135, 150, 165, 180, 195]
```
With -90° offset applied in scan frame. Max range 20m. Interpolation enabled.

## Key Files

### `src/sac_driver/sac_driver/` (ROS2 package source)
- **`sac_driver_node.py`** — Main ROS2 node. Subscribes to /scan, /odom, /servo, /autonomy_lock. Runs 30Hz timer loop. Computes linear_accel from odom speed delta. Builds state, runs inference, publishes /drive. Has debug logging for data readiness and autonomy state transitions.
- **`state_builder.py`** — Builds 32-element observation frames, maintains 4-frame sliding deque. Normalizes all inputs (lidar, speed, servo, accel, yaw). Returns concatenated 128-float state vector.
- **`lidar_converter.py`** — Extracts 27 specific angles from LaserScan with -90deg offset and optional interpolation. Returns normalized [0,1] distances.
- **`inference_engine.py`** — Wraps GaussianPolicy. Takes 128-float state, returns (steer, accel) as two floats. Generic, no hardcoded dimensions.
- **`policy_loader.py`** — Loads .pth checkpoint, auto-detects state_dim/action_dim/hidden_sizes from weight shapes. Builds GaussianPolicy and loads weights. Handles `torch.compile` prefix stripping.
- **`control_mapper.py`** — Maps NN output [-1,1] to Ackermann commands. Rate limiting, speed limiting, safe mode scaling, wheelbase-aware yaw rate. `speed_sign` and `steer_sign` params to flip directions.

### `src/sac_driver/config/`
- **`driver_params.yaml`** — All ROS2 parameters: model path, lidar angles, state config, control limits, topic names, safety settings.

### `src/sac_driver/launch/`
- **`sac_driver.launch.py`** — Launch file with configurable args for model_path, topic names, namespace.

### `src/f1tenth_stack/`
- **`f1tenth_stack/joy_mode_manager.py`** — Gates teleop with LB deadman, controls autonomy lock with RB. Publishes `/autonomy_lock` at 50Hz. Logic: `lock = NOT autonomy_active OR manual_active`.
- **`launch/bringup_launch3.py`** — Main bringup: lidar, vesc, joy, mux, transforms.

### `src/ackermann_mux/`
- C++ mux with priority channels. Joystick (priority 100) overrides navigation (priority 10). Lock topic blocks navigation channel.

## Tech Stack
- **Platform:** NVIDIA Jetson Nano (aarch64), Linux 5.10-tegra
- **ROS2:** Foxy (EOL but functional)
- **Python:** 3.8 with PyTorch 1.13.1 (CPU inference), numpy 1.24.4
- **Hardware:** SLAMTEC RPLiDAR, VESC motor controller, RC servo, Xbox-style gamepad
- **NFS shared folder:** `/home/laptop/shared` ↔ PC at `/home/beba/shared` (192.168.1.102)
- **ROS2 Panel:** GTK3 Python app at `/home/laptop/ros2_panel/` for managing bringup

## How to Run

### Prerequisites
- **Bringup must be running first** — provides lidar, odom, vesc, joystick nodes
- **NFS must be mounted** for model weights: `sudo mount /home/laptop/shared`
- Required ROS2 topics active before launching sac_driver:
  - `/scan` (sensor_msgs/LaserScan) — lidar, ~8Hz, RELIABLE QoS
  - `/odom` (nav_msgs/Odometry) — speed + yaw rate, ~50Hz
  - `/commands/servo/position` (std_msgs/Float64) — servo commands (only publishes when car is driven)
  - `/autonomy_lock` (std_msgs/Bool) — joystick deadman switch, 50Hz from joy_mode_manager
- Model weights file must exist at path specified in config

### Build
```bash
cd /home/laptop/ros2_ws && source /opt/ros/foxy/setup.bash && colcon build --packages-select sac_driver
```

### Run (MUST include --params-file!)
```bash
source /opt/ros/foxy/setup.bash && source /home/laptop/ros2_ws/install/setup.bash && ros2 run sac_driver sac_driver_node --ros-args --params-file /home/laptop/ros2_ws/src/sac_driver/config/driver_params.yaml
```
**CRITICAL:** Without `--params-file` the node has NO model path, NO config → will only print "model.path is empty" and do nothing.

### Run via launch file
```bash
source /opt/ros/foxy/setup.bash && source /home/laptop/ros2_ws/install/setup.bash && ros2 launch sac_driver sac_driver.launch.py
```

### Enabling autonomous driving
- Node starts **disabled** (`control.enable_on_start: false`)
- **Hold RB** (button 5) on joystick AND **release LB** → `/autonomy_lock` = False → sac_driver enables → car drives
- **Release RB** → `/autonomy_lock` = True → immediate stop
- **LB overrides** — if LB is also pressed, autonomy stays locked (manual takes priority)
- Alternative (no joystick): `ros2 service call /sac_driver/enable std_srvs/srv/SetBool "{data: true}"`

### ROS2 interface summary
| Direction | Topic | Type | QoS | Purpose |
|-----------|-------|------|-----|---------|
| Subscribe | `/scan` | LaserScan | RELIABLE (10) | Lidar ranges |
| Subscribe | `/odom` | Odometry | RELIABLE (10) | Speed, yaw rate, accel (derived) |
| Subscribe | `/commands/servo/position` | Float64 | RELIABLE (10) | Servo position commands |
| Subscribe | `/autonomy_lock` | Bool | RELIABLE (10) | Deadman switch (True=stop) |
| Publish | `/drive` | AckermannDriveStamped | RELIABLE (10) | Steering + speed to mux |
| Service | `/sac_driver/enable` | SetBool | — | Manual enable/disable |

### Bringup management
- **ROS2 Panel app:** `cd /home/laptop/ros2_panel && python3 panel_app.py`
- Panel manages: SETUP, Bringup, SLAM, Localize, Pursuit, Stanley
- **Kill zombie nodes after bringup stop:** `pkill -f sllidar && pkill -f vesc && pkill -f joy && pkill -f ackermann_mux`

## Conventions
- **Only edit `sac_driver/`** — `src/` training code lives on PC, read-only here
- **All normalization params come from config** — no magic numbers in code
- **policy_loader auto-detects architecture** — no need to hardcode state_dim/hidden sizes
- **Variable naming:** `sensor_accel`/`sensor_yaw` for sensor readings vs `accel` for NN output (avoid shadowing)
- **Collision flag:** hardcoded 0.0 (no collision sensor on physical car, kept for model compatibility)
- **All subscriptions use RELIABLE QoS (depth 10)** — NOT `qos_profile_sensor_data` (BEST_EFFORT causes missed messages on this setup)

## Debugging Lessons Learned

### QoS Mismatches (CRITICAL)
- **NEVER use `qos_profile_sensor_data` (BEST_EFFORT)** for subscriptions on this Jetson — even though the lidar publishes RELIABLE, BEST_EFFORT subscribers silently miss messages. Use QoS `10` (RELIABLE) for all topics.
- Symptom: `scan=False` in data_ready logs despite `/scan` topic existing with publishers.

### Model Loading
- Models saved on PC with newer numpy (`numpy._core`) fail on Jetson (older `numpy.core`). Fix: re-save on PC as pure state_dict with `torch.save(state_dict, path)`.
- `model.weights_only: false` needed for full checkpoint loading.
- `policy_loader.py` auto-detects from `backbone.0.weight.shape[1]` → state_dim, `mean_layer.weight.shape[0]` → action_dim.

### Bringup / Node Lifecycle
- `ros2 launch` spawns nodes in separate process groups — killing the launch process does NOT kill the nodes.
- Zombie nodes hold USB ports (lidar) and cause subsequent launches to fail silently.
- Always `pkill -f sllidar && pkill -f vesc && pkill -f joy && pkill -f ackermann_mux` before re-launching.
- NFS mount (`/home/laptop/shared`) may disconnect after reboot — always check with `mount | grep nfs` and remount with `sudo mount /home/laptop/shared`.

### _publish_stop Guard
- `_publish_stop()` has a `_last_stop_sent` guard that suppresses repeated stop logs. This makes state transitions invisible — if the node goes disabled→enabled→data_missing, you only see the first "disabled" log.
- Debug logging was added: `_on_estop` logs "Autonomy ENABLED/DISABLED", `_on_timer` logs "Waiting for data: scan=X odom=Y servo=Z".

### Speed/Steer Sign
- `control.speed_sign` and `control.steer_sign` in config flip the NN output direction.
- Currently both set to `-1.0` (inverted from training sim to real car).

## GitHub Repository
- **URL:** https://github.com/Beba-ai-ml/ros2_ws2
- **README.md** — Project overview, architecture, quick start, config reference
- **DOCUMENTATION.md** — Full module-level reference for all packages

Last updated: 2026-03-09
