# SAC Racing — AI-Powered Autonomous Racing on F1TENTH

A neural network-driven autonomous racing vehicle built on ROS2 and NVIDIA Jetson Nano. The core of this project is **SAC Driver** — a real-time inference node that runs a trained **Soft Actor-Critic (SAC)** reinforcement learning policy to drive a 1/10th scale RC car at speed using only lidar and odometry.

The SAC agent was trained in simulation using [occupancy_racer](https://github.com/Beba-ai-ml/occupancy-racer-sac2) and deployed to physical hardware via a modular inference pipeline. The car processes 27 lidar rays + speed + servo + acceleration + yaw rate at 30Hz, stacks 4 frames into a 128-dimensional state vector, and outputs continuous steering and throttle commands through a GaussianPolicy neural network.

## Quick Start

```bash
git clone https://github.com/Beba-ai-ml/ros2_ws2.git
cd ros2_ws2
source /opt/ros/foxy/setup.bash
colcon build --packages-select sac_driver f1tenth_stack ackermann_mux
source install/setup.bash
ros2 launch sac_driver sac_driver.launch.py
```

> **Prerequisites:** ROS2 Foxy, PyTorch 1.13+, NVIDIA Jetson Nano (or any Linux aarch64/x86_64).
> Bringup (lidar, VESC, joystick) must be running first — see [Running](#running) below.

## Architecture

```
                JOYSTICK (Xbox-style gamepad)
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

## SAC Driver — AI Inference Pipeline

The heart of this project. A ROS2 node that runs a trained SAC neural network in real-time on the Jetson Nano.

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

### How It Works

1. **Lidar Converter** — Extracts 27 specific angles from the RPLiDAR scan with a -90deg frame offset. Normalizes distances to [0, 1] range (max 20m). Supports interpolation between scan indices.

2. **State Builder** — Builds a 32-element observation vector per frame:
   - `[0-26]` — 27 lidar rays (distance / 20.0, clipped [0,1])
   - `[27]` — collision flag (hardcoded 0.0 on real car)
   - `[28]` — speed (normalized by max_speed)
   - `[29]` — servo position (normalized)
   - `[30]` — linear acceleration (derived from odom speed delta)
   - `[31]` — angular velocity (from odom twist)

   Maintains a sliding window of 4 frames → **128-float state vector** fed to the neural network.

3. **Inference Engine** — Runs the `GaussianPolicy` network (3x256 hidden layers) on CPU. Input: 128 floats → Output: (steering, acceleration) in [-1, 1].

4. **Control Mapper** — Maps NN output to physical Ackermann commands with rate limiting, speed limiting, safe mode scaling, and configurable sign inversion for sim→real transfer.

### Safety Features

- **Deadman switch** — Hold RB on gamepad to enable AI. Release = immediate stop.
- **Manual override** — LB (manual teleop) always overrides AI via ackermann_mux priority system.
- **Safe mode** — Configurable speed/steering/acceleration scaling factors.
- **Rate limiting** — Smooth steering and acceleration transitions.
- **Watchdog timeout** — Stops if no sensor data arrives within 0.5s.

### Model Details

| Property | Value |
|----------|-------|
| Algorithm | Soft Actor-Critic (SAC) |
| Network | GaussianPolicy, 3x256 hidden layers |
| State dim | 128 (32 features x 4 stacked frames) |
| Action dim | 2 (steering, acceleration) |
| Framework | PyTorch 1.13.1 (CPU inference) |
| Control rate | 30 Hz |
| Training | ~18,000 episodes in occupancy grid simulator |

## Project Structure

```
ros2_ws2/
├── src/
│   ├── sac_driver/              ← AI inference node (main project)
│   │   ├── sac_driver/
│   │   │   ├── sac_driver_node.py    # Main ROS2 node (subscribers, timer, pipeline)
│   │   │   ├── state_builder.py      # 32-elem frame builder + 4-frame stacking
│   │   │   ├── lidar_converter.py    # 27-angle lidar extraction with interpolation
│   │   │   ├── inference_engine.py   # GaussianPolicy wrapper
│   │   │   ├── policy_loader.py      # .pth loader with auto architecture detection
│   │   │   └── control_mapper.py     # NN output → Ackermann commands
│   │   ├── config/
│   │   │   └── driver_params.yaml    # All runtime parameters
│   │   └── launch/
│   │       └── sac_driver.launch.py  # Launch file with configurable args
│   │
│   ├── f1tenth_stack/           ← Vehicle bringup & joystick control
│   │   ├── f1tenth_stack/
│   │   │   └── joy_mode_manager.py   # Deadman switch + autonomy lock
│   │   └── launch/
│   │       └── bringup_launch3.py    # Lidar, VESC, joy, mux, transforms
│   │
│   ├── ackermann_mux/           ← Priority-based command multiplexer (C++)
│   ├── slam_toolbox/            ← SLAM for mapping
│   ├── particle_filter/         ← Monte Carlo localization
│   ├── pure_pursuit/            ← Pure Pursuit path tracker (C++)
│   ├── stanley_avoidance/       ← Stanley controller with obstacle avoidance
│   ├── waypoint_generator/      ← Waypoint recording from odometry (C++)
│   ├── gap_follow/              ← Reactive gap following (F1TENTH lab)
│   ├── wall_follow/             ← PID wall following (F1TENTH lab)
│   ├── safety_node/             ← Emergency braking (F1TENTH lab)
│   ├── sllidar_ros2/            ← SLAMTEC RPLiDAR driver
│   ├── teleop_tools/            ← Teleop utilities
│   └── scan_matching/           ← Scan matching algorithms
│
└── ros2_panel/                  ← GTK3 control panel app (separate)
```

## Running

### 1. Hardware Setup

Run the SETUP process first (configures SPI, device permissions, LED strip):
```bash
python3 ros2_panel/panel_app.py  # Click SETUP button
```

### 2. Bringup (Lidar, VESC, Joystick, Mux)

```bash
source /opt/ros/foxy/setup.bash && source install/setup.bash
ros2 launch f1tenth_stack bringup_launch3.py
```

### 3. Launch SAC Driver

```bash
source /opt/ros/foxy/setup.bash && source install/setup.bash
ros2 run sac_driver sac_driver_node --ros-args --params-file src/sac_driver/config/driver_params.yaml
```

### 4. Enable Autonomous Driving

- **Hold RB** (button 5) on the gamepad and **release LB** → car drives autonomously
- **Release RB** → immediate stop
- **LB** (manual teleop) always overrides AI

Alternative (no gamepad):
```bash
ros2 service call /sac_driver/enable std_srvs/srv/SetBool "{data: true}"
```

## ROS2 Control Panel

A native GTK3 desktop application for managing all ROS2 nodes from a single interface. Features a dark theme with animated LED indicators, toggle switches, and a live debug console.

| Feature | Description |
|---------|-------------|
| **Boot animation** | Fade-in/fade-out splash screen |
| **Process cards** | SETUP, Bringup, SLAM, Localize, Pursuit, Stanley |
| **LED indicators** | Red (stopped), Yellow (starting), Green (running) with pulse animation |
| **Toggle switches** | Animated on/off switches to start/stop each node |
| **Debug console** | Color-coded live log output from all processes |
| **Zombie killer** | Automatically kills orphaned ROS2 nodes on bringup stop |

```bash
cd ros2_panel && python3 panel_app.py
```

## Configuration

Key parameters in `src/sac_driver/config/driver_params.yaml`:

```yaml
model.path: "/path/to/actor_weights.pth"
model.device: "cpu"
state.stack_frames: 4
state.max_speed_mps: 8.0
control.speed_sign: -1.0       # Flip for sim→real transfer
control.steer_sign: -1.0       # Flip for sim→real transfer
control.speed_limit_mps: 2.0   # Max speed cap
control.max_steering_angle_deg: 20.0
control.safe_mode: true
safety.watchdog_timeout_sec: 0.5
```

## Hardware

| Component | Model |
|-----------|-------|
| Compute | NVIDIA Jetson Nano 4GB |
| Lidar | SLAMTEC RPLiDAR A2 |
| Motor Controller | VESC 6 |
| Chassis | F1TENTH 1/10 scale RC car |
| Gamepad | Xbox-style USB controller |

## Tech Stack

- **ROS2 Foxy** (Ubuntu 20.04)
- **Python 3.8** + **PyTorch 1.13.1** (CPU inference on Jetson)
- **C++** for ackermann_mux, pure_pursuit, waypoint_generator
- **GTK3 + Cairo** for the control panel GUI
- **NumPy** for lidar processing and state normalization

## Additional Controllers

This repository also includes classical path-tracking controllers originally developed by [Steven Gong](https://github.com/CL2-UWaterloo):

- **Pure Pursuit** (`src/pure_pursuit/`) — Geometric path tracking controller that follows pre-recorded waypoints. Implemented in C++ for low-latency execution. Includes a waypoint visualizer for RViz.

- **Stanley Controller** (`src/stanley_avoidance/`) — Front-axle based path tracking with integrated obstacle avoidance using lidar gap detection. Implemented in Python. Supports multi-agent scenarios in simulation.

Both controllers use the same localization pipeline (particle filter) and waypoint infrastructure, and can be launched from the ROS2 Control Panel.

## Training

The SAC model was trained on a separate PC using the [occupancy-racer-sac2](https://github.com/Beba-ai-ml/occupancy-racer-sac2) simulator. Trained weights are transferred to the Jetson via NFS shared folder and loaded by `policy_loader.py`, which auto-detects the network architecture from weight shapes.

## License

This project is part of an academic/research effort in autonomous racing. The F1TENTH platform components follow their respective upstream licenses.
