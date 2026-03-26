# SAC Racing — Module Reference Documentation

## Table of Contents

- [sac_driver](#sac_driver) — AI inference pipeline
- [f1tenth_stack](#f1tenth_stack) — Vehicle bringup and joystick control
- [ackermann_mux](#ackermann_mux) — Command multiplexer
- [ros2_panel](#ros2_panel) — GTK3 control panel
- [particle_filter](#particle_filter) — Monte Carlo localization
- [pure_pursuit](#pure_pursuit) — Pure Pursuit controller
- [stanley_avoidance](#stanley_avoidance) — Stanley controller
- [slam_toolbox](#slam_toolbox) — SLAM
- [Utilities](#utilities) — Supporting packages

---

## sac_driver

**Purpose:** Real-time SAC neural network inference for autonomous driving.

### `sac_driver_node.py` — Main ROS2 Node

The central orchestrator. Subscribes to sensor topics, runs the inference pipeline at 30Hz, and publishes drive commands.

| Element | Description |
|---------|-------------|
| `class SACDriverNode(Node)` | Main ROS2 node |
| `_on_scan(msg)` | Callback for `/scan` (LaserScan). Stores latest lidar ranges. |
| `_on_odom(msg)` | Callback for `/odom` (Odometry). Extracts speed, yaw rate. Computes linear acceleration from speed delta/dt. |
| `_on_servo(msg)` | Callback for `/commands/servo/position` (Float64). Stores servo position. |
| `_on_estop(msg)` | Callback for `/autonomy_lock` (Bool). Enables/disables driving. Logs state transitions. |
| `_on_timer()` | 30Hz timer. Builds state → runs inference → publishes `/drive`. Guards: checks data readiness and autonomy lock. |
| `_data_ready()` | Returns True when scan + odom data are available. |
| `_publish_stop()` | Publishes zero-speed Ackermann command. Rate-limited to avoid log spam. |

**Subscriptions:**

| Topic | Type | QoS | Rate |
|-------|------|-----|------|
| `/scan` | `sensor_msgs/LaserScan` | RELIABLE (10) | ~8 Hz |
| `/odom` | `nav_msgs/Odometry` | RELIABLE (10) | ~50 Hz |
| `/commands/servo/position` | `std_msgs/Float64` | RELIABLE (10) | On command |
| `/autonomy_lock` | `std_msgs/Bool` | RELIABLE (10) | 50 Hz |

**Publications:**

| Topic | Type | QoS |
|-------|------|-----|
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | RELIABLE (10) |

**Service:**

| Service | Type | Purpose |
|---------|------|---------|
| `/sac_driver/enable` | `std_srvs/SetBool` | Manual enable/disable |

---

### `lidar_converter.py` — Lidar Angle Extraction

Extracts target angles from the full lidar scan with angle wrapping to [-pi, pi).

| Element | Description |
|---------|-------------|
| `class LidarConverter` | Stateless converter, initialized with angle list and offset |
| `__init__(angles_deg, offset_deg, max_range)` | Sets up target angles, frame offset (-90deg), max range (20m) |
| `convert(scan_msg)` | Returns array of normalized distances [0,1]. Uses interpolation between adjacent scan indices. |
| `build_lidar_angles(front_step, rear_step)` | Generates variable-resolution angles: front hemisphere (0-180°) at front_step, rear (180-360°) at rear_step |

**Current config: 450 rays** via `build_lidar_angles(0.5, 2.0)`:
- Front hemisphere (0-180°): 361 rays at 0.5° step
- Rear hemisphere (182-358°): 89 rays at 2.0° step

---

### `state_builder.py` — Observation Vector Builder

Builds the 1820-float state vector from sensor data.

| Element | Description |
|---------|-------------|
| `class StateBuilder` | Maintains a deque of 4 frames |
| `__init__(stack_frames, lidar_dim, max_speed, ...)` | Configures normalization parameters |
| `update(lidar, speed, steer, accel_fb, accel, yaw)` | Creates one 455-float frame, appends to stack, returns concatenated state |
| `reset(first_obs)` | Clears deque and fills with first_obs |
| `single_obs_dim` | Property: lidar_dim + 5 = 455 |
| `state_dim` | Property: 455 x 4 = 1820 |

**Frame structure (455 floats):**

| Index | Feature | Normalization |
|-------|---------|---------------|
| 0-449 | 450 lidar distances | distance / 20.0, clipped [0,1] |
| 450 | Speed | abs(speed) / max_speed, [0,1] |
| 451 | Steering | servo centered, [-1,1] |
| 452 | Accel feedback | previous NN accel (raw), [-1,1] |
| 453 | Linear acceleration | accel / max_accel, clamped [-1,1] |
| 454 | Angular velocity | yaw / max_yaw_rate, clamped [-1,1] |

---

### `inference_engine.py` — Neural Network Wrapper

| Element | Description |
|---------|-------------|
| `class InferenceEngine` | Wraps a GaussianPolicy model |
| `__init__(model, device)` | Takes a loaded PyTorch model |
| `get_action(state_vector)` | Input: 1820 floats. Output: (steer, accel) tuple. Steer in [-1,1], accel in [0,2]. Runs `torch.no_grad()`. ~5ms on Jetson CPU. |

---

### `policy_loader.py` — Model Loader

| Element | Description |
|---------|-------------|
| `load_policy(checkpoint_path, device, weights_only)` | Loads .pth file, auto-detects state_dim, action_dim, hidden_sizes from weight shapes. Handles `torch.compile` prefix stripping. Includes numpy._core compat fix for PC→Jetson loading. Returns GaussianPolicy in eval mode. |

**Auto-detection logic:**
- `state_dim` ← `backbone.0.weight.shape[1]`
- `action_dim` ← `mean_layer.weight.shape[0]`
- `hidden_sizes` ← shapes of `backbone.{0,3,6,...}.weight`

---

### `control_mapper.py` — Action-to-Command Mapper

| Element | Description |
|---------|-------------|
| `class ControlMapper` | Maps [-1,1] NN output to physical commands |
| `__init__(max_steering_deg, speed_limit, speed_sign, steer_sign, ...)` | Configures limits, signs, rate limiting |
| `map(steer_raw, accel_raw)` | Returns `AckermannDriveStamped`. Applies: sign inversion, angle scaling, speed limiting, rate limiting, safe mode scaling. |

**Key parameters:**
- `speed_sign`, `steer_sign`: -1.0 for sim→real inversion
- `max_steering_angle_deg`: Physical servo limit (20deg)
- `speed_limit_mps`: Maximum allowed speed (2.0 m/s default)
- `safe_mode`: Enables additional scaling factors

---

### `config/driver_params.yaml` — Full Configuration Reference

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `model.path` | string | — | Path to .pth weights file |
| `model.device` | string | `"cpu"` | PyTorch device |
| `model.weights_only` | bool | `false` | torch.load weights_only flag |
| `lidar.front_step_deg` | float | `0.5` | Front hemisphere angular step (0 = use angles_deg list) |
| `lidar.rear_step_deg` | float | `2.0` | Rear hemisphere angular step (0 = use angles_deg list) |
| `lidar.angles_deg` | float[] | 27 angles | Explicit target angles (overridden when front/rear step > 0) |
| `lidar.angle_offset_deg` | float | `-90.0` | Lidar frame offset |
| `lidar.max_range_m` | float | `20.0` | Max lidar range for normalization |
| `lidar.use_interpolation` | bool | `true` | Interpolate between scan indices |
| `state.stack_frames` | int | `4` | Number of frames to stack |
| `state.max_speed_mps` | float | `6.0` | Speed normalization divisor |
| `state.max_accel_mps2` | float | `4.0` | Acceleration normalization divisor |
| `state.max_yaw_rate_rad_s` | float | `3.0` | Yaw rate normalization divisor |
| `state.servo_norm_divisor` | float | `0.435` | Servo normalization divisor (for [-1,1] centered) |
| `state.servo_norm_offset` | float | `-0.535` | Servo normalization offset |
| `state.servo_default` | float | `0.0` | Default servo value (0 = center in [-1,1]) |
| `control.rate_hz` | float | `30.0` | Control loop frequency |
| `control.enable_on_start` | bool | `false` | Auto-enable on node start |
| `control.speed_sign` | float | `-1.0` | Speed direction flip |
| `control.steer_sign` | float | `-1.0` | Steering direction flip |
| `control.max_steering_angle_deg` | float | `20.0` | Max steering angle |
| `control.speed_limit_mps` | float | `2.0` | Speed cap |
| `control.safe_mode` | bool | `true` | Enable safe mode scaling |
| `safety.watchdog_timeout_sec` | float | `0.5` | Data timeout before stop |
| `topics.scan` | string | `"/scan"` | Lidar topic |
| `topics.odom` | string | `"/odom"` | Odometry topic |
| `topics.servo` | string | `"/commands/servo/position"` | Servo topic |
| `topics.drive` | string | `"/drive"` | Output drive topic |
| `topics.autonomy_lock` | string | `"/autonomy_lock"` | Deadman switch topic |

---

## f1tenth_stack

**Purpose:** Vehicle hardware bringup and joystick control.

### `joy_mode_manager.py` — Joystick Mode Controller

| Element | Description |
|---------|-------------|
| `class JoyModeManager(Node)` | Manages manual/autonomy mode switching |
| `_joy_callback(msg)` | Reads LB (btn 4) for manual, RB (btn 5) for autonomy |
| `_publish_lock()` | Publishes `/autonomy_lock` at 50Hz. Logic: `lock = NOT autonomy_active OR manual_active` |
| `/teleop_gated` | Publishes gated teleop commands (only when LB held) |

### `bringup_launch3.py` — Main Launch File

Launches the full hardware stack:
- SLAMTEC RPLiDAR driver (`sllidar_ros2`)
- VESC motor controller driver + odom conversion
- Joystick driver + joy_mode_manager
- Ackermann mux
- Static transform publishers (base_link → laser, etc.)

---

## ackermann_mux

**Purpose:** Priority-based Ackermann command multiplexer (C++).

| Feature | Description |
|---------|-------------|
| **Priority channels** | Joystick (100, highest) and Navigation (10, lowest) |
| **Lock topic** | `/autonomy_lock` blocks the navigation channel |
| **Output** | Forwards highest-priority active command to `/ackermann_cmd` → VESC |
| **Timeout** | Channels expire after configurable timeout (no stale commands) |

---

## ros2_panel

**Purpose:** GTK3 desktop application for managing ROS2 nodes.

**Location:** `/home/laptop/ros2_panel/` (separate from the ROS2 workspace)

### `panel_app.py` — Main Application

| Class | Description |
|-------|-------------|
| `BootOverlay` | Animated splash screen (fade-in → hold → fade-out) |
| `LedIndicator` | Custom Cairo-drawn LED with pulse animation. States: stopped (red), starting (yellow), running (green), disabled (grey) |
| `ToggleSwitch` | Animated on/off toggle switch widget |
| `ProcessCard` | Card combining LED + label + toggle for one process |
| `ROS2Panel` | Main window. 2x3 grid of process cards + debug console. Polls ProcessManager at 1Hz for status, 500ms for logs |

**Managed processes:** SETUP, Bringup, SLAM, AI Inference, Localize, Pursuit, Stanley

### `process_manager.py` — Process Lifecycle Manager

| Class/Function | Description |
|----------------|-------------|
| `ProcessManager` | Manages subprocess lifecycle for all ROS2 launch processes |
| `start(process_id)` | Launches process in background with `preexec_fn=os.setsid` for group kill |
| `stop(process_id)` | Sends SIGINT → waits 3s → SIGKILL if needed → kills orphan nodes |
| `_kill_ros2_orphans()` | Kills zombie ROS2 nodes (sllidar, vesc, joy, mux, etc.) that survive after launch termination |
| `_determine_state()` | Returns "starting" or "running" based on uptime or lidar detection |
| `get_status()` | Returns state of all processes |
| `get_logs(count)` | Returns last N lines from combined log buffer |

### `scan_test.py` — Lidar Calibration Utility

Diagnostic tool that reads one `/scan` message and identifies the closest 20 points. Used to verify the lidar frame offset (-90deg) by placing an object directly in front of the vehicle.

### `panel.css` — Dark Theme Stylesheet

Dark blue/cyan color scheme (`#1a1a2e` background, `#00d4ff` accent). Styled components: process cards, LED indicators, debug console with green monospace text, scrollbars.

---

## particle_filter

**Purpose:** Monte Carlo Localization using a particle filter on an occupancy grid map.

| File | Description |
|------|-------------|
| `particle_filter.py` | Main particle filter node. Maintains a set of weighted particles, resamples based on lidar scan matching against a known map. |
| `utils.py` | Helper functions for coordinate transforms, map operations |
| `localize_launch.py` | Launch file with map and parameter configuration |

Uses `range_libc` for fast ray casting on occupancy grids.

---

## pure_pursuit

**Purpose:** Geometric path tracking controller. Originally by [Steven Gong](https://github.com/CL2-UWaterloo).

| File | Description |
|------|-------------|
| `pure_pursuit.cpp` | Core Pure Pursuit algorithm. Follows pre-recorded waypoints using lookahead distance. Publishes Ackermann commands. |
| `waypoint_visualizer.cpp` | RViz marker publisher for waypoint visualization |
| `pure_pursuit_launch.py` | Launch file for real car |
| `sim_pure_pursuit_launch.py` | Launch file for F1TENTH simulator |

---

## stanley_avoidance

**Purpose:** Stanley path tracking controller with obstacle avoidance. Originally by [Steven Gong](https://github.com/CL2-UWaterloo).

| File | Description |
|------|-------------|
| `stanley_avoidance.py` | Front-axle Stanley controller with integrated reactive obstacle avoidance via lidar gap detection. |
| `stanley_avoidance_launch.py` | Launch file for real car |
| `sim_stanley_avoidance_launch.py` | Launch file for simulator |
| `sim_multi_agent_stanley_avoidance_launch.py` | Multi-agent racing scenario |

---

## Utilities

### waypoint_generator (C++)
Records odometry positions to CSV at configurable minimum distance intervals. Used to create raceline waypoints for pure_pursuit and stanley controllers.

### gap_follow (F1TENTH Lab)
Reactive gap-following algorithm template. Subscribes to `/scan`, publishes to `/drive`.

### wall_follow (F1TENTH Lab)
PID-based wall following algorithm template. Maintains a set distance from a wall using lidar.

### safety_node (F1TENTH Lab)
Emergency braking node. Computes Time-To-Collision (TTC) from lidar ranges and vehicle speed. Publishes emergency stop if TTC is below threshold.

### sllidar_ros2
SLAMTEC RPLiDAR ROS2 driver. Publishes `/scan` (LaserScan) topic.

### slam_toolbox
ROS2 SLAM implementation for online/offline mapping.

### scan_matching
Scan matching algorithms for lidar-based localization.

### range_libc
Fast ray casting library for particle filter localization. Pre-compiled for the target platform.

### teleop_tools
Joystick teleop utilities for manual driving.
