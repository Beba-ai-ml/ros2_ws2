# SAC Driver - Current State

## What Works
- Full inference pipeline: lidar → state → NN → VESC commands — **TESTED ON REAL CAR, IT DRIVES**
- **New model `session_car_1_3.pth`** — 450-ray variable-resolution lidar, 1820-dim state, hidden [512,512,256]
- 450-angle lidar extraction with variable step (0.5° front, 2.0° rear) and -90deg offset
- 4-frame stacking (1820-float state vector: 455 x 4 frames)
- Observation: [450 lidar, speed_norm, steer_norm, accel_feedback, linear_accel, angular_vel] per frame
- Deadman switch via `/autonomy_lock` topic (hold RB to drive, release to stop)
- Safe mode with speed/steer/accel scaling
- Rate limiting on steering and acceleration
- Auto-detection of model architecture from .pth weights
- Numpy compatibility fix for PC→Jetson checkpoint loading (numpy._core alias)
- Debug logging for autonomy state changes and data readiness
- Speed sign and steer sign correctly inverted for real car (both -1.0)
- **AI Inference button in ros2_panel** — one-click start/stop from GTK panel
- Inference time: ~6ms on Jetson CPU (well within 30Hz budget)

## Work in Progress
- Stability of bringup/restart cycle — lidar sometimes fails to reconnect after restart

## Recent Changes (2026-03-26)
- **Switched to session_car_1_3 model** — 450-ray lidar, 1820-dim state (was 128), hidden [512,512,256]
  - Added `build_lidar_angles()` for variable-resolution lidar (0.5° front, 2.0° rear = 450 rays)
  - New observation format: removed collision flag, added accel_feedback (previous NN action)
  - Channel order: [speed_norm, steer_norm, accel_feedback, linear_accel, angular_vel]
  - Servo normalization changed from [0,1] to [-1,1] centered at 0.535
  - Added numpy._core compatibility fix in policy_loader for cross-platform checkpoint loading
  - Model: action_scale=[1,1], action_bias=[0,1] → steer [-1,1], accel [0,2]
  - Physics: max_speed=6.0, max_accel=2.0, max_steer=20°
  - Old 27-ray config backed up as `driver_params_27ray.yaml`
- **AI Inference button added to ros2_panel** — replaced "Reserved" slot with working AI toggle
  - Builds sac_driver, launches node with params file
  - Added orphan kill for sac_driver_node on stop
  - Log color: #ff6b6b (red)

### Previous Changes (2026-03-13)
- **VESC motor & app configuration uploaded via custom Python script**
  - Created `/home/laptop/vesc_config_upload.py` — standalone tool to upload VESC XML configs via serial
  - No VESC Tool needed — communicates directly with VESC over `/dev/vesc` (115200 baud) using VESC binary protocol
  - Motor config: 189 params (49 changed from defaults) — FOC motor type, current limits 52.78A, observer gain tuned
  - App config: 187 params (23 changed from defaults) — controller_id=97, servo_out enabled, PPM/ADC configured
  - Both configs verified by reading back from VESC and comparing all values
  - VESC firmware: 6.02, hardware: HW60
  - Config XMLs stored at `/home/laptop/Downloads/vesc_configs/motor_config.xml` and `app_config.xml`
  - Script uses FW 6.02 parameter definitions from VESC Tool source for correct binary serialization
  - Signature verification: Motor=0x2E43A161, App=0x1D003A2C (matched firmware)

### Previous Changes (2026-03-02)
- **Observation vector expanded from 30 to 32 elements**
  - Added `linear_acceleration` (index 30) — computed from odom speed delta/dt
  - Added `angular_velocity` (index 31) — from odom twist.angular.z
  - Total state: 32 x 4 frames = 128 floats (was 30 x 4 = 120)
  - New params: `state.max_accel_mps2` (4.0), `state.max_yaw_rate_rad_s` (3.0)
  - Files changed: `state_builder.py`, `sac_driver_node.py`, `config/driver_params.yaml`
- **Model path updated** to `/home/laptop/shared/różne/best_mapa1_ep18000_clean.pth`
- **numpy blocker resolved** — PC re-saved model as clean state_dict (`_clean.pth`)
- **Fixed variable shadowing** — renamed `accel`/`yaw` to `sensor_accel`/`sensor_yaw` in timer loop
- **Fixed QoS mismatch** — changed `/scan` subscription from `qos_profile_sensor_data` (BEST_EFFORT) to QoS 10 (RELIABLE). BEST_EFFORT was silently dropping all lidar messages.
- **Added debug logging** — `_on_estop` logs state transitions, `_on_timer` logs data readiness
- **Added `_last_stop_sent = False` reset** in `_on_estop` to ensure state changes are visible in logs
- **Flipped speed_sign and steer_sign to -1.0** — car was driving backwards and steering wrong way
- **Fixed ros2_panel process_manager** — added `_kill_ros2_orphans()` to kill zombie ROS2 nodes on bringup stop

## Decisions
- **Acceleration from speed delta** (not IMU) — simpler, no extra sensor dependency. Computed in `_on_odom()` as `(current_speed - prev_speed) / dt`
- **Yaw rate from odom twist** (not IMU) — `msg.twist.twist.angular.z` already available from odom
- **30Hz control rate despite 8Hz lidar** — odom updates at ~50Hz, so speed/accel/yaw change between lidar frames. Smoother control output and better rate limiter behavior
- **Normalization ranges:** accel clamped to [-1,1] via /4.0 m/s², yaw via /3.0 rad/s — matches training environment
- **All RELIABLE QoS** — BEST_EFFORT causes silent message drops on this Jetson/DDS setup
- **SIGINT before SIGTERM** for process kill — ROS2 nodes handle SIGINT more gracefully

## Known Issues
- **Zombie nodes after bringup stop** — `ros2 launch` spawns nodes in separate process groups. Panel's `stop()` was updated with `_kill_ros2_orphans()` but may not catch all cases. Manual cleanup: `pkill -f sllidar && pkill -f vesc && pkill -f joy && pkill -f ackermann_mux`
- **NFS mount disconnects** — after reboot, `/home/laptop/shared` may not be mounted. Model loading fails with "Checkpoint not found". Fix: `sudo mount /home/laptop/shared`
- **Collision flag always 0.0** — no collision sensor on physical car. Compatible with training (collision=0 during normal driving)
- **Servo data optional** — `/commands/servo/position` only publishes when car is actively driven. First frame after startup may have default servo value (0.535). The `_data_ready()` check requires servo if `servo_sub` is not None, which can block startup until manual driving occurs.

## Config Summary (driver_params.yaml)
```yaml
model.path: "/home/laptop/shared/różne/session_car_1_3.pth"
model.device: "cpu"
model.weights_only: false
lidar.front_step_deg: 0.5   # 450-ray variable resolution
lidar.rear_step_deg: 2.0
state.stack_frames: 4
state.max_speed_mps: 6.0
state.max_accel_mps2: 4.0
state.max_yaw_rate_rad_s: 3.0
state.servo_norm_divisor: 0.435   # centered [-1,1]
state.servo_norm_offset: -0.535
state.servo_default: 0.0
control.speed_sign: -1.0   # inverted for real car
control.steer_sign: -1.0   # inverted for real car
control.rate_hz: 30.0
control.max_steering_angle_deg: 20.0
control.max_speed_mps: 6.0
control.max_accel_mps2: 2.0
control.speed_limit_mps: 2.0
control.safe_mode: true
safety.watchdog_timeout_sec: 0.5
```

## Next Steps
1. Make sac_driver a permanent entry in ros2_panel (button to start/stop)
2. Tune speed_limit_mps and safe mode params based on real driving tests
3. Consider making servo subscription optional (don't block on missing servo data)
4. Add IMU-based acceleration if odom-derived accel proves too noisy
5. Profile inference latency on Jetson CPU — may need optimization

## Documentation (2026-03-09)
- Added professional README.md (GitHub release ready)
- Added DOCUMENTATION.md (full module reference)
- Added .gitignore
- Published to https://github.com/Beba-ai-ml/ros2_ws2

Last updated: 2026-03-26
