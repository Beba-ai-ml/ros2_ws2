# SAC Driver - Current State

## What Works
- Full inference pipeline: lidar → state → NN → VESC commands — **TESTED ON REAL CAR, IT DRIVES**
- **Active model `session_Rybnik_02_1.pth`** (Rybnik_02 map) — 450-ray variable-resolution lidar,
  1820-dim state, hidden [512,512,256]; previous model `session_car_1_3.pth` kept alongside
- 450-angle lidar extraction with variable step (0.5° front, 2.0° rear) and **0° offset**
  (verified by the cardboard test)
- 4-frame stacking (1820-float state vector: 455 x 4 frames)
- Observation: [450 lidar, speed_norm, steer_norm, accel_feedback, linear_accel, angular_vel] per frame
- Deadman switch via `/autonomy_lock` (hold RB to drive, release to stop); LB overrides
- Safe mode, rate limiting, watchdog, speed cap 2.0 m/s
- Auto-detection of model architecture from .pth weights; `_orig_mod.` prefix stripping;
  numpy._core compatibility for PC→Jetson checkpoints
- Speed sign = -1.0, steer sign = +1.0 (verified by cardboard test with offset=0)
- **Model weights tracked in git** and installed into the package share dir → `model.path` is
  relative and portable
- **Keyboard drive** (`scripts/key_drive.sh` / `key_drive.py`) — X11, evdev and terminal backends
- **key_drive.service** — headless bringup + keyboard drive at boot, supervises the VESC chain
- **ros2_panel** in the repo — battery bar, SLAM + RViz, AI Inference toggle, zombie killer
- Inference time: ~5-6 ms on the Jetson CPU (well within the 30 Hz budget)

## Work in Progress
- `install.sh` — one-shot installer for a fresh Jetson; written, **not yet validated on a clean machine**
- Stability of the bringup/restart cycle — the lidar sometimes fails to reconnect after a restart
- Real-world driving tuning of the Rybnik_02 policy (speed limit, safe mode scales)

## Recent Changes (2026-09-08) — repository portability push

Goal: someone clones this repo on **another Jetson in another car**, runs one installer, and
everything works.

- **New repository layout.** Everything the car needs now lives in the repo:
  - `ros2_panel/` moved in from `~/ros2_panel`; all paths derived from the repo location
  - `local_python/` — the patched `joy_teleop` (required by the bringup) + `gpio_shutdown.py`
  - `scripts/` — `key_drive.sh`, `key_drive.py`, `key_drive_boot.sh`, `key_drive_test.py`, `ledy*.py`
  - `tools/` — `vesc/vesc_config_upload.py` + `configs/` + `params/6.02/`, `lidar_diag.py`, `lidar_test.py`
    (the uploader used to live at `~/vesc_config_upload.py` with configs in `~/Downloads`)
  - `system/` — `udev/99-f1tenth.rules`, `systemd/*.service.in`, `sudoers.d/f1tenth.in`, `desktop/`
  - `legacy/system_web/` — the old Flask web panel, parked and unmaintained
  - `maps/` — added `Rybnik_01`, `Rybnik_02`, `Rybnk_04`, `Rybnk_05`
  - `requirements.txt` — torch 1.13.1 CPU, numpy 1.24.4, pyserial, evdev, Jetson.GPIO,
    rpi-ws281x, Adafruit Blinka + neopixel-spi
- **`install.sh`** (new): default run does checks + apt ROS packages + `rosdep install` +
  `pip --user -r requirements.txt` + udev rules + `spidev` modules-load + groups
  (dialout gpio input plugdev video) + sudoers NOPASSWD + `~/ros2_ws` symlink + `.bashrc`
  sourcing + `colcon build --packages-select sac_driver f1tenth_stack ackermann_mux sllidar_ros2`.
  Flags: `--check`, `--with-ros`, `--full`, `--no-build`, `--gpio-shutdown`,
  `--key-drive-service`, `--desktop`, `--yes`. Reboot afterwards; enable SPI via jetson-io if
  using the LEDs.
- **Model `session_Rybnik_02_1.pth`** is now the active policy (trained on the Rybnik_02 map).
  Weights are tracked in git (`.gitignore`: `!src/sac_driver/weights/*.pth`) and installed into
  `share/sac_driver/weights/` by `setup.py`; `model.path` became relative
  (`weights/session_Rybnik_02_1.pth`) and is resolved against the share dir, with support for
  absolute, `~` and `package://sac_driver/...` paths.
- **Keyboard drive** (2026-09-08): `scripts/key_drive.sh` builds `f1tenth_stack`, starts the
  bringup, waits for `/sensors/core`, then runs `key_drive.py`, which publishes
  `AckermannDriveStamped` on `/teleop_gated` (mux priority 100, not masked by `/autonomy_lock`).
  Arrows/WASD, Space stop, `+`/`-` speed, defaults 2.0 m/s (max 4.0), 0.3 rad. Backends: X11
  `XQueryKeymap` (NoMachine, `DISPLAY=:1004`), `--evdev` for a keyboard plugged into the Jetson,
  `--no-x` terminal fallback. `q` quits; `KEEP_BRINGUP=1` leaves the bringup running.
  `SPEED_SIGN = -1.0` — on this car positive `drive.speed` is reverse (user-confirmed).
- **`key_drive.service`** (`scripts/key_drive_boot.sh`, systemd, `User=laptop`, `Restart=always`):
  headless autostart of bringup + `key_drive --evdev` at boot; supervises the VESC chain and
  restarts everything on failure. **ENABLED on this car** → the car responds to a plugged-in
  keyboard right after boot, with no deadman. Wheels off the ground when testing;
  `sudo systemctl disable --now key_drive.service` to turn it off. In the repo it is optional
  (`install.sh --key-drive-service`).
- **`vesc.yaml`**: `speed_min`/`speed_max` raised from ±4250 to **±45250 erpm**;
  `throttle_interpolator.max_acceleration` 2.5 → **9.5** (the node itself is still commented out
  in `bringup_launch3.py`).
- **`scripts/bt_pad_connect.sh`** (new): Bluetooth gamepad auto-connect loop (DualShock 4 style
  pad, `PAD_MAC` overridable). Runs on this car as `bt_pad.service` (enabled), logging to
  `log/bt_pad.log`. Templated as `system/systemd/bt_pad.service.in`, installed with `install.sh --bt-pad`.
- **Documentation rewrite**: `README.md` (hardware table corrected to Jetson Orin Nano Super,
  one-block Quick Start, "First drive on a NEW car" checklist), `DOCUMENTATION.md` (added
  scripts/tools/system/local_python sections, fixed panel paths and config keys), new
  `AGENTS.md` and `CLAUDE.md` for AI coding agents, new `docs/SETUP_NEW_JETSON.md` and
  `docs/TROUBLESHOOTING.md`.

## Decisions
- **Everything the car needs lives in the repository** — no dependency on files in `$HOME`, on
  the NFS share, or on a manually configured machine. The NFS share is now optional and only
  used for exchanging weights/logs with the training PC.
- **Weights committed to git** despite their size (~47-58 MB each) — a clone must be able to
  drive without a separate download step.
- **Relative `model.path` resolved against the package share dir** — absolute paths break on
  every other machine and user account.
- **`sudo -n` + `/etc/sudoers.d/f1tenth`** for the few privileged operations; no passwords
  anywhere in the repository.
- **Boot service is opt-in** (`install.sh --key-drive-service`) even though it is enabled on this
  car — autostarting a driving car is not a safe default.
- **Acceleration from odom speed delta** (not IMU) — computed in `_on_odom()` as
  `(current_speed - prev_speed) / dt`. **Yaw rate from odom twist.**
- **30 Hz control rate despite 8 Hz lidar** — odom updates at ~50 Hz, so speed/accel/yaw change
  between lidar frames; smoother output and better rate-limiter behaviour.
- **All RELIABLE QoS** — BEST_EFFORT causes silent message drops on this Jetson/DDS setup.
- **SIGINT before SIGKILL** for process termination — ROS2 nodes handle SIGINT gracefully.
- **`src/slam_toolbox` is not built** — the apt package is used; only its config file matters.

## Known Issues
- **Lidar does not always reconnect after a bringup restart** — a zombie `sllidar_node` holds
  the port, or the CP210x bridge needs a replug. `pkill -f sllidar`, wait, relaunch.
- **Zombie nodes after bringup stop** — `ros2 launch` spawns nodes in separate process groups.
  The panel's `stop()` calls `_kill_ros2_orphans()` but may not catch every case. Manual
  cleanup: `pkill -f sllidar && pkill -f vesc && pkill -f joy && pkill -f ackermann_mux`.
- **Two VESC drivers on one port** — a manual bringup plus `key_drive.service` produces silent,
  undefined behaviour. Check `pgrep -af vesc_driver_node` and
  `systemctl is-active key_drive.service` first.
- **`key_drive.service` is enabled on this car** — the car drives from a plugged-in keyboard
  right after boot, without a deadman button. Safety-relevant for anyone working on the hardware.
- **NFS mount disconnects** — `/home/laptop/shared` may not be mounted after a reboot. Not
  critical (weights are local). Fix: `sudo mount /home/laptop/shared`.
- **Servo data optional** — `/commands/servo/position` only publishes when the car is actively
  driven; `_data_ready()` requires it when the servo subscription exists, which can block startup
  until someone drives manually once.
- **Safe mode is a no-op** — `safe_steer_scale = 1.0` and `safe_accel_scale = 1.0` apply no
  scaling. For cautious first runs set them below 1.0.
- **Accel feedback channel interpretation** — channel [452] is assumed to be the previous NN raw
  accel action. If the training env uses a different value, behaviour may be suboptimal.
- **`bringup_launch3.py` lidar `serial_port` defaults to `/dev/ttyUSB0`**, not the `/dev/rplidar`
  udev symlink — pass `serial_port:=/dev/rplidar` if the enumeration order is unstable.
- **`sensors.yaml`** still holds the legacy Hokuyo/`urg_node` settings and is unused.
- **`sudo -n` everywhere** — `scripts/key_drive.sh` and the panel rely on `/etc/sudoers.d/f1tenth`
  (installed by `install.sh`, and installed on this car on 2026-09-08). Without it, panel shutdown and
  the key_drive.sh service pause/resume silently do nothing.

## Config Summary (driver_params.yaml)
```yaml
model.path: "weights/session_Rybnik_02_1.pth"   # relative to the package share dir
model.device: "cpu"
model.weights_only: false
lidar.front_step_deg: 0.5   # 450-ray variable resolution
lidar.rear_step_deg: 2.0
lidar.angle_offset_deg: 0.0 # 0 deg = forward for the current models
lidar.angle_direction: 1.0
lidar.max_range_m: 20.0
state.stack_frames: 4
state.max_speed_mps: 6.0
state.max_accel_mps2: 4.0
state.max_yaw_rate_rad_s: 3.0
state.servo_norm_divisor: 0.435   # centered [-1,1]
state.servo_norm_offset: -0.535
state.servo_default: 0.0
control.speed_sign: -1.0   # positive drive.speed = REVERSE on this car
control.steer_sign: 1.0
control.rate_hz: 30.0
control.max_steering_angle_deg: 20.0
control.max_speed_mps: 6.0
control.max_accel_mps2: 2.0
control.speed_limit_mps: 2.0
control.wheelbase_m: 0.35
control.safe_mode: true
safety.watchdog_timeout_sec: 0.5
```

## Next Steps
1. Finish and validate `install.sh` on a clean Jetson (the real test of this whole push)
2. Real-world driving tests with `session_Rybnik_02_1`; tune `speed_limit_mps` and the safe-mode
   scales from the results
3. Fix the lidar reconnect-after-restart issue
4. Point the bringup lidar `serial_port` at `/dev/rplidar` instead of `/dev/ttyUSB0`
5. Verify the accel_feedback channel [452] interpretation against the PC training code
6. Consider making the servo subscription non-blocking in `_data_ready()`
7. Re-enable / evaluate `throttle_interpolator` (config is ready, node commented out)
8. Add IMU-based acceleration if odom-derived accel proves too noisy

---

## History

### 2026-03-28
- **Lidar offset fixed: -90° → 0°** — the 450-ray model uses the 0°=forward convention.
  Diagnosed with the cardboard test; `steer_sign` changed from -1.0 to +1.0 to match.
- **Model weights moved locally** — copied from NFS to `src/sac_driver/weights/`.
  PyTorch 2.x → 1.13 format conversion: repackaged the zip without `.format_version` files.
- **Battery bar added to ros2_panel** — `voltage_input` from `/sensors/core` every 2 s.
- **SLAM auto-launches RViz** — `config/slam_rviz.rviz`, SLAM foreground / RViz background.
- **Debug console background** — CSS forced to `#000000`.

### 2026-03-26
- **Switched to the session_car_1_3 model** — 450-ray lidar, 1820-dim state (was 128),
  hidden [512,512,256]. Added `build_lidar_angles()`; new observation format (collision flag
  removed, accel_feedback added); servo normalization moved from [0,1] to [-1,1] centered at
  0.535; numpy._core compatibility fix; old 27-ray config kept as `driver_params_27ray.yaml`.
- **AI Inference button added to ros2_panel** (replaced the "Reserved" slot).

### 2026-03-13
- **VESC motor & app configuration uploaded via a custom Python script** — no VESC Tool needed.
  Motor config 189 params (49 changed), app config 187 params (23 changed), both verified by
  read-back. FW 6.02, HW60. Signatures: Motor `0x2E43A161`, App `0x1D003A2C`.

### 2026-03-09
- Added README.md and DOCUMENTATION.md, added .gitignore, published to
  https://github.com/Beba-ai-ml/ros2_ws2

### 2026-03-02
- **Observation vector expanded from 30 to 32 elements** — added `linear_acceleration` and
  `angular_velocity` (state 32 x 4 = 128 floats).
- **Fixed QoS mismatch** — `/scan` moved from `qos_profile_sensor_data` (BEST_EFFORT) to QoS 10
  (RELIABLE); BEST_EFFORT was silently dropping every lidar message.
- **Fixed variable shadowing** — `accel`/`yaw` renamed to `sensor_accel`/`sensor_yaw`.
- **Added debug logging** for autonomy transitions and data readiness.
- **Flipped speed_sign and steer_sign to -1.0** — the car drove backwards and steered the wrong way.
- **Fixed ros2_panel process_manager** — added `_kill_ros2_orphans()`.

Last updated: 2026-09-08
