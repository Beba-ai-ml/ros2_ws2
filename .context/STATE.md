# SAC Driver - Current State

## 2026-09-23 - direct Jetson audit and GitHub synchronization

Read [RESEARCH-jetson-20260923.md](RESEARCH-jetson-20260923.md) first. The local changes were
preserved in `8ca0698` and merged with the remote handoff `2df1fc0` on
`fix/sim-parity-20260913`. No calibration values, running nodes or autonomy state were changed
during this audit. The user explicitly prohibited enabling autonomy.

- Live `/sac_driver` parameters are **offset -90, direction -1**, mpo2 policy, 60 Hz tick,
  decision every 8 ticks. Source and installed driver YAML and Python modules match.
- `/autonomy_lock=true`; sampled `/drive` commands are zero. VESC exited at 14:02 because
  `/dev/vesc` was missing; USB enumeration has no expected VESC device. No `/sensors/core`
  publisher or `/odom` messages in the 18-second audit. The AI process is alive but cannot
  perform the normal inference path in this state.
- `/scan`: 720 rays, about **10 Hz**, frame `laser`; saved cardboard captures and the user's
  current left box support raw front 0°, left +90°, right -90°. Replay with -90/-1 places
  these on the expected model sides; +90/-1 rotates them by 180°.
- Live TF `base_link -> laser` is `(0.27, 0, 0.11)`, yaw **π**, inconsistent with the measured
  raw directions. Python fallbacks are still +90. These values were preserved for review.
- Existing offline guard: **11/15 pass, four lidar tests fail** because they assume the
  opposite raw frame. Older statements that all parity tests pass are historical.
- Default DDS discovery caused missing nodes and parameter timeouts. An isolated UDP-only
  diagnostic profile with explicit loopback peers read the active parameters and TF without
  restarting any existing process. Underlying DDS cause remains unproven.
- Panel runs from `~/ros2_panel`, outside the repo, with equivalent bringup/AI/SLAM commands.
  SLAM was not running. `key_drive.service` is **inactive and disabled**.

## ✅ 2026-09-22 - default policy switched to the validated mpo2 training run
The current default is `weights/session_Sesja_mpo2_2_policy.pth`, a policy-only export of
`/home/beba/occupancy_racer/Soft_Actor_Critic_2/runs/session_Sesja_mpo2_2/session_Sesja_mpo2_2.pth`.
The source run used `mpo2`, 450-ray lidar, state 1820, stack 4 and action repeat 8; it logged
7,131 episodes, peak mean_100 195 m and max single episode 256.6 m. The full training checkpoint
stays on the PC; the repo contains the ~5 MB export that `policy_loader.py` can load on Jetson.
The later direct audit above supersedes the earlier claim that +90 and TF yaw π were
physically validated. The default model remains unchanged; scan/TF consistency and physical
steering validation are still pending.

## 2026-09-23 - read-only ROS input diagnostic
Added `tools/ros2_input_diagnostic.py`. It subscribes to `/scan`, `/odom`, `/drive`, and
`/commands/servo/position`; reports raw nearest scan angle, the nearest ray after the AI converter,
message ages/rate, odometry signs, drive command, and servo value. It only reads topics and uses
the checked-in `driver_params.yaml` for the AI-ray estimate by default; `--angle-offset` and
`--angle-direction` can override it with live values from `ros2 param get`. Side labels assume
`laser` yaw is 0 relative to `base_link`; the physical scan orientation still needs to be checked
on the car. `--capture` prompts for one cardboard position per Enter, waits for two fresh scans so
the next capture is not a sweep already in progress, then appends raw ranges, converted AI rays,
odometry, drive, and servo values to `log/*.jsonl`.

Conversation details are in `.context/HANDOFF-jetson_migracja_1.md`; the direct audit now
confirms live -90/-1. Captured AI rays are diagnostic-side conversions, not a readout of the
network's internal state. Re-query after future configuration changes.

## 🔴 2026-09-13 — sim↔car parity fix, NOT YET DRIVEN ON THE CAR
Branch `fix/sim-parity-20260913`. Review with evidence: `.context/review-jazda-ai-20260913.md`.
Fixed four hard mismatches between the node and the training simulator (all four were enough on
their own to make the car "do big strange things" while the sim policy was fine):
1. lidar frame rotated by 90° (`angle_offset_deg 0` → `-90`, `angle_direction 1` → `-1`);
2. state channels [450..452] were `[speed, steer(-1..1), accel_feedback]`, sim has
   `[collision, speed, servo(0..1)]`;
3. speed divisor 6.0 → 2.5 (training physics);
4. weights were early snapshots of the weakest session → final `car_1_2` / `car_1_3` policies.
Also: tick 60 Hz + policy every 8th tick (sim action_repeat), sim steering curve, no dependency
on `/commands/servo/position`. Offline guard: `src/sac_driver/test/test_sim_parity.py` (15 tests,
12 of them fail on the old code).
**Before driving on the ground: wheels up, run the LEFT/RIGHT cardboard test from
docs/TROUBLESHOOTING.md** — the lidar direction/offset came from the simulator code and the
old March cardboard verdict is not trustworthy (front-only test, garbled state channels).

## What Works
- Full inference pipeline: lidar → state → NN → VESC commands (drove on the real car before the
  2026-09-13 parity fix; the fixed pipeline still needs its first run)
- **Active model `weights/session_Sesja_mpo2_2_policy.pth`** (mpo2, peak mean_100 195 m) —
  450-ray variable-resolution lidar, 1820-dim state, hidden [512,512,256]; alternatives
  `session_car_1_2_policy.pth` (R_01, 220 m) and `session_car_1_3_final_policy.pth`.
- 450-angle lidar extraction with variable step (0.5° front, 2.0° rear); active YAML/live
  mapping is `raw = 90° - sim` (`offset -90`, `direction -1`), with TF still under review
- 4-frame stacking (1820-float state vector: 455 x 4 ticks at 60 Hz)
- Observation per frame: [450 lidar, collision=0, speed_norm(/2.5), servo_norm(0..1), linear_accel, angular_vel]
- Deadman switch via `/autonomy_lock` (hold RB to drive, release to stop); LB overrides
- Safe mode, rate limiting, watchdog, speed cap 2.0 m/s
- Auto-detection of model architecture from .pth weights; `_orig_mod.` prefix stripping;
  numpy._core compatibility for PC→Jetson checkpoints
- Speed sign = -1.0 (verified on the car), steer sign = +1.0 (pairs with `angle_direction -1`)
- **Model weights tracked in git** and installed into the package share dir → `model.path` is
  relative and portable
- **Keyboard drive** (`scripts/key_drive.sh` / `key_drive.py`) — X11, evdev and terminal backends
- **key_drive.service** — headless bringup + keyboard drive at boot, supervises the VESC chain
- **ros2_panel** in the repo — battery bar, SLAM + RViz, AI Inference toggle, zombie killer
- Inference time: ~5-6 ms on the Jetson CPU (tick budget at 60 Hz is 16 ms; the policy runs only every 8th tick)

## Work in Progress
- `install.sh` — one-shot installer for a fresh Jetson; written, **not yet validated on a clean machine**
- Stability of the bringup/restart cycle — the lidar sometimes fails to reconnect after a restart
- First physical run of the parity-fixed node with the mpo2 policy (cardboard left/right test,
  then ground run at 2 m/s)

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
- **Historical 2026-09-08 note:** model `session_Rybnik_02_1.pth` was then the active policy
  (trained on the Rybnik_02 map).
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
- **60 Hz control tick despite 8 Hz lidar** — odom updates at ~50 Hz, so speed/accel/yaw change
  between lidar frames; the policy is queried every 8th tick and the held action is published
  between decisions.
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
- **`key_drive.service` is disabled and inactive on 2026-09-23.** If enabled later, keyboard
  driving starts at boot without a deadman button; always re-check before hardware work.
- **NFS mount disconnects** — `/home/laptop/shared` may not be mounted after a reboot. Not
  critical (weights are local). Fix: `sudo mount /home/laptop/shared`.
- **Servo subscription removed (2026-09-13)** — steering feedback is the node's own last command,
  `_data_ready()` waits only for `/scan` and `/odom`.
- **Safe mode is a no-op** — `safe_steer_scale = 1.0` and `safe_accel_scale = 1.0` apply no
  scaling. For cautious first runs set them below 1.0.
- **Steering feedback channel [452]** is `(previous steer + 1) / 2` after the 2026-09-13 fix;
  older notes describing this channel as acceleration feedback are obsolete.
- **`bringup_launch3.py` lidar `serial_port` defaults to `/dev/ttyUSB0`**, not the `/dev/rplidar`
  udev symlink — pass `serial_port:=/dev/rplidar` if the enumeration order is unstable.
- **`sensors.yaml`** still holds the legacy Hokuyo/`urg_node` settings and is unused.
- **`sudo -n` everywhere** — `scripts/key_drive.sh` and the panel rely on `/etc/sudoers.d/f1tenth`
  (installed by `install.sh`, and installed on this car on 2026-09-08). Without it, panel shutdown and
  the key_drive.sh service pause/resume silently do nothing.

## Config Summary (driver_params.yaml)
```yaml
model.path: "weights/session_Sesja_mpo2_2_policy.pth"  # relative to the package share dir
model.device: "cpu"
model.weights_only: false
lidar.front_step_deg: 0.5   # 450-ray variable resolution
lidar.rear_step_deg: 2.0
lidar.angle_offset_deg: -90.0  # confirmed live/YAML; sim front -> raw 0 deg
lidar.angle_direction: -1.0    # sim 0 deg -> raw +90 deg (left in captures)
lidar.max_range_m: 20.0
state.stack_frames: 4
state.max_speed_mps: 2.5    # training physics max_speed
state.max_accel_mps2: 4.0
state.max_yaw_rate_rad_s: 3.0
control.speed_sign: -1.0   # positive drive.speed = REVERSE on this car
control.steer_sign: 1.0
control.rate_hz: 60.0          # sim frame
control.decision_every_n: 8    # sim action_repeat
control.max_steering_angle_deg: 20.0
control.min_steering_angle_deg: 5.0
control.steer_speed_ref_mps: 8.0
control.max_speed_mps: 2.5
control.max_accel_mps2: 2.0
control.speed_limit_mps: 2.0
control.wheelbase_m: 0.35
control.safe_mode: true
safety.watchdog_timeout_sec: 0.5
```

## Next Steps
0. Resolve missing VESC USB/power first, then reconcile the measured raw frame, retained TF
   yaw π, Python fallbacks and offline test assumptions using the research report. Any
   necessary restart must be coordinated with the user; autonomy remains prohibited here.
1. **Wheels up: LEFT/RIGHT cardboard test** (docs/TROUBLESHOOTING.md) with the parity-fixed
   node, then first ground run at `speed_limit_mps 2.0` with `session_Sesja_mpo2_2_policy.pth`;
   compare with `session_car_1_2_policy.pth` only as the R_01 alternative.
2. Finish and validate `install.sh` on a clean Jetson
3. Fix the lidar reconnect-after-restart issue
4. Point the bringup lidar `serial_port` at `/dev/rplidar` instead of `/dev/ttyUSB0`
5. Re-enable / evaluate `throttle_interpolator` (config is ready, node commented out)
6. Add IMU-based acceleration if odom-derived accel proves too noisy
7. Wheelbase: the car has 0.35 m, the policies were trained at 0.27 m (DR 0.23-0.31) — if
   cornering is off after the fixes, retrain with `wheelbase: 0.35` in `physics.yaml`

---

## History

### 2026-09-22
- Default policy changed from `session_car_1_2_policy.pth` to the policy-only
  `session_Sesja_mpo2_2_policy.pth`, exported from the final `Sesja_mpo2_2` checkpoint.
- `install.sh`, launch defaults and lidar diagnostic now point to the mpo2 policy. Earlier
  +90/TF π assumptions are superseded by the 2026-09-23 direct audit above.
- **Panel SETUP no longer depends on the optional LED strip** — removed the missing `~/ros2_ws/ledy.py`/SPI setup from all panel variants and made `/dev/rplidar` and `/dev/vesc` permission changes conditional on those devices existing.

### 2026-09-13
- **Sim↔car parity fix** (see the red block at the top): lidar frame `offset -90 / direction -1`,
  observation layout `[lidar, collision, speed, servo 0..1, accel, yaw]`, speed divisor 2.5,
  60 Hz tick + decision every 8 ticks, simulator steering curve in `ControlMapper`, steering
  feedback from our own command (servo subscription and `state.servo_*` / `topics.servo`
  parameters removed). Weights replaced by policy-only exports of the final `car_1_2` and
  `car_1_3` checkpoints (`Rybnik_02_1` and the episode-5250 `car_1_3` snapshot removed).
  New offline test `src/sac_driver/test/test_sim_parity.py`. Docs (README, DOCUMENTATION,
  KNOWLEDGE, TROUBLESHOOTING) updated - the old texts described the wrong layout as fact.

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

Last updated: 2026-09-23
