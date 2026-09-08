# AGENTS.md — guidance for AI coding agents

Read this before touching anything. It is written for Claude Code, Codex and similar agents
working inside this repository.

## What this repo is

1. A ROS2 Foxy workspace for a **physical** 1/10 scale F1TENTH racing car.
2. The car is an NVIDIA Jetson Orin Nano (JetPack 5.1.5, Ubuntu 20.04, Python 3.8).
3. The headline package is `src/sac_driver/` — a Soft Actor-Critic policy running at 30 Hz on
   the CPU, turning lidar + odometry into Ackermann drive commands.
4. `src/f1tenth_stack/` brings up the hardware: lidar, VESC, joystick, command multiplexer.
5. `src/ackermann_mux/` arbitrates between manual teleop (priority 100) and AI (priority 10).
6. `ros2_panel/` is a GTK3 panel that starts/stops those launches.
7. `scripts/key_drive*` is keyboard teleop, optionally started at boot by a systemd unit.
8. `install.sh` sets a fresh Jetson up end to end; `system/` holds the udev/systemd templates.
9. The policy is trained elsewhere (occupancy-racer-sac2); only checkpoints arrive here.
10. **Code in this repo moves a real vehicle.** Mistakes break hardware and hurt people.

## Repo map — edit here / do not edit

| Path | Status |
|------|--------|
| `src/sac_driver/` | **Edit freely.** Main project code. |
| `src/f1tenth_stack/` | **Edit with care.** Launch + configs that define vehicle behaviour. |
| `src/ackermann_mux/` | Edit with care (C++). Safety-critical arbitration. |
| `ros2_panel/`, `scripts/`, `tools/`, `local_python/gpio_shutdown.py` | Edit freely. |
| `install.sh`, `system/`, `requirements.txt` | Edit freely, but keep them in sync with the docs. |
| `README.md`, `DOCUMENTATION.md`, `docs/`, `.context/` | Edit freely; keep factual. |
| `src/sllidar_ros2/`, `src/slam_toolbox/`, `src/range_libc/`, `src/particle_filter/` | **Vendored upstream. Do not refactor, reformat or "improve".** Patch only with a clearly commented, minimal change. |
| `src/f1tenth_system/`, `src/scripts/`, `src/teleop_tools/` | Upstream leftovers / placeholders. Leave alone. |
| `local_python/joy_teleop/` | A deliberately patched copy of the Foxy `joy_teleop` node. Do not resync it with upstream — the patch is required (see below). |
| `legacy/system_web/` | Dead code kept for reference. Do not maintain. |
| `src/sac_driver/weights/*.pth` | Binary checkpoints tracked in git. Never rewrite; add new files instead. |
| `build/`, `install/`, `log/`, `src/install/` | Build artefacts. Git-ignored. Never commit or edit. |

### The joy_teleop patch

`local_python/joy_teleop/joy_teleop.py` differs from `/opt/ros/foxy/.../joy_teleop.py` in one
place: a command with no deadman buttons and no axes raises an exception upstream, but here it
is marked `always_active = True`. `src/f1tenth_stack/config/joy_teleop.yaml` defines exactly
such a `default` command, so **without the patch the bringup fails to start**.
`bringup_launch3.py` prepends `<ws>/local_python` to `PYTHONPATH` for that node only.

## Where truth lives

| Question | File |
|----------|------|
| What is the current state, what changed, what is broken? | `.context/STATE.md` — **update it after every meaningful change** |
| How does the system work (architecture, conventions, lessons)? | `.context/KNOWLEDGE.md` |
| Entry point for the context files | `.context/INDEX.md` |
| Human-facing overview and setup | `README.md` |
| Module/API-level reference | `DOCUMENTATION.md` |
| Fresh-machine bring-up | `docs/SETUP_NEW_JETSON.md` |
| Failure modes and fixes | `docs/TROUBLESHOOTING.md` |

If a doc and the code disagree, the code wins — then fix the doc in the same change.

## Build, run, verify

```bash
source /opt/ros/foxy/setup.bash
cd ~/ros2_ws

# build only what you touched
colcon build --packages-select sac_driver
source install/setup.bash

# syntax check without a build (fast)
python3 -m py_compile src/sac_driver/sac_driver/*.py scripts/*.py ros2_panel/*.py

# environment / prerequisites check, no changes made
./install.sh --check
```

Run:

```bash
ros2 launch f1tenth_stack bringup_launch3.py      # hardware
ros2 launch sac_driver sac_driver.launch.py       # AI, optionally model_path:=<path>
ros2 service call /sac_driver/enable std_srvs/srv/SetBool "{data: true}"
~/ros2_ws/scripts/key_drive.sh                    # keyboard teleop (starts bringup itself)
python3 scripts/key_drive_test.py                 # automated direction test, wheels off ground
```

Inspect:

```bash
ros2 topic hz /scan          # ~8 Hz
ros2 topic hz /odom          # ~50 Hz
ros2 topic echo /joy         # gamepad
ros2 topic echo /sensors/core --field state.speed        # motor ERPM
ros2 topic echo /commands/servo/position                 # servo command
```

There is no automated test suite. "Verified" means: it builds, `py_compile` passes, and the
behaviour was checked on the car (or the user confirmed it was).

## Hardware safety rules

1. **Never publish drive commands** (`/drive`, `/teleop_gated`, `ackermann_cmd`,
   `/commands/motor/*`) unless the user has explicitly confirmed the wheels are off the ground
   or the car is on a clear track. Ask first, every time.
2. **`/dev/vesc` accepts exactly one driver process.** Before starting a bringup, make sure no
   other bringup, no `key_drive.service` and no stale `vesc_driver_node` is running. Two
   drivers on the port produce silent, undefined behaviour.
3. **`key_drive.service` autostarts driving at boot**, with no deadman. Check
   `systemctl is-active key_drive.service` before any hardware work; stop it with
   `sudo systemctl stop key_drive.service`.
4. **Do not change `speed_to_erpm_gain`, `speed_min`/`speed_max`, `servo_min`/`servo_max`,
   `steering_angle_to_servo_gain`/`offset` in `src/f1tenth_stack/config/vesc.yaml`, or
   `control.speed_sign` / `control.steer_sign` / `lidar.angle_offset_deg` in
   `driver_params.yaml`, without a physical re-test.** These are per-car calibration values.
   The sign convention on this car is: positive `drive.speed` = REVERSE, hence
   `speed_sign: -1.0` and `SPEED_SIGN = -1.0` in `scripts/key_drive.py`.
5. Do not raise `control.speed_limit_mps` (2.0 m/s) or disable `control.safe_mode` on the
   user's behalf.
6. Do not remove or weaken the deadman logic in `joy_mode_manager.py`, the mux lock on
   `/autonomy_lock`, or the watchdog in `sac_driver_node.py`.
7. Do not start or kill ROS nodes, or run `colcon build`, when another agent or the user may be
   driving.
8. After stopping a launch, clean up orphans — `ros2 launch` leaves nodes in their own process
   group:
   ```bash
   pkill -f sllidar; pkill -f vesc; pkill -f joy; pkill -f ackermann_mux; pkill -f sac_driver_node
   ```

## Conventions

- **Python 3.8 only.** No `match`, no `|` type unions at runtime, no `dict | dict`, no
  `functools.cache`, no 3.9+ standard library. `from __future__ import annotations` is fine.
- **PyTorch 1.13 API.** No `torch.compile` at inference, no `weights_only=True` default
  semantics from 2.x, no `torch.func`. Checkpoints from newer PyTorch may need repackaging
  (see `docs/TROUBLESHOOTING.md`).
- **ROS2 Foxy API.** No Humble/Iron-only `rclpy` features (e.g. `rclpy.init(args=..., context=)`
  niceties, `Node.declare_parameters` variants added later, lifecycle helpers). Message types
  are the Foxy versions.
- **QoS: always RELIABLE, depth 10.** `qos_profile_sensor_data` (BEST_EFFORT) silently drops
  lidar messages on this Jetson's DDS stack. This has cost days of debugging — do not "fix" it.
- **No magic numbers.** Every normalization constant, limit and topic name comes from
  `driver_params.yaml`. If you need a new constant, add a parameter.
- **Paths must be portable.** Derive paths from the package share directory, `__file__`, or
  `~/ros2_ws`. Do not hardcode `/home/laptop`.
- **Model paths** are resolved by `_resolve_path` in `sac_driver_node.py`: relative → package
  share dir (falling back to the source tree), plus absolute, `~` and
  `package://sac_driver/...`.
- **`policy_loader.py` auto-detects the architecture** from weight shapes
  (`backbone.0.weight.shape[1]` → state_dim, `mean_layer.weight.shape[0]` → action_dim).
  Never hardcode dimensions.
- **Naming:** sensor readings are `sensor_accel` / `sensor_yaw`; `accel` is the NN output.
  Do not shadow.
- Comments and identifiers in English. Some legacy comments are Polish; leave them unless you
  are rewriting the surrounding block.

## Secrets

- **No passwords in the repository, ever** — not in scripts, configs, service files or docs.
- Privileged actions use `sudo -n` (non-interactive) against the allowlist installed at
  `/etc/sudoers.d/f1tenth` from `system/sudoers.d/f1tenth.in`: `shutdown`,
  `systemctl start|stop|restart key_drive.service`, and `chmod 666` on `/dev/vesc` and
  `/dev/rplidar`. Everything else must prompt the human.
- If you find a hardcoded password in a script, remove it and route the command through the
  sudoers allowlist — do not just move it to a variable or an environment file.
- Device permissions should come from the udev rules (`system/udev/99-f1tenth.rules`,
  `MODE:="0666"`), not from `chmod` at runtime; the sudoers `chmod` entry is a fallback.

## Porting to a new car — checklist

1. Flash JetPack 5.x, then `git clone ... ~/ros2_ws && cd ~/ros2_ws && ./install.sh` (add
   `--with-ros` if ROS2 Foxy is missing). Reboot.
2. Confirm `/dev/rplidar`, `/dev/vesc`, `/dev/input/js0`. Different hardware → update the
   vendor:product IDs in `system/udev/99-f1tenth.rules`; a different lidar also needs the
   serial port / baud / scan mode arguments in `bringup_launch3.py` and possibly
   `src/f1tenth_stack/config/sensors.yaml`.
3. Put the car on a stand. **Wheels off the ground for everything below.**
4. Upload the VESC configuration with `tools/vesc/vesc_config_upload.py --verify`. Check the
   firmware version first — the parameter definitions shipped here are for FW 6.02.
5. `python3 scripts/key_drive_test.py` — verify ERPM sign and servo direction.
6. Determine the speed sign. Set `control.speed_sign` and `SPEED_SIGN` in `key_drive.py`
   together; they must agree.
7. Cardboard test for the lidar: obstacle on one side, wheels must steer away. Adjust
   `lidar.angle_offset_deg` and `control.steer_sign`.
8. Calibrate `vesc.yaml`: `speed_to_erpm_gain`, `steering_angle_to_servo_gain`/`offset`,
   `servo_min`/`servo_max`, `vesc_to_odom_node.wheelbase`.
9. Adjust the static `base_link` → `laser` transform in `bringup_launch3.py` to the real lidar
   mounting position (currently `0.27 0 0.11`).
10. Record a map with SLAM if you plan to use localization / pure pursuit.
11. Only then drive on the ground, starting at `control.speed_limit_mps: 2.0`.
12. Update `.context/STATE.md` with anything that differs from this car.

## Further reading

- [DOCUMENTATION.md](DOCUMENTATION.md) — per-module reference: classes, callbacks, topics,
  every config key.
- [docs/SETUP_NEW_JETSON.md](docs/SETUP_NEW_JETSON.md) — fresh-machine setup, step by step.
- [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) — known failure modes and fixes.
- [.context/KNOWLEDGE.md](.context/KNOWLEDGE.md) — architecture and hard-won debugging lessons.
- [.context/STATE.md](.context/STATE.md) — what works right now, and what does not.
