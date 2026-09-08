# Troubleshooting

Known failure modes on this platform and how to get out of them. Most of these cost real
debugging time — check here before assuming a new bug.

## Quick triage

```bash
source /opt/ros/foxy/setup.bash && source ~/ros2_ws/install/setup.bash

ros2 node list                       # who is running
ros2 topic list                      # do /scan, /odom, /sensors/core exist?
ros2 topic hz /scan                  # ~8 Hz expected
ros2 topic hz /odom                  # ~50 Hz expected
ls -l /dev/rplidar /dev/vesc         # udev symlinks + permissions
systemctl is-active key_drive.service # is the boot service holding the VESC?
tail -n 100 ~/ros2_ws/log/key_drive_bringup.log
tail -n 100 ~/ros2_ws/log/key_drive_boot.log
./install.sh --check                 # environment health check
```

---

## Zombie ROS2 nodes after stopping a launch

**Symptom.** A relaunch fails silently, the lidar never publishes, or the VESC reports the port
is busy. `ros2 node list` still shows nodes you thought you killed.

**Cause.** `ros2 launch` spawns each node in its own process group. Killing the launch process
does not kill the children; they keep holding USB ports.

**Fix.**

```bash
pkill -f sllidar; pkill -f vesc; pkill -f joy; pkill -f ackermann_mux
pkill -f static_transform_publisher; pkill -f sac_driver_node
```

Full list used by the scripts and the panel: `sllidar_ros2_node`, `sllidar_node`,
`vesc_driver_node`, `ackermann_to_vesc_node`, `vesc_to_odom_node`, `joy_linux_node`,
`joy_mode_manager`, `joy_teleop`, `ackermann_mux`, `static_transform_publisher`,
`sac_driver_node`.

The control panel's `stop()` calls `_kill_ros2_orphans()` for this, and
`scripts/key_drive_boot.sh` does the same on startup — but it does not always catch every case.
Always verify with `ros2 node list` before relaunching.

## Two VESC drivers on one port

**Symptom.** Erratic or absent motion, garbled telemetry, commands that appear to be ignored,
`/sensors/core` stuttering.

**Cause.** `/dev/vesc` is a single serial port; two `vesc_driver_node` instances (usually a
manual bringup plus `key_drive.service`, or two bringups) interleave their packets.

**Fix.**

```bash
pgrep -af vesc_driver_node                  # must show at most one
systemctl is-active key_drive.service       # stop it if you are driving manually
sudo systemctl stop key_drive.service
```

`scripts/key_drive.sh` handles this automatically: it pauses the service for the manual session
and restarts it on exit. `scripts/key_drive_boot.sh` kills leftovers before starting.

## `/dev/vesc` or `/dev/rplidar` missing

**Symptom.** `ERROR: /dev/vesc not found`, or the lidar driver cannot open the port.

**Checks.**

```bash
lsusb | grep -Ei '10c4:ea60|0483:5740'    # CP210x lidar bridge / STM32 VCP
dmesg | tail -20                          # did the device enumerate?
ls -l /dev/ttyUSB* /dev/ttyACM*
```

**Causes and fixes.**

- Device not powered — the VESC needs battery power, not just USB.
- udev rules missing: `sudo cp system/udev/99-f1tenth.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger`.
- Different hardware revision → different USB IDs; update the rules file.
- USB enumerating late at boot. `key_drive_boot.sh` waits for `/dev/vesc` in a loop for exactly
  this reason.

## Permission denied on `/dev/vesc` or `/dev/rplidar`

**Symptom.** The driver starts and immediately dies with a permission error.

**Fix.** The udev rules set `MODE:="0666"`, so this normally cannot happen. If it does:

```bash
groups                       # dialout must be listed - relogin after install.sh
sudo -n chmod 666 /dev/vesc  # allowed without a password via /etc/sudoers.d/f1tenth
```

Never hardcode a password to work around this — see the secrets policy in
[AGENTS.md](../AGENTS.md).

## Lidar does not reconnect after a restart

**Symptom.** After stopping and restarting the bringup, `/scan` never appears, or the driver
loops on "cannot bind to the specified serial port".

**Cause.** A zombie `sllidar_node` still holds the port, or the CP210x bridge is in a bad state.

**Fix.**

```bash
pkill -f sllidar
sleep 2
ros2 launch f1tenth_stack bringup_launch3.py
```

If that does not help, unplug and replug the lidar USB (or power-cycle it) and check
`dmesg | tail`. `tools/lidar_diag.py` and `tools/lidar_test.py` talk to the device directly and
help decide whether the problem is ROS-side or hardware-side. This is a known open issue on
this car — the restart cycle is not fully reliable.

## `sac_driver` prints "waiting for data" forever

**Symptom.** Repeated `Waiting for data: scan=False odom=... servo=...`.

**Checks.** For each `False`, confirm the producing node is alive and the topic is publishing:

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic echo /commands/servo/position
```

### BEST_EFFORT QoS drops (the classic)

**Never use `qos_profile_sensor_data` (BEST_EFFORT) for subscriptions on this Jetson.** Even
though the lidar publishes RELIABLE, a BEST_EFFORT subscriber silently receives nothing here.
Symptom: `scan=False` while `/scan` clearly has publishers and `ros2 topic hz /scan` shows
messages. Every subscription in `sac_driver_node.py` uses plain QoS depth 10 (RELIABLE) — keep
it that way.

### Servo data blocks startup

`/commands/servo/position` is only published when the car is actually being driven. `_data_ready()`
requires servo data when the servo subscription exists, so a freshly started node can wait
forever until someone drives manually once. Workarounds: nudge the car with the gamepad or
`key_drive.py` for a moment, or publish one message by hand:

```bash
ros2 topic pub -1 /commands/servo/position std_msgs/msg/Float64 "{data: 0.5304}"
```

### `_publish_stop` hides transitions

`_publish_stop()` has a `_last_stop_sent` guard that suppresses repeated stop logs, which makes
disabled → enabled → data-missing transitions invisible. `_on_estop` logs "Autonomy
ENABLED/DISABLED" and `_on_timer` logs data readiness — read those instead of the stop message.

## Model / checkpoint loading errors

### `ModuleNotFoundError: No module named 'numpy._core'`

Checkpoints saved on the training PC with a newer NumPy reference `numpy._core`, which does not
exist in NumPy 1.24. `policy_loader.py` installs a `sys.modules` alias to handle this. If you
see the error anyway, you are loading through a code path that bypasses `policy_loader`.

### PyTorch 2.x zip format rejected by 1.13

PyTorch 2.x checkpoints carry `.format_version`, `.storage_alignment` and
`.data/serialization_id` entries that torch 1.13 refuses. Fix: repackage the zip without those
files, keeping every entry under a single top-level directory (e.g. `session_x/data.pkl`).
Best done on the training PC before copying the checkpoint over.

### `_orig_mod.` key prefix

Models saved from a `torch.compile()`d module carry an `_orig_mod.` prefix on every state-dict
key. `policy_loader.py` strips it; if you load weights yourself, do the same.

### "model.path is empty" / node does nothing

The node was started without its parameters. Use the launch file, or pass the params file
explicitly:

```bash
ros2 run sac_driver sac_driver_node --ros-args \
    --params-file ~/ros2_ws/src/sac_driver/config/driver_params.yaml
```

### Checkpoint not found with a relative path

`model.path` is relative to the package **share** directory
(`install/sac_driver/share/sac_driver/weights/...`), with the source tree as a fallback. Weights are
copied there by `setup.py` at build time — after adding a new `.pth`, rebuild:

```bash
colcon build --packages-select sac_driver
```

## Car steers into obstacles instead of away

The lidar frame convention does not match the model. Change `lidar.angle_offset_deg` (`0.0` for
the current models, `-90.0` for the old 27-ray one) and re-check `control.steer_sign` — a 90°
offset change flips the effective steering direction. Verify with the cardboard test described
in [SETUP_NEW_JETSON.md](SETUP_NEW_JETSON.md), wheels off the ground.

## Car drives backwards

`control.speed_sign` in `driver_params.yaml` and `SPEED_SIGN` in `scripts/key_drive.py` must
both match the car's wiring. On this car positive `AckermannDrive.speed` = reverse, so both are
`-1.0`. Verify with `scripts/key_drive_test.py` (checks the ERPM sign on `/sensors/core`).

## Safe mode does nothing

`control.safe_steer_scale` and `control.safe_accel_scale` are both `1.0`, so `safe_mode: true`
applies no scaling. For a cautious first run set them below 1.0, and/or lower
`control.safe_speed_limit_mps`.

## joy_teleop refuses to start

**Symptom.** `JoyTeleopException: No buttons or axes configured for command 'default'` and the
bringup aborts.

**Cause.** The patched `joy_teleop` in `local_python/` was not on `PYTHONPATH`. The shipped
`joy_teleop.yaml` defines a `default` command with no deadman buttons, which upstream rejects;
the patched copy treats such commands as always active.

**Fix.** `bringup_launch3.py` prepends `<ws>/local_python` to `PYTHONPATH` and falls back to
`~/ros2_ws/local_python`. It prints a warning when neither exists. Make sure the directory is
present in your clone and that you launch through `bringup_launch3.py`, not by running
`joy_teleop` directly.

## Gamepad buttons do nothing

```bash
ros2 topic echo /joy
```

- No messages → `joy_linux_node` is not running or `/dev/input/js0` does not exist. Check
  `ls /dev/input/js*` and that you are in the `input` group.
- Messages arrive but LB/RB are at the wrong indices → your pad has a different mapping. This
  setup expects LB = button 4 (manual) and RB = button 5 (autonomy). A Logitech F710 must be in
  **X-input** mode (the switch on the front).

## Bluetooth gamepad does not appear

`scripts/bt_pad_connect.sh` (run by `bt_pad.service` on this car) retries the connection every
4 s. Check:

```bash
systemctl status bt_pad.service
tail -f ~/ros2_ws/log/bt_pad.log
bluetoothctl info <PAD_MAC>       # "Connected: yes"?
ls /dev/input/js0                 # joydev creates this once the pad connects
```

The pad MAC is baked into the script and overridable with the `PAD_MAC` environment variable —
on a different car, pair your own pad first (`bluetoothctl` → `scan on` → `pair` → `trust`) and
set `PAD_MAC` accordingly.

## Keyboard drive: DISPLAY problems

**Symptom.** `cannot open X display ':1004'`, or keys are ignored.

- The X11 backend needs a reachable X server. Over NoMachine on this car it is `:1004`;
  `key_drive.sh` exports `DISPLAY=${DISPLAY:-:1004}`. Set `DISPLAY` to your own session, or:
- `--evdev` — read a keyboard plugged straight into the Jetson (`/dev/input/event*`), no X at
  all. This is what the boot service uses. Requires membership in the `input` group.
- `--no-x` — terminal fallback; keys latch for ~0.5 s per press, so it feels sluggish. Only for
  a quick check.

With `--evdev` the script grabs the keyboard exclusively; pass `--no-grab` if that interferes
with your desktop session.

## `key_drive.service` fights with manual work

The unit starts a bringup and keyboard teleop at boot, with no deadman. It also restarts itself
on any failure (`Restart=always`).

```bash
systemctl status key_drive.service
sudo systemctl stop key_drive.service       # for this session
sudo systemctl disable --now key_drive.service   # permanently
tail -f ~/ros2_ws/log/key_drive_boot.log
```

If the log shows a restart loop, the supervisor is detecting a dead node in the VESC chain
(`vesc_driver_node`, `ackermann_to_vesc_node`, `ackermann_mux`) or `/sensors/core` never
appeared within 40 s — usually the VESC is unpowered or another process owns the port.

## NFS share not mounted

**Symptom.** `/home/<user>/shared` is empty after a reboot.

```bash
mount | grep shared
sudo mount /home/<user>/shared
ping -c1 192.168.1.102
```

Not critical: inference uses the local weights in `src/sac_driver/weights/`. The share only
matters when exchanging files with the training PC.

## LEDs do not light

```bash
ls -l /dev/spidev0.0     # missing -> SPI1 not enabled in the device tree
groups | grep gpio
```

Enable SPI1 with `sudo /opt/nvidia/jetson-io/jetson-io.py` (Configure 40-pin header → enable
spi1 → save and reboot), confirm `spidev` is in `/etc/modules-load.d/`, and relogin so the
`gpio` group applies. The strip is on MOSI, pin 19.

## Control panel starts nothing

- Run it from a graphical session (it is GTK3): `python3 ros2_panel/panel_app.py`.
- Missing GTK bindings → `sudo apt install python3-gi gir1.2-gtk-3.0`.
- A card that starts and immediately turns red: read the debug console at the bottom, then run
  the same command in a terminal to see the real error.
- The SLAM card uses the **apt** `slam_toolbox` with
  `src/slam_toolbox/config/mapper_params_online_async.yaml`; the source tree in `src/slam_toolbox/`
  is not built.

## Build failures

```bash
colcon build --packages-select sac_driver --event-handlers console_direct+
```

- Missing dependency → `rosdep install --from-paths src --ignore-src -r -y`.
- `range_libc` / `particle_filter` failing → they are only built with `./install.sh --full`;
  they are not needed for SAC driving.
- Out of memory on a parallel build → `colcon build --parallel-workers 1`.
- Always `source install/setup.bash` again after a build.
