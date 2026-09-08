# Setting up a new Jetson

Step-by-step bring-up of this project on a fresh board, from an unflashed Jetson to a driving
car. Reference machine: NVIDIA Jetson Orin Nano Super Developer Kit, JetPack 5.1.5
(L4T R35.6.1), Ubuntu 20.04.6, Python 3.8.10, 25 W power mode.

Steps 1-6 are required. Steps 7-13 are per-car calibration and optional extras.

---

## 1. Flash JetPack 5.x

Use NVIDIA SDK Manager (or the Orin Nano SD-card image) to flash **JetPack 5.x**, which gives
you Ubuntu 20.04 — the only Ubuntu release ROS2 Foxy targets. JetPack 6.x ships Ubuntu 22.04
and will not work with the packages in this repo without a full port to Humble.

After first boot:

```bash
sudo apt update && sudo apt upgrade -y
sudo nvpmodel -q            # check the power mode; 25W (mode 1) on this car
sudo jetson_clocks          # optional: lock clocks to max
cat /etc/nv_tegra_release   # confirm R35.x
```

Verify the basics:

```bash
lsb_release -a              # Ubuntu 20.04
python3 --version           # 3.8.x
```

## 2. ROS2 Foxy

Either let the installer do it:

```bash
git clone https://github.com/Beba-ai-ml/ros2_ws2.git ~/ros2_ws
cd ~/ros2_ws
./install.sh --with-ros
```

or install it manually first:

```bash
sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-foxy-desktop python3-colcon-common-extensions python3-rosdep
sudo rosdep init && rosdep update
```

## 3. Clone and install

```bash
git clone https://github.com/Beba-ai-ml/ros2_ws2.git ~/ros2_ws && cd ~/ros2_ws && ./install.sh
```

`install.sh` performs, in order:

1. Environment checks (Ubuntu/L4T version, Python, ROS2 presence, free disk).
2. apt packages: `ros-foxy-vesc*`, `ros-foxy-ackermann-msgs`, `ros-foxy-joy*`,
   `ros-foxy-teleop-tools`, `ros-foxy-diagnostic-updater`, `ros-foxy-slam-toolbox`,
   `ros-foxy-nav2-map-server`, `ros-foxy-nav2-lifecycle-manager`, `ros-foxy-rviz2`,
   `ros-foxy-tf2-ros`, `ros-foxy-tf2-geometry-msgs`, `ros-foxy-tf2-tools`,
   `ros-foxy-tf-transformations`, `ros-foxy-rplidar-ros`, `ros-foxy-robot-state-publisher`,
   `python3-colcon-common-extensions`, `python3-rosdep`, `python3-pip`, `python3-gi`,
   `gir1.2-gtk-3.0`, `python3-serial`, `libx11-6`, `git`, `build-essential`, `cmake`,
   `libeigen3-dev`.
3. `rosdep install` over `src/`, skipping keys resolved by pip or deliberately unwanted
   (`torch`, `numpy`, `slam_toolbox`, `range_libc`, ...). `slam_toolbox` and `range_libc` are
   never built from source by colcon.
4. `pip3 install --user -r requirements.txt` — torch 1.13.1 (CPU), numpy 1.24.4, pyserial,
   evdev, Jetson.GPIO, rpi-ws281x, Adafruit Blinka + neopixel-spi.
5. udev rules from `system/udev/99-f1tenth.rules` → `/etc/udev/rules.d/`.
6. `spidev` in `/etc/modules-load.d/` (for the LED strip).
7. Adds the user to `dialout`, `gpio`, `input`, `plugdev`, `video`.
8. Installs `/etc/sudoers.d/f1tenth` (NOPASSWD for `shutdown`, `systemctl start|stop|restart
   key_drive.service`, `chmod 666 /dev/vesc|/dev/rplidar`) after validating it with
   `visudo -c -f`.
9. Creates a `~/ros2_ws` symlink if the repo was cloned somewhere else.
10. Adds the ROS + workspace `source` lines to `~/.bashrc`.
11. `colcon build --packages-select sac_driver f1tenth_stack ackermann_mux sllidar_ros2`.

Useful flags:

| Flag | Effect |
|------|--------|
| `--check` | Run the checks only; change nothing. Also useful later as a health check. |
| `--with-ros` | Install ROS2 Foxy from apt if it is missing |
| `--full` | Also build `particle_filter`, `pure_pursuit`, `stanley_avoidance`, `waypoint_generator`, `gap_follow`, `wall_follow`, `safety_node`, `scan_matching` and the `range_libc` Python wrapper |
| `--no-build` | Skip the colcon build |
| `--gpio-shutdown` | Install **and enable** `gpio-shutdown.service` |
| `--key-drive-service` | Install **and enable** `key_drive.service` (see the warning in step 12) |
| `--bt-pad` | Install **and enable** `bt_pad.service` (Bluetooth gamepad auto-connect; set `PAD_MAC` in `scripts/bt_pad_connect.sh`) |
| `--desktop` | Write a ROS2 Control Panel launcher to `~/Desktop` |
| `--yes` | Non-interactive; assume yes |

## 4. Reboot

Group membership only takes effect after a new login session:

```bash
sudo reboot
```

## 5. Check the devices

```bash
ls -l /dev/rplidar /dev/vesc     # udev symlinks
lsusb | grep -Ei '10c4:ea60|0483:5740|046d:c219'
ls /dev/input/js0                # joystick
groups                           # dialout gpio input plugdev video present?
```

If a symlink is missing, find the real IDs with `lsusb` and adjust
`system/udev/99-f1tenth.rules`, then:

```bash
sudo cp system/udev/99-f1tenth.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Expected IDs on this car:

| Device | USB ID | Symlink |
|--------|--------|---------|
| RPLiDAR S1 (CP210x) | `10c4:ea60` | `/dev/rplidar` |
| VESC 6 (STM32 VCP) | `0483:5740` | `/dev/vesc` |
| Logitech F710 | `046d:c219` | `/dev/input/js0` (via the joystick subsystem) |

## 6. Joystick check

```bash
source /opt/ros/foxy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run joy_linux joy_linux_node --ros-args -p dev:=/dev/input/js0 &
ros2 topic echo /joy
```

Press LB (button index 4) and RB (index 5) and confirm they toggle in the `buttons` array. If
your gamepad maps them elsewhere, update `joy_mode_manager.py` and the `deadman_buttons` list
in `src/f1tenth_stack/config/joy_teleop.yaml`.

The F710 has a switch on the front: it must be in **X-input (X)** mode for this button layout.

---

## 7. VESC configuration

**Wheels off the ground from here on.**

Check what the VESC is running:

```bash
python3 tools/vesc/vesc_config_upload.py --check-sig
```

The uploader carries parameter definitions for **firmware 6.02** in `tools/vesc/params/6.02/`.
If your VESC runs a different firmware, either flash 6.02 or add the matching parameter XMLs
from the VESC Tool source.

Preview, then upload:

```bash
python3 tools/vesc/vesc_config_upload.py \
    --motor tools/vesc/configs/motor_config.xml \
    --app   tools/vesc/configs/app_config.xml --dry-run

python3 tools/vesc/vesc_config_upload.py \
    --motor tools/vesc/configs/motor_config.xml \
    --app   tools/vesc/configs/app_config.xml --verify
```

`--verify` reads the configuration back and compares every value. The configs shipped here are
for this car's motor (FOC, current limits ±52.78 A, abs max 79.17 A, battery cut 10.2 → 9.0 V)
and app (controller id 97, app 3 = PPM + UART, servo output enabled, permanent UART enabled).
**If your motor differs, do not upload the motor config** — run the VESC motor detection
routine instead and export your own XML.

## 8. Calibrate `src/f1tenth_stack/config/vesc.yaml`

| Key | Value on this car | How to find yours |
|-----|-------------------|-------------------|
| `speed_to_erpm_gain` | `1850.0` | Command a known speed, measure the real distance over time, scale the gain until `/odom` matches reality |
| `speed_to_erpm_offset` | `0.0` | Leave at 0 unless the motor has a deadband |
| `speed_min` / `speed_max` | `-45250.0` / `45250.0` | erpm clamp — keep at or below the motor's safe erpm |
| `steering_angle_to_servo_gain` | `-0.9` | servo units per radian; the sign follows the steering linkage |
| `steering_angle_to_servo_offset` | `0.5304` | servo value with the wheels straight |
| `servo_min` / `servo_max` | `0.05` / `0.95` | mechanical end stops — set these before touching the gain |
| `vesc_to_odom_node.wheelbase` | `0.35` | measure front axle to rear axle, in metres |

Find the servo offset by publishing servo positions directly and watching the wheels:

```bash
ros2 topic pub -1 /commands/servo/position std_msgs/msg/Float64 "{data: 0.53}"
```

Then rebuild and restart the bringup (`colcon build --packages-select f1tenth_stack`) —
configs are installed into the package share directory, so editing the source YAML alone is
not enough.

## 9. Sign checks

Start the bringup, then run the automated test with the car still on a stand:

```bash
ros2 launch f1tenth_stack bringup_launch3.py     # terminal 1
python3 scripts/key_drive_test.py                # terminal 2
```

It drives forward, reverse, left and right in turn, and reports the motor ERPM from
`/sensors/core` and the servo command from `/commands/servo/position`.

**Speed sign.** On this car a positive `AckermannDrive.speed` spins the motor *backwards*, so
two places carry `-1.0`:

- `control.speed_sign` in `src/sac_driver/config/driver_params.yaml`
- `SPEED_SIGN` in `scripts/key_drive.py`

They must always agree. Flip both to `+1.0` if your car drives forward on a positive speed.

**Steering sign and lidar offset.** Run the SAC driver, enable it, and hold a large piece of
cardboard close to one side of the lidar:

```bash
ros2 launch sac_driver sac_driver.launch.py
ros2 service call /sac_driver/enable std_srvs/srv/SetBool "{data: true}"
```

The wheels must steer **away** from the cardboard. If they steer into it, the lidar frame
convention is wrong. `lidar.angle_offset_deg` is `0.0` for the current models (0° = forward);
the older 27-ray model used `-90.0` (90° = forward). Changing the offset by 90° also flips the
effective steering direction, so `control.steer_sign` usually has to change with it — on this
car offset `0.0` pairs with `steer_sign: 1.0`.

Check what the lidar actually sees with:

```bash
python3 ros2_panel/scan_test.py     # prints the 20 closest points of one /scan message
```

## 10. Lidar mounting

The bringup publishes a static transform `base_link` → `laser` at `0.27 0.0 0.11` (x y z, no
rotation) in `src/f1tenth_stack/launch/bringup_launch3.py`. Measure your own mounting position
and update it — SLAM and localization depend on it.

Lidar launch arguments (also in `bringup_launch3.py`, overridable on the command line):

```bash
ros2 launch f1tenth_stack bringup_launch3.py serial_port:=/dev/rplidar serial_baudrate:=256000 scan_mode:=Standard
```

| Argument | Default | Note |
|----------|---------|------|
| `serial_port` | `/dev/ttyUSB0` | use `/dev/rplidar` (the udev symlink) for a stable name |
| `serial_baudrate` | `256000` | S1; the A2/A3 use 115200 / 256000 respectively |
| `scan_mode` | `Standard` | try `Express`/`Boost` if the driver complains |
| `frame_id` | `laser` | must match the static transform |
| `angle_compensate` | `true` | keep enabled |

A different lidar model means a different driver package (or different arguments) — replace
the `sllidar_node` entry in the launch file. `src/f1tenth_stack/config/sensors.yaml` holds the
old Hokuyo/`urg_node` settings and is not used by the current bringup; adapt it if you switch
to a Hokuyo.

## 11. LED strip (optional)

Enable SPI1 on the 40-pin header — this rewrites the device tree and needs a reboot:

```bash
sudo /opt/nvidia/jetson-io/jetson-io.py
# Configure the 40-pin expansion header -> Configure header pins manually -> enable spi1
# -> Save pin changes -> Save and reboot
```

After the reboot:

```bash
ls -l /dev/spidev0.0
groups | grep gpio          # install.sh added you; needs a relogin
python3 scripts/ledy.py     # 7 LEDs light violet
```

The strip is WS2812B on SPI MOSI (pin 19), GRB order, driven through
`adafruit-circuitpython-neopixel-spi`.

## 12. Optional services

### Keyboard drive on boot

```bash
./install.sh --key-drive-service
```

> **SAFETY WARNING.** With this unit enabled the car starts a bringup at boot and responds to
> a keyboard plugged into the Jetson **immediately**, with no deadman button. Only enable it
> if that is what you want, and keep the wheels off the ground while testing.

```bash
systemctl status key_drive.service
sudo journalctl -u key_drive.service -f
tail -f ~/ros2_ws/log/key_drive_boot.log
sudo systemctl disable --now key_drive.service    # turn it off
```

`scripts/key_drive.sh` stops the service for the duration of a manual session and restarts it
on exit, because only one process may own `/dev/vesc`.

### GPIO shutdown button

```bash
./install.sh --gpio-shutdown
```

Wire a button (or jumper) between BOARD pins 37 and 38. Pin 37 is driven high; when pin 38
reads high for 0.2 s, the Jetson powers off. Adjust with
`--drive-pin` / `--sense-pin` / `--hold-seconds` in the unit file.

### Desktop launcher

```bash
./install.sh --desktop     # writes ~/Desktop/ROS2-Panel.desktop
```

Your desktop environment may ask you to "Allow Launching" the first time you use it.

## 13. Remote access and the training PC (optional)

**NoMachine.** This car runs `nxserver` for remote desktop, which is why the keyboard-drive
X11 backend defaults to `DISPLAY=:1004`. Any X display works — export `DISPLAY` before running
`scripts/key_drive.sh`, or use `--evdev` / `--no-x` to avoid X entirely.

**NFS share to the training PC.** Purely for exchanging weights and logs; inference does not
depend on it (checkpoints live in `src/sac_driver/weights/` and are tracked in git).

```bash
sudo apt install -y nfs-common
sudo mkdir -p /home/<user>/shared
echo '192.168.1.102:/home/beba/shared  /home/<user>/shared  nfs  defaults,noauto,x-systemd.automount  0  0' \
    | sudo tee -a /etc/fstab
sudo mount /home/<user>/shared
```

Training repository: <https://github.com/Beba-ai-ml/occupancy-racer-sac2>.

---

## 14. First drive

```bash
source /opt/ros/foxy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch f1tenth_stack bringup_launch3.py     # terminal 1
ros2 launch sac_driver sac_driver.launch.py      # terminal 2
```

Hold RB (with LB released) to hand control to the AI; release RB to stop. Start on a stand,
then move to a clear track with `control.speed_limit_mps` at its default `2.0`.

Anything not behaving? See [TROUBLESHOOTING.md](TROUBLESHOOTING.md).
