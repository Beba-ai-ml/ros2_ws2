#!/bin/bash
# systemd entry point (key_drive.service): bringup + keyboard drive, forever.
# Any failure -> exit non-zero -> systemd restarts us (Restart=always, no rate limit).
export HOME="${HOME:-$(getent passwd "$(id -u)" | cut -d: -f6)}"
unset DISPLAY
source /opt/ros/foxy/setup.bash
cd "$(dirname "$(readlink -f "$0")")/.." || exit 1
source install/setup.bash
mkdir -p log

echo "[boot] $(date '+%F %T') key_drive_boot starting"

# --- kill leftovers from a previous instance (never two vesc drivers on one port) ---
pkill -INT -f "ros2 launch f1tenth_stack bringup_launch3.py" 2>/dev/null
pkill -INT -f "scripts/key_drive.py" 2>/dev/null
sleep 1
for p in sllidar_node vesc_driver_node ackermann_to_vesc_node vesc_to_odom_node \
         joy_linux_node joy_mode_manager ackermann_mux static_transform_publisher joy_teleop; do
    pkill -KILL -f "$p" 2>/dev/null
done

# --- wait for the VESC to enumerate (USB may come up after multi-user.target) ---
n=0
until [ -e /dev/vesc ]; do
    [ $((n % 10)) -eq 0 ] && echo "[boot] waiting for /dev/vesc ..."
    n=$((n + 1)); sleep 1
done

BR=""; KD=""
cleanup() {
    echo "[boot] cleanup"
    [ -n "$KD" ] && kill -INT "$KD" 2>/dev/null
    [ -n "$BR" ] && kill -INT -- "-$BR" 2>/dev/null
    sleep 2
    [ -n "$BR" ] && kill -KILL -- "-$BR" 2>/dev/null
    for p in sllidar_node vesc_driver_node ackermann_to_vesc_node vesc_to_odom_node \
             joy_linux_node joy_mode_manager ackermann_mux static_transform_publisher joy_teleop; do
        pkill -KILL -f "$p" 2>/dev/null
    done
}
trap cleanup EXIT

setsid ros2 launch f1tenth_stack bringup_launch3.py &
BR=$!

# --- VESC telemetry must appear, otherwise restart everything ---
ok=0
for _ in $(seq 1 40); do
    if ros2 topic list 2>/dev/null | grep -q '^/sensors/core$'; then ok=1; break; fi
    kill -0 "$BR" 2>/dev/null || { echo "[boot] bringup died"; exit 1; }
    sleep 1
done
[ "$ok" = 1 ] || { echo "[boot] no /sensors/core after 40s - restarting"; exit 1; }
echo "[boot] VESC up, starting key_drive (evdev)"

python3 -u scripts/key_drive.py --evdev --no-quit-key "$@" &
KD=$!

# --- supervise: if either side dies, exit -> systemd restarts the pair ---
while true; do
    kill -0 "$BR" 2>/dev/null || { echo "[boot] bringup exited"; exit 1; }
    kill -0 "$KD" 2>/dev/null || { echo "[boot] key_drive exited"; exit 1; }
    # a single crashed node does not stop ros2 launch -> check the drive chain ourselves
    for p in vesc_driver_node ackermann_to_vesc_node ackermann_mux; do
        pgrep -f "$p" >/dev/null || { echo "[boot] $p died - restarting everything"; exit 1; }
    done
    sleep 2
done
