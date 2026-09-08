#!/bin/bash
# One-shot: start bringup (VESC + lidar, like the ROS2 Control Panel "Bringup" button)
# and drive the car with the keyboard. Press q to quit -> bringup is stopped too.
#
#   ~/ros2_ws/scripts/key_drive.sh              # default 1.0 m/s, 0.3 rad
#   ~/ros2_ws/scripts/key_drive.sh --speed 0.5  # any key_drive.py option is passed through
#   KEEP_BRINGUP=1 ~/ros2_ws/scripts/key_drive.sh   # leave bringup running after quit
# (no "set -u": ROS setup.bash references unset vars)
export DISPLAY="${DISPLAY:-:1004}"
source /opt/ros/foxy/setup.bash
cd ~/ros2_ws
source install/setup.bash
mkdir -p log
BRINGUP_LOG=~/ros2_ws/log/key_drive_bringup.log

# --- device permissions (same as panel SETUP step) ---
for dev in /dev/vesc /dev/rplidar; do
    if [ -e "$dev" ] && [ ! -w "$dev" ]; then
        sudo -n chmod 666 "$dev"
    fi
done
[ -e /dev/vesc ] || { echo "ERROR: /dev/vesc not found - is the VESC plugged in / powered?"; exit 1; }

BRINGUP_PID=""
cleanup() {
    if [ -n "$BRINGUP_PID" ] && [ -z "${KEEP_BRINGUP}" ]; then
        echo "stopping bringup (pid $BRINGUP_PID)..."
        kill -INT -- "-$BRINGUP_PID" 2>/dev/null
        pkill -INT -f "ros2 launch f1tenth_stack bringup_launch3.py" 2>/dev/null
        for _ in $(seq 1 30); do pgrep -f "ros2 launch f1tenth_stack bringup_launch3.py" >/dev/null || break; sleep 0.2; done
        kill -KILL -- "-$BRINGUP_PID" 2>/dev/null
        # same orphan cleanup as the panel
        for p in sllidar_ros2_node sllidar_node vesc_driver_node ackermann_to_vesc_node \
                 vesc_to_odom_node joy_linux_node joy_mode_manager ackermann_mux static_transform_publisher; do
            pkill -f "$p" 2>/dev/null
        done
        echo "bringup stopped."
    fi
    if [ -n "$SERVICE_PAUSED" ]; then
        echo "resuming key_drive.service..."
        sudo -n systemctl start key_drive.service 2>/dev/null
    fi
}
trap cleanup EXIT

# --- pause the boot service (it owns the VESC) while driving manually; resume on exit ---
SERVICE_PAUSED=""
if systemctl is-active --quiet key_drive.service 2>/dev/null; then
    echo "key_drive.service is running - pausing it for the manual session..."
    sudo -n systemctl stop key_drive.service 2>/dev/null
    SERVICE_PAUSED=1
    sleep 2
fi

# --- bringup (reuse if already running, e.g. started from the panel) ---
if ros2 node list 2>/dev/null | grep -q vesc_driver_node; then
    echo "bringup already running - reusing it."
else
    echo "building f1tenth_stack..."
    colcon build --packages-select f1tenth_stack > "$BRINGUP_LOG" 2>&1 || { echo "colcon build failed, see $BRINGUP_LOG"; exit 1; }
    . install/setup.bash
    echo "starting bringup (log: $BRINGUP_LOG)..."
    setsid ros2 launch f1tenth_stack bringup_launch3.py >> "$BRINGUP_LOG" 2>&1 &
    BRINGUP_PID=$!
fi

# --- wait for VESC telemetry ---
echo -n "waiting for VESC"
ok=0
for _ in $(seq 1 40); do
    if ros2 topic list 2>/dev/null | grep -q '^/sensors/core$'; then ok=1; break; fi
    if [ -n "$BRINGUP_PID" ] && ! kill -0 "$BRINGUP_PID" 2>/dev/null; then echo; echo "bringup died, see $BRINGUP_LOG"; exit 1; fi
    echo -n "."; sleep 1
done
echo
[ "$ok" = 1 ] || { echo "VESC topics never appeared, see $BRINGUP_LOG"; exit 1; }
sleep 1

python3 ~/ros2_ws/scripts/key_drive.py "$@"
