#!/bin/bash
# systemd entry point (bt_pad.service): keep trying to connect the Bluetooth gamepad forever.
# Once connected, joydev creates /dev/input/js0 and joy_linux_node (bringup) picks it up on its own.
# Driving with the pad: hold L1 (button 4) = deadman, left stick Y = throttle, right stick X = steering.
PAD_MAC="${PAD_MAC:-57:9D:F2:83:5C:51}"
PAD_NAME="Wireless Controller"
RETRY_S=4

echo "[bt_pad] $(date '+%F %T') starting, pad $PAD_MAC ($PAD_NAME)"
modprobe hid_sony 2>/dev/null

# wait for the adapter
until timeout 5 bluetoothctl show 2>/dev/null | grep -q "Controller"; do
    echo "[bt_pad] waiting for bluetooth adapter..."; sleep 3
done
timeout 5 bluetoothctl power on >/dev/null 2>&1

was_connected=""
while true; do
    if timeout 5 bluetoothctl info "$PAD_MAC" 2>/dev/null | grep -q "Connected: yes"; then
        if [ -z "$was_connected" ]; then
            echo "[bt_pad] $(date '+%T') PAD CONNECTED"
            was_connected=1
        fi
    else
        if [ -n "$was_connected" ]; then
            echo "[bt_pad] $(date '+%T') pad disconnected - reconnecting..."
            was_connected=""
        fi
        timeout 5 bluetoothctl power on >/dev/null 2>&1
        timeout 12 bluetoothctl connect "$PAD_MAC" >/dev/null 2>&1
    fi
    sleep "$RETRY_S"
done
