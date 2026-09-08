#!/usr/bin/env python3
"""
Monitor GPIO and power off Jetson when a trigger pin goes high.

Default: drive BOARD 37 high, watch BOARD 38 and shut down if it stays high
for the configured time. Suitable when 37/38 are wired together by przycisk
lub zworka (38 normally low thanks to pull-down). Requires Jetson.GPIO
(JetPack) and root privileges.
Usage: sudo python3 gpio_shutdown.py [--drive-pin 37] [--sense-pin 38] [--hold-seconds 0.2]
"""

import argparse
import subprocess
import sys
import time

import Jetson.GPIO as GPIO


def shutdown():
    """Request system power-off."""
    try:
        subprocess.run(["/sbin/poweroff"], check=True)
    except Exception as exc:  # pragma: no cover - defensive log
        print(f"Failed to call poweroff: {exc}", file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(description="Shutdown Jetson when pin goes high.")
    parser.add_argument(
        "--drive-pin",
        type=int,
        default=37,
        help="Board pin number to drive HIGH (default: 37).",
    )
    parser.add_argument(
        "--sense-pin",
        type=int,
        default=38,
        help="Board pin number to monitor for HIGH (default: 38).",
    )
    parser.add_argument(
        "--hold-seconds",
        type=float,
        default=0.2,
        help="How long the sense pin must stay HIGH before shutting down.",
    )
    parser.add_argument(
        "--poll-interval",
        type=float,
        default=0.05,
        help="Seconds between input polls.",
    )
    args = parser.parse_args()

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(args.drive_pin, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(args.sense_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    pressed_at = None
    try:
        while True:
            is_pressed = GPIO.input(args.sense_pin) == GPIO.HIGH
            now = time.monotonic()
            if is_pressed:
                pressed_at = pressed_at or now
                if now - pressed_at >= args.hold_seconds:
                    print(
                        f"Pin {args.sense_pin} high for {args.hold_seconds}s; shutting down."
                    )
                    shutdown()
                    break
            else:
                pressed_at = None
            time.sleep(args.poll_interval)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup([args.drive_pin, args.sense_pin])


if __name__ == "__main__":
    main()
