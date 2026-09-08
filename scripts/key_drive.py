#!/usr/bin/env python3
"""Keyboard teleop for the F1TENTH car (arrow keys / WASD).

Publishes AckermannDriveStamped on /teleop_gated (ackermann_mux, priority 100),
the same channel joy_mode_manager uses for the gamepad. It bypasses the
joystick deadman and is NOT masked by /autonomy_lock.

Input backends (auto-selected, or forced with --evdev / --no-x):
  evdev    keyboard plugged straight into the Jetson (/dev/input/event*).
           Hot-plug safe: waits for a keyboard, re-attaches after unplug,
           sends STOP the moment the keyboard disappears. Used by the boot service.
  X11      key state from the X server (XQueryKeymap) - works over NoMachine.
  terminal fallback, latched ~0.5 s per key press.

Keys:
  Up / W      forward          Down / S    reverse
  Left / A    steer left       Right / D   steer right
  Space / Esc stop             + / -       speed +- 0.25 m/s
  q / Ctrl-C  quit (disabled with --no-quit-key, used by the boot service)
"""
import argparse
import ctypes
import os
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

RATE_HZ = 20.0
TERM_HOLD_S = 0.5  # terminal fallback: how long a key press stays "held"
# On this car a positive AckermannDrive.speed spins the motor backwards (motor/ESC wiring),
# so Up must send a NEGATIVE speed to drive forward.
SPEED_SIGN = -1.0


def log(msg):
    sys.stdout.write('\r%s %s\n' % (time.strftime('%H:%M:%S'), msg))
    sys.stdout.flush()


class XKeyboard:
    """Polls the X server for the pressed/released state of a few keys."""

    KEYS = {
        'fwd': ['Up', 'w', 'W'],
        'rev': ['Down', 's', 'S'],
        'left': ['Left', 'a', 'A'],
        'right': ['Right', 'd', 'D'],
        'stop': ['space', 'Escape'],
    }

    def __init__(self):
        self.x = ctypes.cdll.LoadLibrary('libX11.so.6')
        self.x.XOpenDisplay.restype = ctypes.c_void_p
        self.x.XOpenDisplay.argtypes = [ctypes.c_char_p]
        self.x.XStringToKeysym.restype = ctypes.c_ulong
        self.x.XStringToKeysym.argtypes = [ctypes.c_char_p]
        self.x.XKeysymToKeycode.restype = ctypes.c_ubyte
        self.x.XKeysymToKeycode.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        self.x.XQueryKeymap.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
        self.dpy = self.x.XOpenDisplay(None)
        if not self.dpy:
            raise RuntimeError('cannot open X display %r' % os.environ.get('DISPLAY'))
        self.codes = {}
        for action, names in self.KEYS.items():
            codes = []
            for n in names:
                kc = self.x.XKeysymToKeycode(self.dpy, self.x.XStringToKeysym(n.encode()))
                if kc:
                    codes.append(kc)
            self.codes[action] = codes
        self.buf = ctypes.create_string_buffer(32)
        self.quit = False

    def poll(self):
        pass

    def connected(self):
        return True

    def state(self):
        self.x.XQueryKeymap(self.dpy, self.buf)
        raw = self.buf.raw

        def down(kc):
            return bool(raw[kc >> 3] & (1 << (kc & 7)))

        return {a: any(down(kc) for kc in codes) for a, codes in self.codes.items()}


class TermKeyboard:
    """Fallback: reads arrow keys from the terminal, latches them for TERM_HOLD_S."""

    SEQ = {
        b'\x1b[A': 'fwd', b'\x1b[B': 'rev', b'\x1b[D': 'left', b'\x1b[C': 'right',
        b'w': 'fwd', b's': 'rev', b'a': 'left', b'd': 'right', b' ': 'stop',
    }

    def __init__(self):
        self.last = {}
        self.quit = False

    def feed(self, data):
        now = time.time()
        for seq, action in self.SEQ.items():
            if seq in data:
                self.last[action] = now
        if b' ' in data:
            self.last = {'stop': now}

    def poll(self):
        pass

    def connected(self):
        return True

    def state(self):
        now = time.time()
        return {a: (now - t) < TERM_HOLD_S for a, t in self.last.items()}


class EvdevKeyboard:
    """Reads a physical keyboard from /dev/input/event* (no X needed), hot-plug safe."""

    RESCAN_S = 1.0
    HOLD_TIMEOUT_S = 1.0  # a held key must keep sending repeat events, else it is released

    def __init__(self, grab=True, allow_quit=True):
        import evdev
        from evdev import ecodes as E
        self.evdev = evdev
        self.E = E
        self.need = {E.KEY_UP, E.KEY_DOWN, E.KEY_LEFT, E.KEY_RIGHT, E.KEY_SPACE}
        self.map = {
            E.KEY_UP: 'fwd', E.KEY_W: 'fwd',
            E.KEY_DOWN: 'rev', E.KEY_S: 'rev',
            E.KEY_LEFT: 'left', E.KEY_A: 'left',
            E.KEY_RIGHT: 'right', E.KEY_D: 'right',
            E.KEY_SPACE: 'stop', E.KEY_ESC: 'stop',
        }
        self.grab = grab
        self.allow_quit = allow_quit
        self.dev = None
        self.has_repeat = False
        self.down = {}
        self.last_scan = 0.0
        self.quit = False
        self.speed_delta = 0.0

    def _find(self):
        for path in sorted(self.evdev.list_devices()):
            try:
                d = self.evdev.InputDevice(path)
                keys = set(d.capabilities().get(self.E.EV_KEY, []))
            except OSError:
                continue
            if self.need <= keys:
                return d
            d.close()
        return None

    def _attach(self, dev):
        self.dev = dev
        self.has_repeat = self.E.EV_REP in dev.capabilities()
        os.set_blocking(dev.fd, False)
        if self.grab:
            try:
                dev.grab()
            except OSError as e:
                log('warning: cannot grab %s (%s)' % (dev.path, e))
        self.down.clear()
        log('keyboard connected: %s (%s)%s' % (dev.name, dev.path, '' if self.has_repeat else ' [no autorepeat]'))

    def _detach(self, why):
        if self.dev is not None:
            try:
                self.dev.close()
            except Exception:  # noqa: BLE001
                pass
        self.dev = None
        self.down.clear()
        log('keyboard disconnected (%s) - STOP, waiting for keyboard...' % why)

    def poll(self):
        now = time.time()
        if self.dev is None:
            if now - self.last_scan >= self.RESCAN_S:
                self.last_scan = now
                dev = self._find()
                if dev is not None:
                    self._attach(dev)
            return
        if not os.path.exists(self.dev.path):
            self._detach('device node gone')
            return
        try:
            while True:
                ev = self.dev.read_one()
                if ev is None:
                    break
                if ev.type != self.E.EV_KEY:
                    continue
                action = self.map.get(ev.code)
                if action is not None:
                    if ev.value in (1, 2):
                        self.down[action] = now
                    else:
                        self.down.pop(action, None)
                elif ev.value == 1:
                    if ev.code in (self.E.KEY_EQUAL, self.E.KEY_KPPLUS):
                        self.speed_delta += 0.25
                    elif ev.code in (self.E.KEY_MINUS, self.E.KEY_KPMINUS):
                        self.speed_delta -= 0.25
                    elif ev.code == self.E.KEY_Q and self.allow_quit:
                        self.quit = True
        except BlockingIOError:
            pass
        except OSError as e:
            self._detach(str(e))

    def connected(self):
        return self.dev is not None

    def state(self):
        if self.dev is None:
            return {}
        now = time.time()
        if self.has_repeat:
            return {a: (now - t) < self.HOLD_TIMEOUT_S for a, t in self.down.items()}
        return {a: True for a in self.down}


class KeyDrive(Node):
    def __init__(self, speed, steer, topic):
        super().__init__('key_drive')
        self.speed = speed
        self.steer = steer
        self.pub = self.create_publisher(AckermannDriveStamped, topic, 10)
        self.topic = topic

    def send(self, speed, steer):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steer)
        self.pub.publish(msg)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--speed', type=float, default=2.0, help='m/s for Up/Down (default 2.0)')
    ap.add_argument('--steer', type=float, default=0.3, help='rad for Left/Right (default 0.3)')
    ap.add_argument('--max-speed', type=float, default=4.0)
    ap.add_argument('--topic', default='/teleop_gated')
    ap.add_argument('--evdev', action='store_true', help='read a keyboard plugged into the Jetson (no X needed)')
    ap.add_argument('--no-grab', action='store_true', help='evdev: do not grab the keyboard exclusively')
    ap.add_argument('--no-quit-key', action='store_true', help='ignore q (boot service)')
    ap.add_argument('--no-x', action='store_true', help='force terminal mode')
    args = ap.parse_args()

    kb = None
    mode = ''
    if args.evdev:
        kb = EvdevKeyboard(grab=not args.no_grab, allow_quit=not args.no_quit_key)
        mode = 'evdev (keyboard plugged into the car)'
    elif not args.no_x and os.environ.get('DISPLAY'):
        try:
            kb = XKeyboard()
            mode = 'X11 (DISPLAY=%s)' % os.environ['DISPLAY']
        except Exception as e:  # noqa: BLE001
            print('X keyboard unavailable (%s), using terminal mode' % e)
    term = TermKeyboard()
    if kb is None:
        kb = term
        mode = 'terminal (latched %.1fs)' % TERM_HOLD_S

    rclpy.init()
    node = KeyDrive(args.speed, args.steer, args.topic)

    fd = sys.stdin.fileno() if sys.stdin and sys.stdin.isatty() else None
    old = None
    if fd is not None:
        old = termios.tcgetattr(fd)
        tty.setcbreak(fd)
    log('=== KEY DRIVE === input: %s  topic: %s  speed %.2f m/s  steer %.2f rad' % (mode, args.topic, node.speed, node.steer))
    log('  Up/Down = forward/reverse   Left/Right = steer   Space/Esc = stop   +/- = speed   q = quit')
    period = 1.0 / RATE_HZ
    last_print = ''
    was_connected = kb.connected()
    zero_until = 0.0  # keep sending zeros for a moment after a disconnect / key release
    ZERO_TAIL_S = 0.5  # publish zeros this long after the last active key, then go silent
    # Silent when idle so the gamepad (joy_teleop -> joy_mode_manager) can use the same
    # /teleop_gated topic; ackermann_mux simply takes whichever source is publishing.
    try:
        while rclpy.ok():
            t0 = time.time()
            if fd is not None and select.select([fd], [], [], 0)[0]:
                data = os.read(fd, 64)
                if (b'q' in data and not args.no_quit_key) or b'\x03' in data:
                    break
                if b'+' in data or b'=' in data:
                    node.speed = min(args.max_speed, node.speed + 0.25)
                if b'-' in data:
                    node.speed = max(0.25, node.speed - 0.25)
                term.feed(data)

            kb.poll()
            if kb.quit:
                break
            delta = getattr(kb, 'speed_delta', 0.0)
            if delta:
                node.speed = max(0.25, min(args.max_speed, node.speed + delta))
                kb.speed_delta = 0.0
                log('speed set to %.2f m/s' % node.speed)

            connected = kb.connected()
            if was_connected and not connected:
                zero_until = t0 + 1.0
            was_connected = connected

            speed = 0.0
            steer = 0.0
            if connected:
                st = kb.state()
                if not st.get('stop'):
                    if st.get('fwd'):
                        speed = SPEED_SIGN * node.speed
                    elif st.get('rev'):
                        speed = -SPEED_SIGN * node.speed
                    if st.get('left'):
                        steer = node.steer
                    elif st.get('right'):
                        steer = -node.steer
                active = st.get('stop') or speed != 0.0 or steer != 0.0
                if active:
                    zero_until = t0 + ZERO_TAIL_S
                    node.send(speed, steer)
                elif t0 < zero_until:
                    node.send(0.0, 0.0)
            elif t0 < zero_until:
                node.send(0.0, 0.0)
            # idle keyboard / no keyboard -> publish nothing; mux times out, car stays stopped

            if fd is not None:
                line = '\r  car speed %+5.2f m/s (cmd %+5.2f)  steer %+5.2f rad   [max %.2f]   ' % (
                    SPEED_SIGN * speed, speed, steer, node.speed) if connected else '\r  NO KEYBOARD - stopped, waiting...   '
                if line != last_print:
                    sys.stdout.write(line)
                    sys.stdout.flush()
                    last_print = line
            dt = time.time() - t0
            if dt < period:
                time.sleep(period - dt)
    finally:
        for _ in range(5):
            node.send(0.0, 0.0)
            time.sleep(0.05)
        if old is not None:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        log('key_drive: stopped, zero command sent.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
