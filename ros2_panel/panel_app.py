#!/usr/bin/env python3
"""ROS2 Control Panel - Native GTK3 Application."""

import gi
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk, Gdk, GLib, Pango
import threading
import subprocess
import time
import os
import sys
import re as _re

# Add our directory to path so we can import process_manager
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from process_manager import ProcessManager, PROCESS_DEFINITIONS, ROS2_PREFIX


class BootOverlay(Gtk.Window):
    """Boot animation window - shows glowing title then fades to main app."""

    def __init__(self, on_done):
        super().__init__(title="ROS2 Control Panel")
        self.on_done = on_done
        self.set_default_size(800, 600)
        self.set_position(Gtk.WindowPosition.CENTER)
        self.set_decorated(False)

        # Black background
        self.override_background_color(Gtk.StateFlags.NORMAL, Gdk.RGBA(0, 0, 0, 1))

        self.label = Gtk.Label(label="ROS2 Control Panel")
        self.label.override_color(Gtk.StateFlags.NORMAL, Gdk.RGBA(0, 0.83, 1, 0))
        font_desc = Pango.FontDescription("Sans Bold 36")
        self.label.modify_font(font_desc)
        self.add(self.label)

        self.alpha = 0.0
        self.phase = "fade_in"  # fade_in -> hold -> fade_out
        self.phase_timer = 0
        GLib.timeout_add(30, self._animate)

    def _animate(self):
        if self.phase == "fade_in":
            self.alpha += 0.02
            if self.alpha >= 1.0:
                self.alpha = 1.0
                self.phase = "hold"
                self.phase_timer = 0
            self.label.override_color(
                Gtk.StateFlags.NORMAL, Gdk.RGBA(0, 0.83, 1, self.alpha)
            )
            return True
        elif self.phase == "hold":
            self.phase_timer += 30
            if self.phase_timer >= 1500:
                self.phase = "fade_out"
            return True
        elif self.phase == "fade_out":
            self.alpha -= 0.04
            if self.alpha <= 0:
                self.destroy()
                self.on_done()
                return False
            self.label.override_color(
                Gtk.StateFlags.NORMAL, Gdk.RGBA(0, 0.83, 1, self.alpha)
            )
            self.override_background_color(
                Gtk.StateFlags.NORMAL, Gdk.RGBA(0, 0, 0, self.alpha)
            )
            return True
        return False


class LedIndicator(Gtk.DrawingArea):
    """Custom LED indicator widget with glow effect."""

    STATE_COLORS = {
        "stopped": (1.0, 0.27, 0.27),    # #ff4444
        "starting": (1.0, 0.87, 0.27),   # #ffdd44
        "running": (0.27, 1.0, 0.27),    # #44ff44
        "disabled": (0.33, 0.33, 0.33),  # #555555
    }

    def __init__(self):
        super().__init__()
        self.state = "stopped"
        self.set_size_request(20, 20)
        self._pulse_alpha = 0.4
        self._pulse_dir = 1
        GLib.timeout_add(50, self._pulse_tick)

    def set_state(self, state):
        if state != self.state:
            self.state = state
            self.queue_draw()

    def _pulse_tick(self):
        if self.state in ("running", "starting"):
            speed = 0.03 if self.state == "running" else 0.06
            self._pulse_alpha += speed * self._pulse_dir
            if self._pulse_alpha >= 0.7:
                self._pulse_alpha = 0.7
                self._pulse_dir = -1
            elif self._pulse_alpha <= 0.2:
                self._pulse_alpha = 0.2
                self._pulse_dir = 1
            self.queue_draw()
        return True

    def do_draw(self, cr):
        w = self.get_allocated_width()
        h = self.get_allocated_height()
        cx, cy = w / 2, h / 2
        r = min(w, h) / 2 - 2

        color = self.STATE_COLORS.get(self.state, (1.0, 0.27, 0.27))

        # Outer glow
        if self.state != "disabled":
            import cairo
            glow = cairo.RadialGradient(cx, cy, r * 0.5, cx, cy, r * 2)
            glow.add_color_stop_rgba(0, *color, self._pulse_alpha)
            glow.add_color_stop_rgba(1, *color, 0)
            cr.set_source(glow)
            cr.arc(cx, cy, r * 2, 0, 3.14159 * 2)
            cr.fill()

        # Main dot
        cr.set_source_rgb(*color)
        cr.arc(cx, cy, r, 0, 3.14159 * 2)
        cr.fill()

        # Highlight
        import cairo
        highlight = cairo.RadialGradient(cx - r * 0.3, cy - r * 0.3, 0, cx, cy, r)
        highlight.add_color_stop_rgba(0, 1, 1, 1, 0.4)
        highlight.add_color_stop_rgba(1, 1, 1, 1, 0)
        cr.set_source(highlight)
        cr.arc(cx, cy, r, 0, 3.14159 * 2)
        cr.fill()


class ToggleSwitch(Gtk.DrawingArea):
    """Custom toggle switch widget matching the web UI style."""

    def __init__(self, callback=None):
        super().__init__()
        self.active = False
        self.callback = callback
        self.sensitive_flag = True
        self.set_size_request(56, 28)
        self.add_events(Gdk.EventMask.BUTTON_PRESS_MASK)
        self.connect("button-press-event", self._on_click)
        self._anim_pos = 0.0
        GLib.timeout_add(16, self._animate)

    def set_active(self, active):
        if active != self.active:
            self.active = active
            self.queue_draw()

    def set_enabled(self, enabled):
        self.sensitive_flag = enabled
        self.queue_draw()

    def _on_click(self, widget, event):
        if not self.sensitive_flag:
            return
        self.active = not self.active
        self.queue_draw()
        if self.callback:
            self.callback(self.active)

    def _animate(self):
        target = 1.0 if self.active else 0.0
        if abs(self._anim_pos - target) > 0.01:
            self._anim_pos += (target - self._anim_pos) * 0.3
            self.queue_draw()
        else:
            self._anim_pos = target
        return True

    def do_draw(self, cr):
        w = self.get_allocated_width()
        h = self.get_allocated_height()
        radius = h / 2
        alpha = 0.4 if not self.sensitive_flag else 1.0

        # Track background
        if self.active:
            cr.set_source_rgba(0.06, 0.2, 0.38, alpha)  # #0f3460
        else:
            cr.set_source_rgba(0.2, 0.2, 0.2, alpha)    # #333

        # Rounded rectangle
        cr.arc(radius, radius, radius - 1, 3.14159 * 0.5, 3.14159 * 1.5)
        cr.arc(w - radius, radius, radius - 1, -3.14159 * 0.5, 3.14159 * 0.5)
        cr.close_path()
        cr.fill()

        # Border
        if self.active:
            cr.set_source_rgba(0, 0.83, 1, alpha)  # #00d4ff
        else:
            cr.set_source_rgba(0.33, 0.33, 0.33, alpha)  # #555
        cr.arc(radius, radius, radius - 1, 3.14159 * 0.5, 3.14159 * 1.5)
        cr.arc(w - radius, radius, radius - 1, -3.14159 * 0.5, 3.14159 * 0.5)
        cr.close_path()
        cr.set_line_width(2)
        cr.stroke()

        # Knob
        knob_x = 4 + radius - 2 + self._anim_pos * (w - 2 * radius - 4)
        if self.active:
            cr.set_source_rgba(0, 0.83, 1, alpha)  # #00d4ff
        else:
            cr.set_source_rgba(0.67, 0.67, 0.67, alpha)  # #aaa
        cr.arc(knob_x, h / 2, radius - 4, 0, 3.14159 * 2)
        cr.fill()


class BatteryBar(Gtk.Box):
    """Battery level bar reading voltage from VESC. 12V=100%, 9V=0%."""

    V_MAX = 12.0
    V_MIN = 9.0

    def __init__(self):
        super().__init__(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
        self.get_style_context().add_class("battery-bar")
        self.set_margin_start(0)
        self.set_margin_end(0)

        self._voltage = 0.0
        self._percent = 0

        # Icon
        icon_label = Gtk.Label(label="BAT")
        icon_label.get_style_context().add_class("battery-icon")
        self.pack_start(icon_label, False, False, 0)

        # Progress bar
        self.progress = Gtk.ProgressBar()
        self.progress.set_hexpand(True)
        self.progress.set_valign(Gtk.Align.CENTER)
        self.progress.get_style_context().add_class("battery-progress")
        self.pack_start(self.progress, True, True, 0)

        # Voltage + percent label
        self.label = Gtk.Label(label="--.-V  --%")
        self.label.get_style_context().add_class("battery-label")
        self.pack_start(self.label, False, False, 0)

        # Start voltage reader thread
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self):
        """Read voltage from VESC /sensors/core topic every 2 seconds."""
        cmd = ROS2_PREFIX + "timeout 3 ros2 topic echo /sensors/core 2>/dev/null | head -30"
        while self._running:
            try:
                result = subprocess.run(
                    cmd, shell=True, executable="/bin/bash",
                    capture_output=True, text=True, timeout=5,
                )
                match = _re.search(r'voltage_input:\s*([0-9.]+)', result.stdout)
                if match:
                    v = float(match.group(1))
                    pct = max(0, min(100, int((v - self.V_MIN) / (self.V_MAX - self.V_MIN) * 100)))
                    self._voltage = v
                    self._percent = pct
                    GLib.idle_add(self._update_ui, v, pct)
            except Exception:
                pass
            time.sleep(2)

    def _update_ui(self, voltage, percent):
        self.progress.set_fraction(percent / 100.0)
        self.label.set_text(f"{voltage:.1f}V  {percent}%")

        # Color: green > 50%, yellow 20-50%, red < 20%
        ctx = self.progress.get_style_context()
        for cls in ("bat-ok", "bat-warn", "bat-crit"):
            ctx.remove_class(cls)
        if percent > 50:
            ctx.add_class("bat-ok")
        elif percent > 20:
            ctx.add_class("bat-warn")
        else:
            ctx.add_class("bat-crit")

    def stop(self):
        self._running = False


class ProcessCard(Gtk.Box):
    """A single process card with LED and toggle."""

    def __init__(self, process_id, name, on_toggle, is_setup=False, disabled=False):
        super().__init__(orientation=Gtk.Orientation.HORIZONTAL, spacing=12)
        self.process_id = process_id
        self.get_style_context().add_class("process-card")
        if disabled:
            self.get_style_context().add_class("disabled")

        # Left side: LED + name
        left_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=12)
        left_box.set_valign(Gtk.Align.CENTER)

        self.led = LedIndicator()
        if disabled:
            self.led.set_state("disabled")
        left_box.pack_start(self.led, False, False, 0)

        label = Gtk.Label(label=name)
        label.get_style_context().add_class("process-name")
        left_box.pack_start(label, False, False, 0)

        self.pack_start(left_box, True, True, 0)

        # Right side
        if is_setup:
            right_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=12)
            right_box.set_valign(Gtk.Align.CENTER)

            self.setup_btn = Gtk.Button(label="SETUP")
            self.setup_btn.get_style_context().add_class("setup-btn")
            self.setup_btn.connect("clicked", lambda b: on_toggle(True))
            right_box.pack_start(self.setup_btn, False, False, 0)

            self.status_label = Gtk.Label(label="")
            self.status_label.get_style_context().add_class("setup-status")
            right_box.pack_start(self.status_label, False, False, 0)

            self.pack_end(right_box, False, False, 0)
            self.toggle = None
        elif not disabled:
            self.toggle = ToggleSwitch(callback=on_toggle)
            self.toggle.set_valign(Gtk.Align.CENTER)
            self.pack_end(self.toggle, False, False, 0)
            self.setup_btn = None
            self.status_label = None
        else:
            self.toggle = ToggleSwitch()
            self.toggle.set_enabled(False)
            self.toggle.set_valign(Gtk.Align.CENTER)
            self.pack_end(self.toggle, False, False, 0)
            self.setup_btn = None
            self.status_label = None


class ROS2Panel(Gtk.Window):
    """Main application window."""

    # Log colors per process name
    LOG_COLORS = {
        "SETUP": "#ff79c6",
        "Bringup": "#50fa7b",
        "SLAM": "#8be9fd",
        "Localize": "#ffb86c",
        "Pursuit": "#bd93f9",
        "Stanley": "#f1fa8c",
        "AI Inference": "#00d4ff",
        "Bluetooth": "#5b8af7",
    }

    def __init__(self):
        super().__init__(title="ROS2 Control Panel")
        self.set_default_size(700, 700)
        self.set_position(Gtk.WindowPosition.CENTER)

        self._load_css()

        self.manager = ProcessManager()
        self.cards = {}
        self.setup_running = False
        self._last_log_count = 0

        # Main layout
        self.main_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        self.main_box.get_style_context().add_class("main-container")

        scroll = Gtk.ScrolledWindow()
        scroll.set_policy(Gtk.PolicyType.NEVER, Gtk.PolicyType.AUTOMATIC)

        content = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=16)
        content.set_margin_top(30)
        content.set_margin_bottom(30)
        content.set_margin_start(30)
        content.set_margin_end(30)

        # Title
        title = Gtk.Label(label="ROS2 Control Panel")
        title.get_style_context().add_class("panel-title")
        content.pack_start(title, False, False, 0)

        # Battery bar
        self.battery = BatteryBar()
        content.pack_start(self.battery, False, False, 0)

        # SETUP card (full width)
        setup_card = ProcessCard(0, "SETUP", self._on_setup_click, is_setup=True)
        self.cards[0] = setup_card
        content.pack_start(setup_card, False, False, 0)

        # Grid 2x3
        grid = Gtk.Grid()
        grid.set_row_spacing(16)
        grid.set_column_spacing(16)
        grid.set_column_homogeneous(True)

        positions = [(0, 0, 1), (0, 1, 2), (1, 0, 3), (1, 1, 4), (2, 0, 5), (2, 1, 6)]
        for row, col, pid in positions:
            defn = PROCESS_DEFINITIONS[pid]
            disabled = defn["command"] is None
            card = ProcessCard(
                pid, defn["name"],
                lambda active, p=pid: self._on_toggle(p, active),
                disabled=disabled,
            )
            card.set_hexpand(True)
            self.cards[pid] = card
            grid.attach(card, col, row, 1, 1)

        content.pack_start(grid, False, False, 0)

        # Debug Console
        debug_frame = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        debug_frame.get_style_context().add_class("debug-console")

        # Debug header
        debug_header = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=0)
        debug_header.get_style_context().add_class("debug-header")

        debug_title = Gtk.Label(label="Debug Console")
        debug_title.set_halign(Gtk.Align.START)
        debug_header.pack_start(debug_title, True, True, 8)

        clear_btn = Gtk.Button(label="Clear")
        clear_btn.get_style_context().add_class("debug-clear")
        clear_btn.connect("clicked", self._on_clear_logs)
        debug_header.pack_end(clear_btn, False, False, 8)

        debug_frame.pack_start(debug_header, False, False, 0)

        # Debug output (TextView)
        debug_scroll = Gtk.ScrolledWindow()
        debug_scroll.set_policy(Gtk.PolicyType.AUTOMATIC, Gtk.PolicyType.AUTOMATIC)
        debug_scroll.set_min_content_height(250)
        debug_scroll.get_style_context().add_class("debug-scroll")

        self.log_view = Gtk.TextView()
        self.log_view.set_editable(False)
        self.log_view.set_cursor_visible(False)
        self.log_view.set_wrap_mode(Gtk.WrapMode.WORD_CHAR)
        self.log_view.get_style_context().add_class("debug-output")

        self.log_buffer = self.log_view.get_buffer()

        # Create text tags for colors
        self.log_buffer.create_tag("timestamp", foreground="#666666")
        for name, color in self.LOG_COLORS.items():
            self.log_buffer.create_tag(f"proc-{name}", foreground=color)
        self.log_buffer.create_tag("message", foreground="#39ff14")

        debug_scroll.add(self.log_view)
        debug_frame.pack_start(debug_scroll, True, True, 0)

        content.pack_start(debug_frame, True, True, 0)

        # Bottom action bar (Bluetooth + Shutdown)
        action_bar = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=16)
        action_bar.set_margin_top(16)

        # Bluetooth connect button
        self.bt_btn = Gtk.Button()
        self.bt_btn.get_style_context().add_class("bt-btn")
        bt_icon = Gtk.Image.new_from_icon_name("bluetooth", Gtk.IconSize.LARGE_TOOLBAR)
        self.bt_btn.set_image(bt_icon)
        self.bt_btn.set_always_show_image(True)
        self.bt_btn.connect("clicked", self._on_bluetooth_click)
        action_bar.pack_start(self.bt_btn, True, True, 0)

        # Shutdown button
        shutdown_btn = Gtk.Button(label="SHUTDOWN")
        shutdown_btn.get_style_context().add_class("shutdown-btn")
        shutdown_btn.connect("clicked", self._on_shutdown_click)
        action_bar.pack_start(shutdown_btn, True, True, 0)

        content.pack_start(action_bar, False, False, 0)

        scroll.add(content)
        self.main_box.pack_start(scroll, True, True, 0)
        self.add(self.main_box)

        # Start background monitor
        GLib.timeout_add(1000, self._update_status)
        GLib.timeout_add(500, self._update_logs)

        self.connect("destroy", self._on_quit)

    def _load_css(self):
        css_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "panel.css")
        if os.path.exists(css_path):
            provider = Gtk.CssProvider()
            provider.load_from_path(css_path)
            Gtk.StyleContext.add_provider_for_screen(
                Gdk.Screen.get_default(),
                provider,
                Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION,
            )

    def _on_setup_click(self, _active):
        card = self.cards[0]
        card.setup_btn.set_sensitive(False)
        self.setup_running = True
        card.status_label.set_text("Running setup...")
        card.status_label.get_style_context().remove_class("success")
        card.status_label.get_style_context().remove_class("error")
        card.status_label.get_style_context().add_class("running")
        card.led.set_state("starting")
        self.manager.start(0)

    def _on_toggle(self, process_id, active):
        if active:
            self.manager.start(process_id)
        else:
            self.manager.stop(process_id)

    def _update_status(self):
        statuses = self.manager.get_status()
        for pid_str, info in statuses.items():
            pid = int(pid_str) if isinstance(pid_str, str) else pid_str
            state = info["state"]
            card = self.cards.get(pid)
            if not card:
                continue

            card.led.set_state(state)

            # SETUP card
            if pid == 0:
                if state == "stopped" and self.setup_running:
                    self.setup_running = False
                    card.setup_btn.set_sensitive(True)
                    exit_code = info.get("exit_code")
                    if exit_code is None or exit_code == 0:
                        card.status_label.set_text("Setup complete!")
                        card.status_label.get_style_context().remove_class("running")
                        card.status_label.get_style_context().remove_class("error")
                        card.status_label.get_style_context().add_class("success")
                        card.led.set_state("running")
                    else:
                        card.status_label.set_text(f"Error (code {exit_code})")
                        card.status_label.get_style_context().remove_class("running")
                        card.status_label.get_style_context().remove_class("success")
                        card.status_label.get_style_context().add_class("error")
                continue

            # Toggle cards
            if card.toggle:
                if state in ("running", "starting"):
                    card.toggle.set_active(True)
                elif state == "stopped":
                    card.toggle.set_active(False)

        return True  # keep running

    def _update_logs(self):
        current_logs = self.manager.get_logs(200)
        current_total = len(list(self.manager._combined_log))

        if current_total > self._last_log_count:
            new_count = current_total - self._last_log_count
            new_lines = current_logs[-new_count:] if new_count <= len(current_logs) else current_logs

            for line in new_lines:
                self._append_log_line(line)

            self._last_log_count = current_total

            # Auto-scroll to bottom
            adj = self.log_view.get_parent().get_vadjustment()
            adj.set_value(adj.get_upper() - adj.get_page_size())

        return True  # keep running

    def _append_log_line(self, line):
        end_iter = self.log_buffer.get_end_iter()

        import re
        match = re.match(r'^(\[\d{2}:\d{2}:\d{2}\])\s+\[([^\]]+)\]\s+(.*)$', line)
        if match:
            timestamp, proc_name, message = match.groups()
            self.log_buffer.insert_with_tags_by_name(end_iter, timestamp + " ", "timestamp")
            end_iter = self.log_buffer.get_end_iter()

            tag_name = f"proc-{proc_name}"
            if self.log_buffer.get_tag_table().lookup(tag_name):
                self.log_buffer.insert_with_tags_by_name(end_iter, f"[{proc_name}] ", tag_name)
            else:
                self.log_buffer.insert_with_tags_by_name(end_iter, f"[{proc_name}] ", "message")
            end_iter = self.log_buffer.get_end_iter()

            self.log_buffer.insert_with_tags_by_name(end_iter, message + "\n", "message")
        else:
            self.log_buffer.insert_with_tags_by_name(end_iter, line + "\n", "message")

        # Limit log lines
        line_count = self.log_buffer.get_line_count()
        if line_count > 500:
            start = self.log_buffer.get_start_iter()
            cut = self.log_buffer.get_iter_at_line(line_count - 500)
            self.log_buffer.delete(start, cut)

    def _on_clear_logs(self, _btn):
        self.log_buffer.set_text("")

    def _on_bluetooth_click(self, _btn):
        self.bt_btn.set_sensitive(False)
        self.manager.log("Bluetooth", "Connecting to 57:9D:F2:83:5C:51...")
        threading.Thread(target=self._bluetooth_connect, daemon=True).start()

    def _bluetooth_connect(self):
        try:
            result = subprocess.run(
                ["bluetoothctl", "connect", "57:9D:F2:83:5C:51"],
                capture_output=True, text=True, timeout=15,
            )
            output = result.stdout.strip().split("\n")
            msg = output[-1] if output else "No response"
            GLib.idle_add(self._bt_done, msg)
        except subprocess.TimeoutExpired:
            GLib.idle_add(self._bt_done, "Connection timed out")
        except Exception as e:
            GLib.idle_add(self._bt_done, f"Error: {e}")

    def _bt_done(self, msg):
        self.manager.log("Bluetooth", msg)
        self.bt_btn.set_sensitive(True)

    def _on_shutdown_click(self, _btn):
        # Passwordless via /etc/sudoers.d/f1tenth (installed by install.sh).
        # -n so we fail fast instead of hanging on a password prompt.
        result = subprocess.run(
            ["sudo", "-n", "shutdown", "-h", "now"],
            capture_output=True, text=True,
        )
        if result.returncode != 0:
            self.manager.log(
                "Shutdown",
                "Failed: {} (is /etc/sudoers.d/f1tenth installed? run ./install.sh)".format(
                    (result.stderr or "").strip() or "sudo refused"
                ),
            )

    def _on_quit(self, _widget):
        self.battery.stop()
        # Stop all running processes on exit
        for pid in list(self.manager.processes.keys()):
            self.manager.stop(pid)
        Gtk.main_quit()


def main():
    # Show boot animation, then main window
    def show_main():
        win = ROS2Panel()
        win.show_all()

    boot = BootOverlay(on_done=show_main)
    boot.show_all()

    Gtk.main()


if __name__ == "__main__":
    main()
