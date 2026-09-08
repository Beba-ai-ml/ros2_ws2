#!/usr/bin/env python3
import eventlet
eventlet.monkey_patch()

import os
import secrets

from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO
from process_manager import ProcessManager

app = Flask(__name__)
# Set ROS2_PANEL_SECRET to keep sessions valid across restarts.
app.config["SECRET_KEY"] = os.environ.get("ROS2_PANEL_SECRET") or secrets.token_hex(16)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="eventlet")

manager = ProcessManager()


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/start/<int:process_id>", methods=["POST"])
def start_process(process_id):
    result = manager.start(process_id)
    broadcast_status()
    return jsonify(result)


@app.route("/api/stop/<int:process_id>", methods=["POST"])
def stop_process(process_id):
    result = manager.stop(process_id)
    broadcast_status()
    return jsonify(result)


@app.route("/api/status")
def get_status():
    return jsonify(manager.get_status())


@app.route("/api/logs")
def get_logs():
    count = request.args.get("count", 200, type=int)
    return jsonify({"lines": manager.get_logs(count)})


@socketio.on("connect")
def handle_connect():
    broadcast_status()


def broadcast_status():
    statuses = manager.get_status()
    socketio.emit("status_update", statuses)


def status_monitor():
    """Background thread that emits process status every second."""
    while True:
        eventlet.sleep(1)
        broadcast_status()


def log_monitor():
    """Background thread that emits new log lines every second."""
    last_count = 0
    while True:
        eventlet.sleep(1)
        current_logs = manager.get_logs(200)
        current_total = len(list(manager._combined_log))
        if current_total > last_count:
            new_count = current_total - last_count
            new_lines = current_logs[-new_count:] if new_count <= len(current_logs) else current_logs
            socketio.emit("log_update", {"lines": new_lines})
            last_count = current_total


if __name__ == "__main__":
    socketio.start_background_task(status_monitor)
    socketio.start_background_task(log_monitor)
    print("=== ROS2 Control Panel ===")
    print("Running on http://0.0.0.0:8080")
    socketio.run(app, host="0.0.0.0", port=8080, debug=False)
