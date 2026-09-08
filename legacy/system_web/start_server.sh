#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=== ROS2 Control Panel - Start ==="

# Check and install Python dependencies
for pkg in flask flask-socketio eventlet; do
    if ! python3 -c "import ${pkg//-/_}" 2>/dev/null; then
        echo "Installing $pkg..."
        pip3 install "$pkg"
    fi
done

echo "Starting server on port 8080..."
python3 server.py
