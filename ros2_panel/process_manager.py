import subprocess
import os
import signal
import time
import re
import collections
import threading

# Workspace root: this file lives in <ws>/ros2_panel/, so go up one level.
WS_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

ROS_DISTRO_SETUP = "/opt/ros/foxy/setup.bash"

PROCESS_DEFINITIONS = {
    0: {
        "name": "SETUP",
        "command": (
            # Device permissions need root; sudo -n relies on /etc/sudoers.d/f1tenth
            # installed by install.sh (no password prompt, no password in this file).
            "sudo -n chmod 666 /dev/rplidar 2>/dev/null; sudo -n chmod 666 /dev/vesc 2>/dev/null; "
            f"source {ROS_DISTRO_SETUP} && "
            f"source {WS_ROOT}/install/setup.bash && "
            f"python3 {WS_ROOT}/scripts/ledy.py &"
        ),
    },
    1: {
        "name": "Bringup",
        "command": (
            "colcon build --packages-select f1tenth_stack && "
            ". install/setup.bash && "
            "ros2 launch f1tenth_stack bringup_launch3.py"
        ),
    },
    2: {
        "name": "SLAM",
        "command": (
            f"{{ (sleep 3 && rviz2 -d {WS_ROOT}/config/slam_rviz.rviz "
            ">/dev/null 2>&1) & } 2>/dev/null; "
            "ros2 launch slam_toolbox online_async_launch.py "
            f"params_file:={WS_ROOT}/src/slam_toolbox/config/mapper_params_online_async.yaml"
        ),
    },
    3: {
        "name": "AI Inference",
        "command": (
            "colcon build --packages-select sac_driver && "
            ". install/setup.bash && "
            "ros2 run sac_driver sac_driver_node --ros-args "
            f"--params-file {WS_ROOT}/src/sac_driver/config/driver_params.yaml"
        ),
    },
    4: {
        "name": "Localize",
        "command": (
            "colcon build --packages-select particle_filter && "
            ". install/setup.bash && "
            "ros2 launch particle_filter localize_launch.py"
        ),
    },
    5: {
        "name": "Pursuit",
        "command": (
            "colcon build --packages-select pure_pursuit && "
            ". install/setup.bash && "
            "ros2 launch pure_pursuit pure_pursuit_launch.py"
        ),
    },
    6: {
        "name": "Stanley",
        "command": (
            "colcon build --packages-select stanley_avoidance && "
            ". install/setup.bash && "
            "ros2 launch stanley_avoidance stanley_avoidance_launch.py"
        ),
    },
}

ROS2_PREFIX = (
    f"source {ROS_DISTRO_SETUP} && "
    f"source {WS_ROOT}/install/setup.bash && "
)
WORKING_DIR = WS_ROOT

LIDAR_READY_PATTERN = re.compile(r"(lidar|laser|scan|rplidar)", re.IGNORECASE)
MIN_UPTIME_FOR_RUNNING = 3
MAX_LOG_LINES = 1000


class ProcessManager:
    def __init__(self):
        self.processes = {}
        self.start_times = {}
        self.log_buffers = {}
        self.bringup_lidar_ready = {}
        self._combined_log = collections.deque(maxlen=MAX_LOG_LINES)
        self._reader_threads = {}

    def start(self, process_id):
        process_id = int(process_id)
        if process_id not in PROCESS_DEFINITIONS:
            return {"error": f"Unknown process id: {process_id}"}

        definition = PROCESS_DEFINITIONS[process_id]
        if definition["command"] is None:
            return {"error": f"Process '{definition['name']}' is reserved and cannot be started"}

        if process_id in self.processes and self.processes[process_id].poll() is None:
            return {"error": f"Process '{definition['name']}' is already running"}

        if process_id == 0:
            full_command = definition["command"]
        else:
            full_command = ROS2_PREFIX + definition["command"]

        env = os.environ.copy()
        env.setdefault("DISPLAY", ":1")

        proc = subprocess.Popen(
            full_command,
            shell=True,
            executable="/bin/bash",
            cwd=WORKING_DIR,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
            env=env,
        )
        self.processes[process_id] = proc
        self.start_times[process_id] = time.time()
        self.log_buffers[process_id] = collections.deque(maxlen=MAX_LOG_LINES)
        self.bringup_lidar_ready[process_id] = False

        self._start_stdout_reader(process_id, proc, definition["name"])

        return {"status": "started", "pid": proc.pid, "name": definition["name"]}

    def _start_stdout_reader(self, process_id, proc, process_name):
        def _reader():
            try:
                while True:
                    line = proc.stdout.readline()
                    if not line:
                        break
                    try:
                        decoded = line.decode("utf-8", errors="replace").rstrip("\n").rstrip("\r")
                    except Exception:
                        decoded = str(line)

                    timestamp = time.strftime("%H:%M:%S")
                    prefixed_line = "[{}] [{}] {}".format(timestamp, process_name, decoded)

                    buf = self.log_buffers.get(process_id)
                    if buf is not None:
                        buf.append(prefixed_line)
                    self._combined_log.append(prefixed_line)

                    if process_id == 1 and not self.bringup_lidar_ready.get(process_id, False):
                        if LIDAR_READY_PATTERN.search(decoded):
                            self.bringup_lidar_ready[process_id] = True
            except Exception:
                pass

        t = threading.Thread(target=_reader, daemon=True)
        t.start()
        self._reader_threads[process_id] = t

    def stop(self, process_id):
        process_id = int(process_id)
        if process_id not in self.processes:
            return {"error": "Process not tracked"}

        proc = self.processes[process_id]
        if proc.poll() is not None:
            self._cleanup_process(process_id)
            return {"status": "already_stopped"}

        # Kill the process group (shell + direct children)
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except ProcessLookupError:
            pass

        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                pass

        # For bringup (id=1): kill orphaned ROS2 nodes that ros2 launch spawned
        if process_id == 1:
            self._kill_ros2_orphans()

        # For SLAM (id=2): kill RViz launched alongside it
        if process_id == 2:
            try:
                subprocess.run(["pkill", "-f", "rviz2"], timeout=2, capture_output=True)
            except Exception:
                pass

        # For AI inference (id=3): kill orphaned sac_driver node
        if process_id == 3:
            try:
                subprocess.run(["pkill", "-f", "sac_driver_node"], timeout=2, capture_output=True)
            except Exception:
                pass

        name = PROCESS_DEFINITIONS.get(process_id, {}).get("name", "unknown")
        self._cleanup_process(process_id)
        return {"status": "stopped", "name": name}

    def _kill_ros2_orphans(self):
        """Kill ROS2 nodes that survive after ros2 launch is terminated."""
        orphan_patterns = [
            "sllidar_ros2_node",
            "vesc_driver_node",
            "ackermann_to_vesc_node",
            "vesc_to_odom_node",
            "joy_node",
            "joy_mode_manager",
            "ackermann_mux",
            "static_transform_publisher",
        ]
        for pattern in orphan_patterns:
            try:
                subprocess.run(
                    ["pkill", "-f", pattern],
                    timeout=2, capture_output=True,
                )
            except Exception:
                pass

    def _cleanup_process(self, process_id):
        self.processes.pop(process_id, None)
        self.start_times.pop(process_id, None)
        self.bringup_lidar_ready.pop(process_id, False)
        self._reader_threads.pop(process_id, None)

    def _determine_state(self, process_id):
        if process_id == 1:
            if self.bringup_lidar_ready.get(process_id, False):
                return "running"
            return "starting"

        start_time = self.start_times.get(process_id)
        if start_time and (time.time() - start_time) >= MIN_UPTIME_FOR_RUNNING:
            return "running"
        return "starting"

    def get_status(self):
        statuses = {}
        to_remove = []
        for pid_key, definition in PROCESS_DEFINITIONS.items():
            if definition["command"] is None:
                statuses[pid_key] = {"name": definition["name"], "state": "disabled"}
                continue

            if pid_key not in self.processes:
                statuses[pid_key] = {"name": definition["name"], "state": "stopped"}
                continue

            proc = self.processes[pid_key]
            poll_result = proc.poll()
            if poll_result is None:
                state = self._determine_state(pid_key)
                statuses[pid_key] = {"name": definition["name"], "state": state}
            else:
                statuses[pid_key] = {
                    "name": definition["name"],
                    "state": "stopped",
                    "exit_code": poll_result,
                }
                to_remove.append(pid_key)

        for key in to_remove:
            self._cleanup_process(key)

        return statuses

    def log(self, source, message):
        """Add a manual log entry (not tied to a process)."""
        timestamp = time.strftime("%H:%M:%S")
        self._combined_log.append(f"[{timestamp}] [{source}] {message}")

    def get_logs(self, count=200):
        log_list = list(self._combined_log)
        return log_list[-count:]
