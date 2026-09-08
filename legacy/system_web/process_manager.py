import subprocess
import os
import signal
import time
import re
import collections
import eventlet

# Workspace root: this file lives in <ws>/legacy/system_web/, so go up two levels.
WS_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

ROS_DISTRO_SETUP = "/opt/ros/foxy/setup.bash"

PROCESS_DEFINITIONS = {
    0: {
        "name": "SETUP",
        "command": (
            # sudo -n relies on /etc/sudoers.d/f1tenth installed by install.sh.
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
            "ros2 launch slam_toolbox online_async_launch.py "
            f"params_file:={WS_ROOT}/src/slam_toolbox/config/mapper_params_online_async.yaml"
        ),
    },
    3: {
        "name": "Reserved",
        "command": None,
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
        self._reader_greenlets = {}

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

        proc = subprocess.Popen(
            full_command,
            shell=True,
            executable="/bin/bash",
            cwd=WORKING_DIR,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
        self.processes[process_id] = proc
        self.start_times[process_id] = time.time()
        self.log_buffers[process_id] = collections.deque(maxlen=MAX_LOG_LINES)
        self.bringup_lidar_ready[process_id] = False

        self._start_stdout_reader(process_id, proc, definition["name"])

        return {"status": "started", "pid": proc.pid, "name": definition["name"]}

    def _start_stdout_reader(self, process_id, proc, process_name):
        if process_id in self._reader_greenlets:
            self._reader_greenlets[process_id].cancel()

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

                    eventlet.sleep(0)
            except Exception:
                pass

        self._reader_greenlets[process_id] = eventlet.spawn(_reader)

    def stop(self, process_id):
        process_id = int(process_id)
        if process_id not in self.processes:
            return {"error": "Process not tracked"}

        proc = self.processes[process_id]
        if proc.poll() is not None:
            self._cleanup_process(process_id)
            return {"status": "already_stopped"}

        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass

        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass

        name = PROCESS_DEFINITIONS.get(process_id, {}).get("name", "unknown")
        self._cleanup_process(process_id)
        return {"status": "stopped", "name": name}

    def _cleanup_process(self, process_id):
        self.processes.pop(process_id, None)
        self.start_times.pop(process_id, None)
        self.bringup_lidar_ready.pop(process_id, False)
        if process_id in self._reader_greenlets:
            self._reader_greenlets[process_id].cancel()
            del self._reader_greenlets[process_id]

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

    def get_logs(self, count=200):
        log_list = list(self._combined_log)
        return log_list[-count:]
