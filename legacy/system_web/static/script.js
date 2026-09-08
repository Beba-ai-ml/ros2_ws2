document.addEventListener("DOMContentLoaded", function () {
    var socket = io();
    var startingProcesses = {};
    var setupRunning = false;

    // ========== Boot Animation ==========
    setTimeout(function () {
        document.body.classList.add("boot-done");
    }, 5000);

    // ========== WebSocket Handlers ==========
    socket.on("connect", function () {
        console.log("WebSocket connected");
    });

    socket.on("disconnect", function () {
        console.log("WebSocket disconnected");
    });

    socket.on("status_update", function (statuses) {
        for (var id in statuses) {
            var info = statuses[id];
            var state = info.state;
            var led = document.getElementById("led-" + id);

            if (!led) continue;

            // Handle SETUP (id=0) separately - no toggle
            if (id === "0") {
                if (state === "disabled") {
                    led.className = "led led-disabled";
                } else if (state === "running" || state === "starting") {
                    led.className = "led led-starting";
                } else if (state === "stopped") {
                    led.className = "led led-stopped";
                    if (setupRunning) {
                        setupRunning = false;
                        var setupBtn = document.getElementById("setup-btn");
                        var setupStatus = document.getElementById("setup-status");
                        if (setupBtn) setupBtn.disabled = false;
                        if (setupStatus) {
                            if (info.exit_code === 0 || info.exit_code === undefined) {
                                setupStatus.textContent = "Setup complete!";
                                setupStatus.className = "setup-status success";
                            } else {
                                setupStatus.textContent = "Error (code " + info.exit_code + ")";
                                setupStatus.className = "setup-status error";
                            }
                        }
                    }
                }
                continue;
            }

            var toggle = document.getElementById("toggle-" + id);
            if (!toggle) continue;

            if (state === "disabled") {
                led.className = "led led-disabled";
                toggle.checked = false;
                continue;
            }

            if (state === "starting") {
                led.className = "led led-starting";
                toggle.checked = true;
            } else if (state === "running") {
                led.className = "led led-running";
                toggle.checked = true;
                delete startingProcesses[id];
            } else {
                led.className = "led led-stopped";
                toggle.checked = false;
                delete startingProcesses[id];
            }
        }
    });

    // ========== Debug Console ==========
    var debugOutput = document.getElementById("debug-output");
    var debugClear = document.getElementById("debug-clear");

    socket.on("log_update", function (data) {
        if (!data || !data.lines || !debugOutput) return;

        data.lines.forEach(function (line) {
            var div = document.createElement("div");
            div.className = "log-line";
            div.innerHTML = colorizeLogLine(line);
            debugOutput.appendChild(div);
        });

        while (debugOutput.children.length > 500) {
            debugOutput.removeChild(debugOutput.firstChild);
        }

        debugOutput.scrollTop = debugOutput.scrollHeight;
    });

    if (debugClear) {
        debugClear.addEventListener("click", function () {
            if (debugOutput) debugOutput.innerHTML = "";
        });
    }

    function colorizeLogLine(line) {
        var match = line.match(/^(\[\d{2}:\d{2}:\d{2}\])\s+\[([^\]]+)\]\s+(.*)$/);
        if (match) {
            var timestamp = match[1];
            var processName = match[2];
            var message = match[3];
            var cssClass = "log-process-" + processName.replace(/\s+/g, "_");
            return '<span style="color:#666">' + escapeHtml(timestamp) + '</span> ' +
                   '<span class="' + cssClass + '">[' + escapeHtml(processName) + ']</span> ' +
                   escapeHtml(message);
        }
        return escapeHtml(line);
    }

    function escapeHtml(text) {
        var div = document.createElement("div");
        div.appendChild(document.createTextNode(text));
        return div.innerHTML;
    }

    // ========== SETUP Button ==========
    var setupBtn = document.getElementById("setup-btn");
    if (setupBtn) {
        setupBtn.addEventListener("click", function () {
            var setupStatus = document.getElementById("setup-status");
            setupBtn.disabled = true;
            setupRunning = true;

            if (setupStatus) {
                setupStatus.textContent = "Running setup...";
                setupStatus.className = "setup-status running";
            }

            var led = document.getElementById("led-0");
            if (led) led.className = "led led-starting";

            fetch("/api/start/0", { method: "POST" })
                .then(function (r) { return r.json(); })
                .then(function (data) {
                    if (data.error) {
                        console.error("Setup error:", data.error);
                        setupBtn.disabled = false;
                        setupRunning = false;
                        if (setupStatus) {
                            setupStatus.textContent = data.error;
                            setupStatus.className = "setup-status error";
                        }
                    }
                })
                .catch(function (err) {
                    console.error("Fetch error:", err);
                    setupBtn.disabled = false;
                    setupRunning = false;
                    if (setupStatus) {
                        setupStatus.textContent = "Network error";
                        setupStatus.className = "setup-status error";
                    }
                });
        });
    }

    // ========== Toggle Switches (processes 1-6) ==========
    var toggles = document.querySelectorAll(".toggle-switch input:not(:disabled)");
    toggles.forEach(function (toggle) {
        toggle.addEventListener("change", function () {
            var card = this.closest(".process-card");
            var processId = card.getAttribute("data-id");

            if (this.checked) {
                startProcess(processId);
            } else {
                stopProcess(processId);
            }
        });
    });

    function startProcess(id) {
        startingProcesses[id] = true;
        var led = document.getElementById("led-" + id);
        if (led) led.className = "led led-starting";

        fetch("/api/start/" + id, { method: "POST" })
            .then(function (r) { return r.json(); })
            .then(function (data) {
                if (data.error) {
                    console.error("Start error:", data.error);
                    delete startingProcesses[id];
                    var toggle = document.getElementById("toggle-" + id);
                    if (toggle) toggle.checked = false;
                }
            })
            .catch(function (err) {
                console.error("Fetch error:", err);
                delete startingProcesses[id];
            });
    }

    function stopProcess(id) {
        delete startingProcesses[id];
        fetch("/api/stop/" + id, { method: "POST" })
            .then(function (r) { return r.json(); })
            .then(function (data) {
                if (data.error) {
                    console.error("Stop error:", data.error);
                }
            })
            .catch(function (err) {
                console.error("Fetch error:", err);
            });
    }
});
