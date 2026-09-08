#!/usr/bin/env bash
#
# F1TENTH / ros2_ws installer for NVIDIA Jetson (Orin Nano, JetPack 5.x,
# Ubuntu 20.04 aarch64, ROS 2 Foxy, Python 3.8).
#
#   git clone https://github.com/Beba-ai-ml/ros2_ws2.git ~/ros2_ws
#   cd ~/ros2_ws && ./install.sh
#   sudo reboot
#
# Idempotent: safe to re-run. Run it as your normal user (NOT with sudo) --
# it calls sudo itself for the few steps that need root.
#
# See ./install.sh --help for flags.

set -eo pipefail   # deliberately no -u: ROS setup.bash reads unset variables

# ---------------------------------------------------------------------------
# Paths and constants
# ---------------------------------------------------------------------------

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_USER="${SUDO_USER:-$(id -un)}"
RUN_HOME="$(getent passwd "$RUN_USER" | cut -d: -f6)"
RUN_HOME="${RUN_HOME:-$HOME}"

ROS_DISTRO_NAME="foxy"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"

UDEV_RULES_SRC="${WS}/system/udev/99-f1tenth.rules"
UDEV_RULES_DST="/etc/udev/rules.d/99-f1tenth.rules"
SUDOERS_SRC="${WS}/system/sudoers.d/f1tenth.in"
SUDOERS_DST="/etc/sudoers.d/f1tenth"
SPIDEV_CONF="/etc/modules-load.d/spidev.conf"
GPIO_SERVICE_SRC="${WS}/system/systemd/gpio-shutdown.service.in"
KEY_DRIVE_SERVICE_SRC="${WS}/system/systemd/key_drive.service.in"
BT_PAD_SERVICE_SRC="${WS}/system/systemd/bt_pad.service.in"
DESKTOP_SRC="${WS}/system/desktop/ROS2-Panel.desktop.in"
REQUIREMENTS="${WS}/requirements.txt"
WEIGHTS_FILE="${WS}/src/sac_driver/weights/session_Rybnik_02_1.pth"
RANGE_LIBC_PYWRAPPER="${WS}/src/range_libc/pywrapper"

# Packages built by default (everything needed to drive).
CORE_PACKAGES=(sac_driver f1tenth_stack ackermann_mux sllidar_ros2)
# Extra packages built with --full (mapping / planning / classic controllers).
FULL_PACKAGES=(particle_filter pure_pursuit stanley_avoidance waypoint_generator
               gap_follow wall_follow safety_node scan_matching)
# Never built from source: slam_toolbox comes from apt, range_libc is a plain
# CMake/Cython project that colcon must not try to build.
SKIP_PACKAGES=(slam_toolbox range_libc)

# rosdep keys we resolve ourselves (pip wheels) or deliberately do not want.
ROSDEP_SKIP_KEYS="torch numpy python-transforms3d-pip rosbridge_server urg_node libceres-dev slam_toolbox range_libc"

USER_GROUPS=(dialout gpio input plugdev video)

APT_PACKAGES=(
  ros-${ROS_DISTRO_NAME}-vesc
  ros-${ROS_DISTRO_NAME}-vesc-ackermann
  ros-${ROS_DISTRO_NAME}-vesc-driver
  ros-${ROS_DISTRO_NAME}-vesc-msgs
  ros-${ROS_DISTRO_NAME}-ackermann-msgs
  ros-${ROS_DISTRO_NAME}-joy
  ros-${ROS_DISTRO_NAME}-joy-linux
  ros-${ROS_DISTRO_NAME}-joy-teleop
  ros-${ROS_DISTRO_NAME}-teleop-tools
  ros-${ROS_DISTRO_NAME}-diagnostic-updater
  ros-${ROS_DISTRO_NAME}-slam-toolbox
  ros-${ROS_DISTRO_NAME}-nav2-map-server
  ros-${ROS_DISTRO_NAME}-nav2-lifecycle-manager
  ros-${ROS_DISTRO_NAME}-rviz2
  ros-${ROS_DISTRO_NAME}-tf2-ros
  ros-${ROS_DISTRO_NAME}-tf2-geometry-msgs
  ros-${ROS_DISTRO_NAME}-tf2-tools
  ros-${ROS_DISTRO_NAME}-tf-transformations
  ros-${ROS_DISTRO_NAME}-rplidar-ros
  ros-${ROS_DISTRO_NAME}-robot-state-publisher
  python3-colcon-common-extensions
  python3-rosdep
  python3-pip
  python3-gi
  gir1.2-gtk-3.0
  python3-serial
  libx11-6
  git
  build-essential
  cmake
  libeigen3-dev
)

# Python modules that must be importable afterwards: "module:human name".
PY_IMPORTS=(torch numpy rclpy serial evdev gi yaml)

# ---------------------------------------------------------------------------
# Flags
# ---------------------------------------------------------------------------

DO_CHECK=0
WITH_ROS=0
FULL=0
NO_BUILD=0
GPIO_SHUTDOWN=0
KEY_DRIVE_SERVICE=0
BT_PAD=0
DESKTOP=0
ASSUME_YES=0

usage() {
  cat <<EOF
Usage: ./install.sh [FLAGS]

  (no flags)            core install: apt deps, rosdep, pip deps, udev, spidev,
                        groups, sudoers, .bashrc, colcon build of the core
                        packages (${CORE_PACKAGES[*]})

  --check               verify only, change nothing; PASS/FAIL per item,
                        exit 1 if anything required is missing
  --with-ros            install ROS 2 ${ROS_DISTRO_NAME^} from apt if /opt/ros/${ROS_DISTRO_NAME} is missing
  --full                also build ${FULL_PACKAGES[*]}
                        and the range_libc python wrapper
  --no-build            skip the colcon build step
  --gpio-shutdown       install + enable the gpio-shutdown systemd service
  --key-drive-service   install + enable key_drive.service (DANGEROUS: the car
                        drives from a plugged-in keyboard right after boot)
  --bt-pad              install + enable bt_pad.service (Bluetooth DualShock 4
                        auto-connect loop; set PAD_MAC in scripts/bt_pad_connect.sh)
  --desktop             put a ROS2 Control Panel launcher on ~/Desktop
  --yes                 non-interactive, assume yes to every prompt
  -h, --help            this help
EOF
}

while [ $# -gt 0 ]; do
  case "$1" in
    --check)             DO_CHECK=1 ;;
    --with-ros)          WITH_ROS=1 ;;
    --full)              FULL=1 ;;
    --no-build)          NO_BUILD=1 ;;
    --gpio-shutdown)     GPIO_SHUTDOWN=1 ;;
    --key-drive-service) KEY_DRIVE_SERVICE=1 ;;
    --bt-pad)            BT_PAD=1 ;;
    --desktop)           DESKTOP=1 ;;
    --yes|-y)            ASSUME_YES=1 ;;
    -h|--help)           usage; exit 0 ;;
    *) echo "Unknown flag: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------

if [ -t 1 ]; then
  C_RESET=$'\033[0m'; C_BOLD=$'\033[1m'; C_RED=$'\033[31m'
  C_GREEN=$'\033[32m'; C_YELLOW=$'\033[33m'; C_BLUE=$'\033[34m'
else
  C_RESET=; C_BOLD=; C_RED=; C_GREEN=; C_YELLOW=; C_BLUE=
fi

STEP_NO=0
step()  { STEP_NO=$((STEP_NO + 1)); printf '\n%s==> [%d] %s%s\n' "${C_BOLD}${C_BLUE}" "$STEP_NO" "$*" "$C_RESET"; }
ok()    { printf '    %s[ ok ]%s %s\n'   "$C_GREEN"  "$C_RESET" "$*"; }
info()  { printf '    %s[info]%s %s\n'   "$C_BLUE"   "$C_RESET" "$*"; }
warn()  { printf '    %s[warn]%s %s\n'   "$C_YELLOW" "$C_RESET" "$*"; }
die()   { printf '\n%s[FAIL]%s %s\n' "$C_RED" "$C_RESET" "$*" >&2; exit 1; }

confirm() {
  # confirm "question" -> 0 yes / 1 no
  if [ "$ASSUME_YES" = "1" ]; then return 0; fi
  local reply
  read -r -p "    ${C_YELLOW}?${C_RESET} $1 [y/N] " reply || reply=""
  case "$reply" in [yY]|[yY][eE][sS]) return 0 ;; *) return 1 ;; esac
}

# --check bookkeeping
CHECK_FAILURES=0
CHECK_WARNINGS=0
pass()   { printf '    %sPASS%s %s\n' "$C_GREEN"  "$C_RESET" "$*"; }
fail()   { printf '    %sFAIL%s %s\n' "$C_RED"    "$C_RESET" "$*"; CHECK_FAILURES=$((CHECK_FAILURES + 1)); }
softwarn() { printf '    %sWARN%s %s\n' "$C_YELLOW" "$C_RESET" "$*"; CHECK_WARNINGS=$((CHECK_WARNINGS + 1)); }

# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------

render_template() {
  # render_template SRC DST -- substitutes @WS@ / @USER@ / @HOME@
  sed -e "s|@WS@|${WS}|g" -e "s|@USER@|${RUN_USER}|g" -e "s|@HOME@|${RUN_HOME}|g" "$1" > "$2"
}

py_import_ok() { python3 -c "import $1" >/dev/null 2>&1; }

apt_installed() { dpkg -s "$1" 2>/dev/null | grep -q '^Status: install ok installed'; }

in_group() { id -nG "$RUN_USER" 2>/dev/null | tr ' ' '\n' | grep -qx "$1"; }

installed_pkg_present() { [ -d "${WS}/install/$1" ]; }

skip_args() {
  # echoes: --packages-skip a b c
  printf -- '--packages-skip'
  local p
  for p in "${SKIP_PACKAGES[@]}"; do printf ' %s' "$p"; done
}

# ===========================================================================
# --check
# ===========================================================================

run_check() {
  printf '%s%sros2_ws install check%s\n' "$C_BOLD" "$C_BLUE" "$C_RESET"
  printf '  workspace: %s\n' "$WS"
  printf '  user:      %s\n' "$RUN_USER"

  step "Platform"
  local arch; arch="$(uname -m)"
  if [ "$arch" = "aarch64" ]; then pass "architecture $arch"; else softwarn "architecture $arch (expected aarch64)"; fi
  local rel; rel="$(. /etc/os-release 2>/dev/null && echo "${VERSION_ID}")"
  if [ "$rel" = "20.04" ]; then pass "Ubuntu $rel"; else softwarn "Ubuntu '${rel:-unknown}' (expected 20.04)"; fi

  step "ROS 2 ${ROS_DISTRO_NAME}"
  if [ -f "$ROS_SETUP" ]; then pass "$ROS_SETUP"; else fail "$ROS_SETUP missing (re-run with --with-ros)"; fi

  step "apt packages"
  local p missing=0
  for p in "${APT_PACKAGES[@]}"; do
    if apt_installed "$p"; then pass "$p"; else fail "$p"; missing=$((missing + 1)); fi
  done
  if [ "$missing" -eq 0 ]; then info "all ${#APT_PACKAGES[@]} apt packages present"; fi

  step "Python imports"
  for p in "${PY_IMPORTS[@]}"; do
    if py_import_ok "$p"; then pass "import $p"; else fail "import $p"; fi
  done
  if py_import_ok torch; then
    info "torch $(python3 -c 'import torch; print(torch.__version__)' 2>/dev/null)"
  fi
  if py_import_ok numpy; then
    info "numpy $(python3 -c 'import numpy; print(numpy.__version__)' 2>/dev/null)"
  fi

  step "udev rules"
  if [ ! -f "$UDEV_RULES_DST" ]; then
    fail "$UDEV_RULES_DST missing"
  elif cmp -s "$UDEV_RULES_SRC" "$UDEV_RULES_DST"; then
    pass "$UDEV_RULES_DST matches repo copy"
  else
    fail "$UDEV_RULES_DST differs from ${UDEV_RULES_SRC#$WS/} (re-run ./install.sh)"
  fi

  step "spidev module"
  if [ -f "$SPIDEV_CONF" ] && grep -qx 'spidev' "$SPIDEV_CONF"; then
    pass "$SPIDEV_CONF"
  else
    fail "$SPIDEV_CONF missing or empty"
  fi

  step "user groups"
  for p in "${USER_GROUPS[@]}"; do
    if in_group "$p"; then pass "$RUN_USER in group $p"; else fail "$RUN_USER not in group $p (log out / reboot after install)"; fi
  done

  step "sudoers"
  if [ -f "$SUDOERS_DST" ]; then pass "$SUDOERS_DST"; else fail "$SUDOERS_DST missing (panel shutdown will not work)"; fi

  step "colcon workspace"
  for p in "${CORE_PACKAGES[@]}"; do
    if installed_pkg_present "$p"; then pass "install/$p"; else fail "install/$p missing (run ./install.sh)"; fi
  done
  if [ "$FULL" = "1" ]; then
    for p in "${FULL_PACKAGES[@]}"; do
      if installed_pkg_present "$p"; then pass "install/$p"; else fail "install/$p missing (run ./install.sh --full)"; fi
    done
  fi

  step "model weights"
  if [ -f "$WEIGHTS_FILE" ]; then
    pass "${WEIGHTS_FILE#$WS/} ($(du -h "$WEIGHTS_FILE" | cut -f1))"
  else
    fail "${WEIGHTS_FILE#$WS/} missing (git lfs / full clone?)"
  fi

  step "devices (plug the car in for these)"
  for p in /dev/vesc /dev/rplidar; do
    if [ -e "$p" ]; then pass "$p -> $(readlink -f "$p")"; else softwarn "$p not present (device unplugged?)"; fi
  done

  step "~/ros2_ws"
  local home_ws="${RUN_HOME}/ros2_ws"
  if [ ! -e "$home_ws" ]; then
    fail "$home_ws does not exist"
  elif [ "$(readlink -f "$home_ws")" = "$WS" ]; then
    pass "$home_ws -> $WS"
  else
    fail "$home_ws resolves to $(readlink -f "$home_ws"), not $WS"
  fi

  printf '\n'
  if [ "$CHECK_FAILURES" -eq 0 ]; then
    printf '%s%sAll required checks passed%s' "$C_BOLD" "$C_GREEN" "$C_RESET"
  else
    printf '%s%s%d check(s) FAILED%s' "$C_BOLD" "$C_RED" "$CHECK_FAILURES" "$C_RESET"
  fi
  if [ "$CHECK_WARNINGS" -gt 0 ]; then printf ' (%d warning(s))' "$CHECK_WARNINGS"; fi
  printf '\n'
  [ "$CHECK_FAILURES" -eq 0 ]
}

if [ "$DO_CHECK" = "1" ]; then
  if run_check; then exit 0; else exit 1; fi
fi

# ===========================================================================
# Install
# ===========================================================================

printf '%s%sros2_ws installer%s\n' "$C_BOLD" "$C_BLUE" "$C_RESET"
printf '  workspace: %s\n' "$WS"
printf '  user:      %s\n' "$RUN_USER"

# --- 1. sanity ------------------------------------------------------------
step "Sanity checks"

if [ "$(id -u)" -eq 0 ]; then die "Do not run install.sh as root / with sudo. Run it as your normal user; it calls sudo where needed."; fi

ARCH="$(uname -m)"
if [ "$ARCH" = "aarch64" ]; then ok "architecture $ARCH"; else warn "architecture $ARCH -- this repo targets Jetson aarch64; continuing anyway"; fi

UBUNTU_VERSION="$(. /etc/os-release 2>/dev/null && echo "${VERSION_ID}")"
if [ "$UBUNTU_VERSION" = "20.04" ]; then ok "Ubuntu $UBUNTU_VERSION"; else warn "Ubuntu '${UBUNTU_VERSION:-unknown}' -- ROS 2 ${ROS_DISTRO_NAME^} expects 20.04; continuing anyway"; fi

[ -d "${WS}/src" ] || die "${WS}/src not found -- is this really the ros2_ws checkout?"
ok "workspace layout looks right"

info "keeping sudo warm..."
sudo -v || die "sudo is required"

# --- 2. ROS 2 -------------------------------------------------------------
step "ROS 2 ${ROS_DISTRO_NAME^}"

if [ -f "$ROS_SETUP" ]; then
  ok "already installed at /opt/ros/${ROS_DISTRO_NAME}"
elif [ "$WITH_ROS" = "1" ]; then
  info "installing ROS 2 ${ROS_DISTRO_NAME^} from apt (this takes a while)"
  sudo apt-get update
  sudo apt-get install -y locales curl gnupg2 lsb-release software-properties-common
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  sudo add-apt-repository -y universe
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
  sudo apt-get update
  sudo apt-get install -y ros-${ROS_DISTRO_NAME}-desktop
  [ -f "$ROS_SETUP" ] || die "ROS 2 install finished but $ROS_SETUP is still missing"
  ok "ROS 2 ${ROS_DISTRO_NAME^} installed"
else
  die "$ROS_SETUP not found. Re-run with --with-ros to install ROS 2 ${ROS_DISTRO_NAME^} from apt."
fi

# --- 3. apt dependencies --------------------------------------------------
step "apt dependencies"

MISSING_APT=()
for pkg in "${APT_PACKAGES[@]}"; do
  apt_installed "$pkg" || MISSING_APT+=("$pkg")
done

if [ "${#MISSING_APT[@]}" -eq 0 ]; then
  ok "all ${#APT_PACKAGES[@]} packages already installed"
else
  info "installing ${#MISSING_APT[@]} missing package(s): ${MISSING_APT[*]}"
  sudo apt-get update
  # One package that is unavailable on this apt mirror must not abort the run,
  # so retry the failures individually and report them.
  if ! sudo apt-get install -y "${MISSING_APT[@]}"; then
    warn "batch install failed -- retrying package by package"
    for pkg in "${MISSING_APT[@]}"; do
      sudo apt-get install -y "$pkg" || warn "could not install $pkg (skipping)"
    done
  fi
  for pkg in "${MISSING_APT[@]}"; do
    apt_installed "$pkg" || warn "$pkg is still not installed"
  done
  ok "apt dependencies done"
fi

# --- 4. rosdep ------------------------------------------------------------
step "rosdep"

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init || warn "rosdep init failed (already initialized?)"
else
  ok "rosdep already initialized"
fi
rosdep update || warn "rosdep update failed (offline?) -- continuing"

info "resolving package dependencies from src/"
# shellcheck disable=SC2086
( cd "$WS" && rosdep install --from-paths src --ignore-src -r -y \
    --skip-keys "$ROSDEP_SKIP_KEYS" ) \
  || warn "rosdep install reported problems -- check the output above"
ok "rosdep done"

# --- 5. pip dependencies --------------------------------------------------
step "Python dependencies"

[ -f "$REQUIREMENTS" ] || die "$REQUIREMENTS not found"
python3 -m pip install --user --upgrade pip setuptools wheel || warn "could not upgrade pip -- continuing"
python3 -m pip install --user -r "$REQUIREMENTS" || warn "some pip packages failed to install -- see the import checks below"
ok "pip dependencies installed (from ${REQUIREMENTS#$WS/})"

for mod in "${PY_IMPORTS[@]}"; do
  py_import_ok "$mod" || warn "python3 -c 'import $mod' still fails"
done

# --- 6. udev --------------------------------------------------------------
step "udev rules (/dev/vesc, /dev/rplidar)"

[ -f "$UDEV_RULES_SRC" ] || die "$UDEV_RULES_SRC not found"
if cmp -s "$UDEV_RULES_SRC" "$UDEV_RULES_DST"; then
  ok "$UDEV_RULES_DST already up to date"
else
  sudo install -m 0644 -o root -g root "$UDEV_RULES_SRC" "$UDEV_RULES_DST"
  sudo udevadm control --reload-rules
  sudo udevadm trigger
  ok "installed $UDEV_RULES_DST and reloaded udev"
fi

# --- 7. spidev ------------------------------------------------------------
step "spidev kernel module (LED strip)"

if [ -f "$SPIDEV_CONF" ] && grep -qx 'spidev' "$SPIDEV_CONF"; then
  ok "$SPIDEV_CONF already configured"
else
  echo spidev | sudo tee "$SPIDEV_CONF" >/dev/null
  ok "wrote $SPIDEV_CONF"
fi

# --- 8. groups ------------------------------------------------------------
step "user groups"

GROUPS_ADDED=0
for grp in "${USER_GROUPS[@]}"; do
  if ! getent group "$grp" >/dev/null; then
    warn "group '$grp' does not exist on this system -- skipping"
    continue
  fi
  if in_group "$grp"; then
    ok "$RUN_USER already in $grp"
  else
    sudo usermod -a -G "$grp" "$RUN_USER"
    GROUPS_ADDED=1
    ok "added $RUN_USER to $grp"
  fi
done
if [ "$GROUPS_ADDED" = "1" ]; then warn "group changes need a reboot (or full re-login) to take effect"; fi

# --- 9. sudoers -----------------------------------------------------------
step "passwordless sudo for panel actions"

[ -f "$SUDOERS_SRC" ] || die "$SUDOERS_SRC not found"
SUDOERS_TMP="$(mktemp)"
trap 'rm -f "$SUDOERS_TMP"' EXIT
render_template "$SUDOERS_SRC" "$SUDOERS_TMP"
chmod 0440 "$SUDOERS_TMP"
if ! sudo visudo -c -f "$SUDOERS_TMP" >/dev/null; then
  sudo visudo -c -f "$SUDOERS_TMP" || true
  die "generated sudoers file is invalid -- refusing to install it"
fi
if sudo cmp -s "$SUDOERS_TMP" "$SUDOERS_DST" 2>/dev/null; then
  ok "$SUDOERS_DST already up to date"
else
  sudo install -m 0440 -o root -g root "$SUDOERS_TMP" "$SUDOERS_DST"
  ok "installed $SUDOERS_DST (validated with visudo)"
fi
rm -f "$SUDOERS_TMP"; trap - EXIT

# --- 10. ~/ros2_ws --------------------------------------------------------
step "~/ros2_ws path"

HOME_WS="${RUN_HOME}/ros2_ws"
if [ "$(readlink -f "$HOME_WS" 2>/dev/null)" = "$WS" ]; then
  ok "$HOME_WS already resolves to the workspace"
elif [ -e "$HOME_WS" ]; then
  warn "$HOME_WS exists but points at $(readlink -f "$HOME_WS") -- leaving it alone"
  warn "scripts and docs that assume ~/ros2_ws may use the wrong workspace"
else
  ln -s "$WS" "$HOME_WS"
  ok "created symlink $HOME_WS -> $WS"
fi

# --- 11. .bashrc ----------------------------------------------------------
step "shell setup (.bashrc)"

BASHRC="${RUN_HOME}/.bashrc"
add_bashrc_line() {
  local marker="$1" line="$2"
  if grep -Fq "$marker" "$BASHRC" 2>/dev/null; then
    ok "already in .bashrc: $marker"
  else
    printf '%s\n' "$line" >> "$BASHRC"
    ok "appended to .bashrc: $marker"
  fi
}
touch "$BASHRC"
add_bashrc_line "$ROS_SETUP"          "[ -f $ROS_SETUP ] && source $ROS_SETUP"
add_bashrc_line "${WS}/install/setup.bash" "[ -f ${WS}/install/setup.bash ] && source ${WS}/install/setup.bash"

# --- 12. colcon build -----------------------------------------------------
step "colcon build"

if [ "$NO_BUILD" = "1" ]; then
  info "--no-build given, skipping"
else
  BUILD_PACKAGES=("${CORE_PACKAGES[@]}")
  if [ "$FULL" = "1" ]; then BUILD_PACKAGES+=("${FULL_PACKAGES[@]}"); fi
  info "building: ${BUILD_PACKAGES[*]}"
  # Plain build (no --symlink-install) on purpose: --symlink-install symlinks
  # package data instead of copying it, which has bitten us with launch/config
  # files that setup.py installs via data_files. A plain build is slower to
  # iterate on but is what actually ends up on the car.
  (
    set +u
    # shellcheck disable=SC1090
    source "$ROS_SETUP"
    cd "$WS"
    colcon build --packages-select "${BUILD_PACKAGES[@]}" $(skip_args)
  ) || die "colcon build failed -- see the output above"
  ok "build finished"
fi

# --- 13. --full extras ----------------------------------------------------
if [ "$FULL" = "1" ]; then
  step "range_libc python wrapper (--full)"
  if [ ! -d "$RANGE_LIBC_PYWRAPPER" ]; then
    warn "$RANGE_LIBC_PYWRAPPER not found -- skipping"
  else
    info "building with: python3 setup.py install --user (in ${RANGE_LIBC_PYWRAPPER#$WS/})"
    # Known issue: upstream setup.py still uses Python 2 print statements, so
    # this can fail on Python 3. particle_filter falls back to a slower pure
    # Python ray caster, so a failure here is a warning, not an error.
    if ( cd "$RANGE_LIBC_PYWRAPPER" && python3 setup.py install --user ); then
      ok "range_libc python wrapper installed"
    else
      warn "range_libc wrapper failed to build (upstream setup.py is Python 2)"
      warn "particle_filter will fall back to its slow ray caster"
    fi
  fi
fi

# --- 14. optional systemd services ---------------------------------------
mkdir -p "${WS}/log"   # the units append their logs here; log/ is gitignored
if [ "$GPIO_SHUTDOWN" = "1" ]; then
  step "gpio-shutdown.service"
  TMP_UNIT="$(mktemp)"
  render_template "$GPIO_SERVICE_SRC" "$TMP_UNIT"
  sudo install -m 0644 -o root -g root "$TMP_UNIT" /etc/systemd/system/gpio-shutdown.service
  rm -f "$TMP_UNIT"
  sudo systemctl daemon-reload
  sudo systemctl enable gpio-shutdown.service
  ok "installed and enabled gpio-shutdown.service (starts on next boot)"
fi

if [ "$BT_PAD" = "1" ]; then
  step "bt_pad.service"
  TMP_UNIT="$(mktemp)"
  render_template "$BT_PAD_SERVICE_SRC" "$TMP_UNIT"
  sudo install -m 0644 -o root -g root "$TMP_UNIT" /etc/systemd/system/bt_pad.service
  rm -f "$TMP_UNIT"
  sudo systemctl daemon-reload
  sudo systemctl enable bt_pad.service
  ok "installed and enabled bt_pad.service (edit PAD_MAC in scripts/bt_pad_connect.sh for your pad)"
fi

if [ "$KEY_DRIVE_SERVICE" = "1" ]; then
  step "key_drive.service"
  printf '\n%s%s  !!! WARNING !!!%s\n' "$C_BOLD" "$C_RED" "$C_RESET"
  printf '%s  key_drive.service starts bringup AND keyboard driving automatically\n' "$C_YELLOW"
  printf '  on every boot. With a keyboard plugged into the car, the arrow keys\n'
  printf '  will make it DRIVE seconds after power-on, with no other confirmation.\n'
  printf '  Only enable this if the car is on a stand or you know what you are doing.%s\n\n' "$C_RESET"
  if confirm "Really install and enable key_drive.service?"; then
    TMP_UNIT="$(mktemp)"
    render_template "$KEY_DRIVE_SERVICE_SRC" "$TMP_UNIT"
    sudo install -m 0644 -o root -g root "$TMP_UNIT" /etc/systemd/system/key_drive.service
    rm -f "$TMP_UNIT"
    sudo systemctl daemon-reload
    sudo systemctl enable key_drive.service
    ok "installed and enabled key_drive.service"
    warn "disable it with: sudo systemctl disable --now key_drive.service"
  else
    info "skipped key_drive.service"
  fi
fi

# --- 15. desktop launcher -------------------------------------------------
if [ "$DESKTOP" = "1" ]; then
  step "desktop launcher"
  DESKTOP_DIR="${RUN_HOME}/Desktop"
  mkdir -p "$DESKTOP_DIR"
  render_template "$DESKTOP_SRC" "${DESKTOP_DIR}/ROS2-Panel.desktop"
  chmod +x "${DESKTOP_DIR}/ROS2-Panel.desktop"
  ok "wrote ${DESKTOP_DIR}/ROS2-Panel.desktop"
  info "your desktop may ask you to 'Allow Launching' the first time"
fi

# --- 16. summary ----------------------------------------------------------
printf '\n%s%s================ install complete ================%s\n' "$C_BOLD" "$C_GREEN" "$C_RESET"
cat <<EOF

Next steps
----------
1. REBOOT. Group membership (${USER_GROUPS[*]}) only takes
   effect after a full re-login:
       sudo reboot

2. Enable SPI for the LED strip (once, needs a reboot of its own):
       sudo /opt/nvidia/jetson-io/jetson-io.py
   -> Configure 40-pin expansion header -> enable "spi1"
      (SPI1 MOSI is pin 19 on the 40-pin header -- that is the LED data line)
   -> Save and reboot.

3. Plug in the VESC and the lidar, then confirm the udev symlinks:
       ls -l /dev/vesc /dev/rplidar

4. Verify the whole install at any time:
       ${WS}/install.sh --check

Running things
--------------
  Control panel (GTK):
       cd ${WS}/ros2_panel && python3 panel_app.py
  Bringup (lidar + VESC + joystick + mux):
       source ${WS}/install/setup.bash
       ros2 launch f1tenth_stack bringup_launch3.py
  Keyboard driving (builds + starts bringup itself, 'q' to quit):
       ${WS}/scripts/key_drive.sh
  Autonomous SAC driver:
       ros2 run sac_driver sac_driver_node --ros-args \\
           --params-file ${WS}/src/sac_driver/config/driver_params.yaml

EOF
if [ "$KEY_DRIVE_SERVICE" = "1" ]; then
  printf '%skey_drive.service is enabled: the car may drive on the next boot.%s\n\n' "$C_YELLOW" "$C_RESET"
fi
exit 0
