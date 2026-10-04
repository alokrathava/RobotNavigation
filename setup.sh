#!/usr/bin/env bash
# Prepare a machine to run RobotNavigation: dependencies, Python venv, build, runtime dirs.
# Idempotent: safe to run repeatedly.
set -Eeuo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
cd "$REPO_ROOT"

STAGE="startup"
stage() { STAGE="$1"; printf '\n==> %s\n' "$1"; }
info()  { printf '    %s\n' "$*"; }
die()   { printf '\nERROR: %s\n' "$*" >&2; exit 1; }
trap 'rc=$?; printf "\nSETUP FAILED during stage: %s (line %s, exit %s)\n" "$STAGE" "$LINENO" "$rc" >&2; exit "$rc"' ERR

SUDO=""
if [[ $EUID -ne 0 ]]; then
    SUDO="sudo"
fi

# --- 1. environment -----------------------------------------------------------------------
stage "Detecting environment"
[[ -r /etc/os-release ]] || die "/etc/os-release not found; this script supports Ubuntu"
# shellcheck disable=SC1091
. /etc/os-release
[[ "${ID:-}" == "ubuntu" ]] || die "unsupported OS '${ID:-unknown}' (Ubuntu required)"
info "OS:            ${PRETTY_NAME:-Ubuntu}"
info "Ubuntu:        ${VERSION_ID:-?} (${VERSION_CODENAME:-?})"
info "Architecture:  $(uname -m)"
info "Python:        $(/usr/bin/python3 --version 2>&1)"
info "Repository:    $REPO_ROOT"
info "RAM available: $(awk '/MemAvailable/ {printf "%.1f GiB", $2/1048576}' /proc/meminfo)"
info "Disk free:     $(df -h --output=avail "$REPO_ROOT" | tail -1 | tr -d ' ')"

# --- 2. ROS distro ------------------------------------------------------------------------
stage "Selecting ROS 2 distribution"
if [[ -n "${ROS_DISTRO:-}" ]]; then
    [[ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]] \
        || die "ROS_DISTRO=$ROS_DISTRO but /opt/ros/$ROS_DISTRO/setup.bash does not exist"
else
    mapfile -t found < <(compgen -G '/opt/ros/*/setup.bash' | awk -F/ '{print $4}' | sort || true)
    case ${#found[@]} in
        0) die "no ROS 2 installation found in /opt/ros. Install ROS 2 first: https://docs.ros.org/" ;;
        1) ROS_DISTRO="${found[0]}" ;;
        *) die "multiple ROS distributions installed (${found[*]}) and ROS_DISTRO is unset. Re-run with: ROS_DISTRO=<one of them> ./setup.sh" ;;
    esac
fi
export ROS_DISTRO
info "ROS distro: $ROS_DISTRO"
set +u
# shellcheck disable=SC1090
source "/opt/ros/$ROS_DISTRO/setup.bash"
set -u

# --- 3. base tools ------------------------------------------------------------------------
stage "Checking base tools"
PACKAGES=(git curl python3 python3-pip python3-venv python3-dev build-essential cmake
          python3-rosdep python3-colcon-common-extensions iproute2)
missing=()
for pkg in "${PACKAGES[@]}"; do
    if ! dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null | grep -q "install ok installed"; then
        missing+=("$pkg")
    fi
done
if ((${#missing[@]})); then
    info "installing: ${missing[*]}"
    $SUDO apt-get update
    $SUDO apt-get install -y "${missing[@]}"
else
    info "all present"
fi

# --- 4. rosdep ----------------------------------------------------------------------------
stage "Resolving ROS dependencies (rosdep)"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    info "initialising rosdep"
    $SUDO rosdep init
fi
rosdep update --rosdistro "$ROS_DISTRO"
rosdep install --from-paths src --ignore-src -r -y --rosdistro "$ROS_DISTRO"
# -r keeps going past failures; make sure nothing is left unresolved.
rosdep check --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" \
    || die "rosdep could not satisfy all dependencies declared in package.xml files"

# --- 5. python environment ----------------------------------------------------------------
stage "Preparing Python environment (.venv)"
VENV="$REPO_ROOT/.venv"
SYS_PY="$(command -v /usr/bin/python3)"
recreate=""
if [[ ! -x "$VENV/bin/python" ]]; then
    recreate="missing"
elif ! grep -qiE '^include-system-site-packages *= *true' "$VENV/pyvenv.cfg" 2>/dev/null; then
    recreate="created without --system-site-packages (rclpy would be hidden)"
elif [[ "$("$VENV/bin/python" -c 'import sys;print(sys.version_info[:2])')" != "$("$SYS_PY" -c 'import sys;print(sys.version_info[:2])')" ]]; then
    recreate="python version differs from system python3"
fi
if [[ -n "$recreate" ]]; then
    info "creating .venv ($recreate)"
    rm -rf "$VENV"
    "$SYS_PY" -m venv --system-site-packages "$VENV"
    touch "$VENV/COLCON_IGNORE"
    fresh_venv=1
else
    info "reusing existing .venv"
    fresh_venv=0
fi
set +u
# shellcheck disable=SC1091
source "$VENV/bin/activate"
set -u
if ((fresh_venv)); then
    python -m pip install --upgrade pip
fi
python -m pip install -r requirements.txt
python - <<'PY' || die "Python environment verification failed"
import re, sys
import rclpy  # noqa: F401
import websockets
try:
    from packaging.requirements import Requirement
except ImportError:
    from pip._vendor.packaging.requirements import Requirement
from importlib.metadata import version
installed = version("websockets")
for line in open("requirements.txt"):
    line = line.split("#", 1)[0].strip()
    if not line:
        continue
    req = Requirement(line)
    if req.name.lower() == "websockets" and not req.specifier.contains(installed, prereleases=True):
        sys.exit(f"websockets {installed} does not satisfy '{line}'")
print(f"    import rclpy, websockets OK (websockets {installed})")
PY

# --- 6. build -----------------------------------------------------------------------------
stage "Building workspace (colcon)"
STAMP_FILE="$REPO_ROOT/build/.robot_setup_stamp"
STAMP="distro=$ROS_DISTRO arch=$(uname -m) python=$VENV/bin/python repo=$REPO_ROOT"
if [[ -d build || -d install ]]; then
    if [[ ! -f "$STAMP_FILE" ]]; then
        info "cleaning stale build state (no setup stamp)"
        rm -rf build install
    elif [[ "$(cat "$STAMP_FILE")" != "$STAMP" ]]; then
        info "cleaning stale build state (distro/arch/python/location changed)"
        rm -rf build install
    fi
fi
python -m colcon build --symlink-install --base-paths src
mkdir -p build
printf '%s\n' "$STAMP" > "$STAMP_FILE"

# --- 7. validation ------------------------------------------------------------------------
stage "Validating installation"
set +u
# shellcheck disable=SC1091
source "$REPO_ROOT/install/setup.bash"
set -u
for pkg in robot_bringup robot_description robot_interfaces robot_navigation robot_gateway; do
    prefix="$(ros2 pkg prefix "$pkg")" || die "package $pkg not found after build"
    info "$pkg -> $prefix"
done
for f in "$(ros2 pkg prefix robot_bringup)/share/robot_bringup/launch/robot.launch.py" \
         "$(ros2 pkg prefix robot_navigation)/share/robot_navigation/launch/navigation.launch.py"; do
    [[ -f "$f" ]] || die "launch file missing: $f"
done
info "launch files installed"
nav_lib="$(ros2 pkg prefix robot_navigation)/lib/robot_navigation"
for script in "$nav_lib"/*.py; do
    [[ -x "$script" ]] || die "not executable: $script"
    python -m py_compile "$script" || die "does not compile: $script"
done
info "robot_navigation scripts executable and compile"
gateway_py="$(head -1 "$(ros2 pkg prefix robot_gateway)/lib/robot_gateway/gateway_node" | sed 's/^#! *//')"
"$gateway_py" -c 'import robot_gateway.gateway_node' \
    || die "gateway_node cannot be imported with its interpreter ($gateway_py)"
info "gateway_node importable ($gateway_py)"
ros2 launch --show-args robot_bringup robot.launch.py >/dev/null \
    || die "robot.launch.py does not parse"
info "robot.launch.py parses"

# --- 8. runtime directories ---------------------------------------------------------------
stage "Creating runtime directories"
mkdir -p "$HOME/.robot/maps" "$HOME/.robot/logs" "$HOME/.robot/run"
chmod 700 "$HOME/.robot/run"
[[ -w "$HOME/.robot/maps" ]] || die "$HOME/.robot/maps is not writable"
info "$HOME/.robot/{maps,logs,run} ready (existing maps untouched)"

STAGE="done"
cat <<EOF

RobotNavigation setup complete

ROS distro:         $ROS_DISTRO
Architecture:       $(uname -m)
Workspace:          $REPO_ROOT
Python environment: $VENV
Maps:               ~/.robot/maps
Build:              SUCCESS

Run simulation:
  ./run.sh --backend sim

Run hardware:
  ./run.sh --backend hardware
EOF
