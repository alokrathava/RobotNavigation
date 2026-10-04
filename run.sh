#!/usr/bin/env bash
# Start the RobotNavigation stack (ros2 launch robot_bringup robot.launch.py), wait for real
# readiness signals, and shut down only the process group this script created.
set -Eeuo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
cd "$REPO_ROOT"

RUN_DIR="$HOME/.robot/run"
LOG_DIR="$HOME/.robot/logs"
PID_FILE="$RUN_DIR/robot_navigation.pid"
LOCK_FILE="$RUN_DIR/robot_navigation.lock"

STARTUP_TIMEOUT="${ROBOT_STARTUP_TIMEOUT:-240}"
INT_WAIT="${ROBOT_SHUTDOWN_INT_WAIT:-15}"
TERM_WAIT="${ROBOT_SHUTDOWN_TERM_WAIT:-5}"

BACKEND="sim"
MODE="navigation"
RVIZ="false"
HEADLESS=0
GATEWAY="true"
GATEWAY_HOST="127.0.0.1"
GATEWAY_PORT="8080"
GATEWAY_TOKEN="${ROBOT_GATEWAY_TOKEN:-}"
MAPS_DIR="$HOME/.robot/maps"
CLEAN=0
STATUS=0

usage() {
    cat <<'EOF'
Usage: ./run.sh [options]

  --backend sim|hardware   robot backend (default: sim)
  --mode navigation|mapping
                           navigation: localize on the saved map; mapping: SLAM (default: navigation)
  --rviz | --no-rviz       start RViz (default: off)
  --headless               run Gazebo without its GUI (sim only)
  --gateway | --no-gateway WebSocket gateway (default: on)
  --gateway-host HOST      gateway bind address (default: 127.0.0.1)
  --gateway-port PORT      gateway port (default: 8080)
  --gateway-token TOKEN    gateway auth token; required when the host is not loopback. The flag
                           is visible in `ps`: prefer `export ROBOT_GATEWAY_TOKEN=...`
  --maps-dir PATH          runtime map root (default: ~/.robot/maps)
  --clean                  stop a previous RobotNavigation run owned by this script first
  --status                 show whether RobotNavigation is running
  --help                   show this help

Environment: ROBOT_STARTUP_TIMEOUT (seconds, default 240)
EOF
}

die() { printf '\nERROR: %s\n' "$*" >&2; exit "${DIE_CODE:-1}"; }
info() { printf '%s\n' "$*"; }

need_arg() { [[ $# -ge 2 && -n "$2" ]] || die "option $1 requires a value"; }

while [[ $# -gt 0 ]]; do
    case "$1" in
        --backend)       need_arg "$@"; BACKEND="$2"; shift 2 ;;
        --mode)          need_arg "$@"; MODE="$2"; shift 2 ;;
        --rviz)          RVIZ="true"; shift ;;
        --no-rviz)       RVIZ="false"; shift ;;
        --headless)      HEADLESS=1; shift ;;
        --gateway)       GATEWAY="true"; shift ;;
        --no-gateway)    GATEWAY="false"; shift ;;
        --gateway-host)  need_arg "$@"; GATEWAY_HOST="$2"; shift 2 ;;
        --gateway-port)  need_arg "$@"; GATEWAY_PORT="$2"; shift 2 ;;
        --gateway-token) need_arg "$@"; GATEWAY_TOKEN="$2"; shift 2 ;;
        --maps-dir)      need_arg "$@"; MAPS_DIR="$2"; shift 2 ;;
        --clean)         CLEAN=1; shift ;;
        --status)        STATUS=1; shift ;;
        -h|--help)       usage; exit 0 ;;
        *) usage >&2; die "unknown option: $1" ;;
    esac
done

# ---------------------------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------------------------
detect_ros_distro() {
    if [[ -n "${ROS_DISTRO:-}" ]]; then
        [[ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]] \
            || die "ROS_DISTRO=$ROS_DISTRO but /opt/ros/$ROS_DISTRO/setup.bash does not exist"
        return
    fi
    local found=()
    mapfile -t found < <(compgen -G '/opt/ros/*/setup.bash' | awk -F/ '{print $4}' | sort || true)
    case ${#found[@]} in
        0) die "no ROS 2 installation found in /opt/ros (run ./setup.sh after installing ROS 2)" ;;
        1) ROS_DISTRO="${found[0]}" ;;
        *) die "multiple ROS distributions installed (${found[*]}) and ROS_DISTRO is unset. Use: ROS_DISTRO=<one> ./run.sh" ;;
    esac
    export ROS_DISTRO
}

source_ros() {
    set +u
    # shellcheck disable=SC1090
    source "/opt/ros/$ROS_DISTRO/setup.bash"
    set -u
}

source_workspace() {
    [[ -f "$REPO_ROOT/install/setup.bash" ]] || die "workspace is not built (install/setup.bash missing). Run ./setup.sh first"
    if [[ -f "$REPO_ROOT/.venv/bin/activate" ]]; then
        set +u
        # shellcheck disable=SC1091
        source "$REPO_ROOT/.venv/bin/activate"
        set -u
    fi
    set +u
    # shellcheck disable=SC1091
    source "$REPO_ROOT/install/setup.bash"
    set -u
}

is_loopback() {
    case "$1" in
        localhost|::1|127.*) return 0 ;;
        *) return 1 ;;
    esac
}

# starttime (clock ticks since boot) of a pid: stable identity guard against PID reuse.
proc_starttime() {
    local stat
    stat="$(cat "/proc/$1/stat" 2>/dev/null)" || return 1
    stat="${stat##*) }"
    awk '{print $20}' <<<"$stat"
}

proc_is_zombie() {
    local stat
    stat="$(cat "/proc/$1/stat" 2>/dev/null)" || return 0
    stat="${stat##*) }"
    [[ "${stat%% *}" == "Z" ]]
}

pf_get() { sed -n "s/^$1=//p" "$PID_FILE" 2>/dev/null | head -1; }

# Live, non-zombie members of a process group.
group_pids() {
    ps -eo pid=,pgid=,stat= | awk -v g="$1" '$2==g && $3 !~ /^Z/ {print $1}'
}

# Members of a process group that carry this run's id in their environment.
group_owned_pids() {
    local pgid="$1" runid="$2" p
    [[ -n "$runid" ]] || return 0
    for p in $(group_pids "$pgid"); do
        if tr '\0' '\n' <"/proc/$p/environ" 2>/dev/null | grep -qx "ROBOT_NAVIGATION_RUN_ID=$runid"; then
            echo "$p"
        fi
    done
}

# Is the launch recorded in the PID file still ours and alive?
pidfile_leader_alive() {
    local pid st cur
    pid="$(pf_get pid)"; st="$(pf_get starttime)"
    [[ -n "$pid" && -n "$st" && -d "/proc/$pid" ]] || return 1
    cur="$(proc_starttime "$pid")" || return 1
    [[ "$cur" == "$st" ]] || return 1
    proc_is_zombie "$pid" && return 1
    tr '\0' ' ' <"/proc/$pid/cmdline" 2>/dev/null | grep -q 'robot.launch.py'
}

# Stop a process group: INT (graceful, like Ctrl+C), then TERM, then KILL.
stop_group() {
    local pgid="$1" waited
    [[ -n "$pgid" && "$pgid" -gt 1 ]] || return 0
    [[ -n "$(group_pids "$pgid")" ]] || return 0
    kill -INT -- "-$pgid" 2>/dev/null || true
    for ((waited = 0; waited < INT_WAIT; waited++)); do
        [[ -z "$(group_pids "$pgid")" ]] && return 0
        sleep 1
    done
    info "    still running after ${INT_WAIT}s; sending SIGTERM"
    kill -TERM -- "-$pgid" 2>/dev/null || true
    for ((waited = 0; waited < TERM_WAIT; waited++)); do
        [[ -z "$(group_pids "$pgid")" ]] && return 0
        sleep 1
    done
    info "    still running after SIGTERM; sending SIGKILL"
    kill -KILL -- "-$pgid" 2>/dev/null || true
    sleep 1
}

# ---------------------------------------------------------------------------------------------
# probe: one rclpy process that watches real ROS signals (topics, services, actions, TCP port)
# ---------------------------------------------------------------------------------------------
read -r -d '' PROBE_PY <<'PY' || true
import argparse
import os
import re
import socket
import subprocess
import sys
import time

import rclpy
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from robot_interfaces.msg import MapState
from robot_interfaces.srv import StartMapping
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage

STATE_NAMES = {0: 'NO_MAP', 1: 'LOADING', 2: 'ACTIVE', 3: 'MAPPING', 4: 'SAVING',
               5: 'SWITCHING', 6: 'ERROR'}


def say(text):
    print(text, flush=True)


class Probe(Node):
    def __init__(self):
        super().__init__('run_sh_probe')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        live = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                          reliability=ReliabilityPolicy.BEST_EFFORT)
        self.seen = set()
        self.map_state = None
        self.nav_ready = False
        self.map_stamp = 0
        self.create_subscription(MapState, '/robot/map/state', self.on_state, latched)
        self.create_subscription(Bool, '/robot/navigation/runtime_ready', self.on_ready, latched)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        for name, typ in (('/clock', Clock), ('/odom', Odometry), ('/scan', LaserScan),
                          ('/tf', TFMessage)):
            self.create_subscription(typ, name, lambda _m, n=name: self.seen.add(n), live)
        self.start_client = self.create_client(StartMapping, '/robot/maps/start_mapping')
        self.nav_actions = {
            '/navigate_to_pose': ActionClient(self, NavigateToPose, '/navigate_to_pose'),
            '/navigate_through_poses': ActionClient(
                self, NavigateThroughPoses, '/navigate_through_poses'),
        }

    def on_state(self, msg):
        self.map_state = msg

    def on_ready(self, msg):
        self.nav_ready = bool(msg.data)

    def on_map(self, _msg):
        self.map_stamp += 1
        self.seen.add('/map')

    def spin(self, seconds=0.2):
        rclpy.spin_once(self, timeout_sec=seconds)


def leader_gone(pid):
    try:
        with open(f'/proc/{pid}/stat') as stream:
            state = stream.read().rsplit(') ', 1)[1].split()[0]
        return state == 'Z'
    except OSError:
        return True


def wait_for(probe, predicate, deadline, leader):
    while time.monotonic() < deadline:
        probe.spin()
        if leader and leader_gone(leader):
            raise RuntimeError('the robot launch process exited during startup')
        if predicate():
            return True
    return False


def gateway_listening(host, port, pgid):
    try:
        out = subprocess.run(['ss', '-ltnpH', f'sport = :{port}'], capture_output=True,
                             text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        out = None
    if out is not None:
        pids = [int(p) for p in re.findall(r'pid=(\d+)', out)]
        if not pids:
            return False
        if pgid:
            return any(os.getpgid(p) == pgid for p in pids if os.path.exists(f'/proc/{p}'))
        return True
    try:
        with socket.create_connection(('127.0.0.1', port), timeout=1):
            return True
    except OSError:
        return False


def status(probe):
    deadline = time.monotonic() + 8.0
    wait_for(probe, lambda: probe.map_state is not None, deadline, 0)
    # The ready topic is latched; give it a moment after the state arrives.
    wait_for(probe, lambda: False, min(deadline, time.monotonic() + 1.0), 0)
    if probe.map_state is None:
        say('Map subsystem: unknown (no /robot/map/state)')
    else:
        m = probe.map_state
        extra = f' (map {m.active_map_id})' if m.active_map_id else ''
        say(f'Map subsystem: {STATE_NAMES.get(m.state, m.state)}{extra}')
    say('Navigation runtime: ' + ('READY' if probe.nav_ready else 'NOT READY'))


def startup(probe, args):
    t0 = time.monotonic()
    overall = t0 + args.timeout
    leader = args.leader

    # [3/7] backend interfaces
    required = ['/odom', '/scan', '/tf'] + (['/clock'] if args.backend == 'sim' else [])
    backend_deadline = overall if args.backend == 'sim' else min(overall, t0 + 60)
    say(f'[3/7] {"Simulator" if args.backend == "sim" else "Hardware"} backend starting '
        f'(waiting for {" ".join(required)})')
    if not wait_for(probe, lambda: all(n in probe.seen for n in required),
                    backend_deadline, leader):
        missing = [n for n in required if n not in probe.seen]
        raise RuntimeError(f'backend interfaces never appeared: {" ".join(missing)}'
                           + ('. The hardware drivers must publish them.'
                              if args.backend == 'hardware' else ''))
    say('      backend interfaces up')

    # [4/7] map subsystem
    say('[4/7] Waiting for map subsystem (/robot/map/state)')

    def map_ready():
        m = probe.map_state
        if m is not None and m.state == MapState.ERROR:
            raise RuntimeError(f'map subsystem error {m.error_code}: {m.detail}')
        return m is not None and m.state == MapState.ACTIVE and m.map_available

    if not wait_for(probe, map_ready, overall, leader):
        m = probe.map_state
        raise RuntimeError('map subsystem not active in time'
                           + (f' (state {STATE_NAMES.get(m.state)}: {m.detail})' if m else
                              ' (no /robot/map/state published)'))
    if args.mode == 'navigation':
        if not wait_for(probe, lambda: '/map' in probe.seen, overall, leader):
            raise RuntimeError('/map was never published')
    else:
        say('      starting mapping via /robot/maps/start_mapping')
        if not probe.start_client.wait_for_service(timeout_sec=max(1.0, overall - time.monotonic())):
            raise RuntimeError('/robot/maps/start_mapping service is not available')
        probe.map_stamp = 0
        future = probe.start_client.call_async(StartMapping.Request())
        if not wait_for(probe, future.done, overall, leader):
            raise RuntimeError('start_mapping did not return in time')
        response = future.result()
        if not response.success:
            raise RuntimeError(f'start_mapping failed: {response.error_code}: {response.message}')
        ok = wait_for(probe, lambda: probe.map_state is not None
                      and probe.map_state.state == MapState.MAPPING and probe.map_stamp > 0,
                      overall, leader)
        if not ok:
            raise RuntimeError('mapping did not publish a /map in time')
    say('      map subsystem ready')

    # [5/7] navigation runtime
    if args.mode == 'navigation':
        say('[5/7] Waiting for navigation runtime (runtime_ready, action servers)')

        def nav_up():
            return (probe.nav_ready
                    and all(c.server_is_ready() for c in probe.nav_actions.values())
                    and probe.count_publishers('/cmd_vel') > 0)

        if not wait_for(probe, nav_up, overall, leader):
            detail = [f'runtime_ready={probe.nav_ready}']
            detail += [f'{n}={"up" if c.server_is_ready() else "down"}'
                       for n, c in probe.nav_actions.items()]
            detail.append(f'cmd_vel_publishers={probe.count_publishers("/cmd_vel")}')
            raise RuntimeError('navigation runtime not ready: ' + ' '.join(detail))
    else:
        say('[5/7] Mapping mode: navigation runtime intentionally paused; checking /cmd_vel')
        if not wait_for(probe, lambda: probe.count_publishers('/cmd_vel') > 0, overall, leader):
            raise RuntimeError('no /cmd_vel publisher (safety supervisor missing)')
    say('      runtime ready')

    # [6/7] gateway
    if args.gateway_port:
        say(f'[6/7] Waiting for WebSocket gateway on port {args.gateway_port}')
        if not wait_for(probe, lambda: gateway_listening(args.gateway_host, args.gateway_port,
                                                         args.pgid), overall, leader):
            raise RuntimeError(f'gateway is not listening on port {args.gateway_port}')
        say('      gateway listening')
    else:
        say('[6/7] WebSocket gateway disabled')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('command', choices=['startup', 'status'])
    parser.add_argument('--backend', default='sim')
    parser.add_argument('--mode', default='navigation')
    parser.add_argument('--timeout', type=float, default=240.0)
    parser.add_argument('--leader', type=int, default=0)
    parser.add_argument('--pgid', type=int, default=0)
    parser.add_argument('--gateway-host', default='127.0.0.1')
    parser.add_argument('--gateway-port', type=int, default=0)
    args = parser.parse_args()

    rclpy.init()
    probe = Probe()
    code = 0
    try:
        if args.command == 'status':
            status(probe)
        else:
            startup(probe, args)
    except RuntimeError as exc:
        say(f'STARTUP FAILED: {exc}')
        code = 1
    except KeyboardInterrupt:
        code = 130
    finally:
        probe.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


main()
PY

# ---------------------------------------------------------------------------------------------
# --status
# ---------------------------------------------------------------------------------------------
if ((STATUS)); then
    if [[ ! -f "$PID_FILE" ]]; then
        info "RobotNavigation: STOPPED"
        exit 3
    fi
    if ! pidfile_leader_alive; then
        runid="$(pf_get run_id)"; pgid="$(pf_get pgid)"
        if [[ -n "$pgid" && -n "$(group_owned_pids "$pgid" "$runid")" ]]; then
            info "RobotNavigation: DEGRADED (launch process gone, orphaned processes remain in group $pgid; use ./run.sh --clean)"
        else
            info "RobotNavigation: STOPPED (stale PID file; the next ./run.sh removes it)"
        fi
        exit 3
    fi
    distro="$(pf_get distro)"
    ghost="$(pf_get gateway_host)"; gport="$(pf_get gateway_port)"
    info "RobotNavigation: RUNNING"
    info "PID: $(pf_get pid)"
    info "Process group: $(pf_get pgid)"
    info "Backend: $(pf_get backend)"
    info "Mode: $(pf_get mode)"
    info "ROS distro: $distro"
    info "Maps directory: $(pf_get maps_dir)"
    if [[ -z "$gport" ]]; then
        info "Gateway: disabled"
    else
        shown="$ghost"
        if [[ "$ghost" == "0.0.0.0" || "$ghost" == "::" ]]; then
            shown="$(hostname -I 2>/dev/null | awk '{print $1}')"
            shown="${shown:-$ghost}"
        fi
        info "Gateway: $shown:$gport"
    fi
    info "Log: $(pf_get log)"
    if [[ -n "$distro" && -f "/opt/ros/$distro/setup.bash" && -f "$REPO_ROOT/install/setup.bash" ]]; then
        ROS_DISTRO="$distro"; source_ros
        set +u
        # shellcheck disable=SC1091
        source "$REPO_ROOT/install/setup.bash"
        set -u
        cfg="$(ros2 pkg prefix robot_navigation 2>/dev/null)/share/robot_navigation/config/fastdds_no_shm.xml"
        [[ -f "$cfg" ]] && export FASTDDS_DEFAULT_PROFILES_FILE="$cfg"
        /usr/bin/python3 -c "$PROBE_PY" status || true
    fi
    exit 0
fi

# ---------------------------------------------------------------------------------------------
# validation (before touching anything)
# ---------------------------------------------------------------------------------------------
[[ "$BACKEND" == "sim" || "$BACKEND" == "hardware" ]] || die "--backend must be sim or hardware"
[[ "$MODE" == "navigation" || "$MODE" == "mapping" ]] || die "--mode must be navigation or mapping"
[[ "$GATEWAY_PORT" =~ ^[0-9]+$ && "$GATEWAY_PORT" -ge 1 && "$GATEWAY_PORT" -le 65535 ]] \
    || die "--gateway-port must be 1-65535"
[[ "$STARTUP_TIMEOUT" =~ ^[0-9]+$ ]] || die "ROBOT_STARTUP_TIMEOUT must be an integer"
MAPS_DIR="${MAPS_DIR/#\~/$HOME}"
[[ "$MAPS_DIR" == /* ]] || MAPS_DIR="$PWD/$MAPS_DIR"

if [[ "$GATEWAY" == "true" ]]; then
    if ! is_loopback "$GATEWAY_HOST" && [[ -z "$GATEWAY_TOKEN" ]]; then
        DIE_CODE=2 die "Gateway is being exposed to the network but no authentication token was provided.

Use:
./run.sh --gateway-host $GATEWAY_HOST --gateway-token <token>
(or export ROBOT_GATEWAY_TOKEN=<token>)"
    fi
else
    GATEWAY_TOKEN=""
fi

if [[ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
    if [[ "$RVIZ" == "true" ]]; then
        die "--rviz needs a graphical display (DISPLAY/WAYLAND_DISPLAY is not set)"
    fi
    if [[ "$BACKEND" == "sim" && $HEADLESS -eq 0 ]]; then
        info "No display detected: running Gazebo headless"
        HEADLESS=1
    fi
fi

# ---------------------------------------------------------------------------------------------
# preflight
# ---------------------------------------------------------------------------------------------
detect_ros_distro
source_ros
source_workspace
for pkg in robot_bringup robot_description robot_interfaces robot_navigation robot_gateway; do
    ros2 pkg prefix "$pkg" >/dev/null 2>&1 || die "ROS package '$pkg' not found. Run ./setup.sh first"
done
command -v ss >/dev/null || die "'ss' not found (install iproute2; ./setup.sh does this)"

mkdir -p "$MAPS_DIR" "$LOG_DIR" "$RUN_DIR"
chmod 700 "$RUN_DIR"
[[ -w "$MAPS_DIR" ]] || die "maps directory is not writable: $MAPS_DIR"

NAV_CONFIG="$(ros2 pkg prefix robot_navigation)/share/robot_navigation/config/fastdds_no_shm.xml"
[[ -f "$NAV_CONFIG" ]] && export FASTDDS_DEFAULT_PROFILES_FILE="$NAV_CONFIG"

# A previous run owned by this script?
if [[ -f "$PID_FILE" ]]; then
    prev_pgid="$(pf_get pgid)"; prev_runid="$(pf_get run_id)"
    if pidfile_leader_alive; then
        if ((CLEAN)); then
            info "Stopping previous RobotNavigation run (pgid $prev_pgid)..."
            stop_group "$prev_pgid"
        else
            die "RobotNavigation is already running (PID $(pf_get pid)). Use ./run.sh --status, or --clean to restart"
        fi
    elif [[ -n "$prev_pgid" && -n "$(group_owned_pids "$prev_pgid" "$prev_runid")" ]]; then
        if ((CLEAN)); then
            info "Stopping orphaned processes from a previous run (pgid $prev_pgid)..."
            stop_group "$prev_pgid"
        else
            die "orphaned processes from a previous RobotNavigation run remain in process group $prev_pgid. Use ./run.sh --clean"
        fi
    fi
    if [[ -z "$prev_pgid" || -z "$(group_owned_pids "$prev_pgid" "$prev_runid")" ]] \
        && ! pidfile_leader_alive; then
        info "Removing stale PID file"
        rm -f "$PID_FILE"
    else
        die "previous RobotNavigation processes could not be stopped"
    fi
fi

# Only one supervisor at a time (the lock is not inherited by the launched stack).
exec 9>"$LOCK_FILE"
if ((CLEAN)); then
    flock -w 30 9 || die "another run.sh is still shutting down"
else
    flock -n 9 || die "another run.sh is already starting or running (see ./run.sh --status)"
fi

if [[ "$GATEWAY" == "true" ]]; then
    if [[ -n "$(ss -ltnH "sport = :$GATEWAY_PORT")" ]]; then
        owner="$(ss -ltnpH "sport = :$GATEWAY_PORT" | grep -o 'users:(.*)' | head -1 || true)"
        die "gateway port $GATEWAY_PORT is already in use${owner:+ ($owner)}. Choose another with --gateway-port"
    fi
fi

# ---------------------------------------------------------------------------------------------
# start
# ---------------------------------------------------------------------------------------------
LOG="$LOG_DIR/robot-navigation-$(date +%Y%m%d-%H%M%S).log"
RUN_ID="$(date +%s)-$$-$RANDOM"
LAUNCH_PID=""
LAUNCH_PGID=""
TAIL_PID=""
CLEANED=0
EXIT_CODE=0

cleanup() {
    ((CLEANED)) && return
    CLEANED=1
    set +e
    trap '' INT TERM
    if [[ -n "$LAUNCH_PGID" ]]; then
        info ""
        info "Shutting down RobotNavigation (process group $LAUNCH_PGID)..."
        stop_group "$LAUNCH_PGID"
        wait "$LAUNCH_PID" 2>/dev/null
        info "RobotNavigation stopped (maps untouched)"
    fi
    [[ -n "$TAIL_PID" ]] && kill "$TAIL_PID" 2>/dev/null
    if [[ -f "$PID_FILE" && "$(pf_get run_id)" == "$RUN_ID" ]]; then
        rm -f "$PID_FILE"
    fi
}
on_signal() { EXIT_CODE="$1"; exit "$1"; }
trap 'on_signal 130' INT
trap 'on_signal 143' TERM
trap 'rc=$?; ((EXIT_CODE)) && rc=$EXIT_CODE; cleanup; exit $rc' EXIT

info "[1/7] Environment ready"
info "      ROS distro:  $ROS_DISTRO"
info "      Backend:     $BACKEND   Mode: $MODE"
info "      Maps dir:    $MAPS_DIR"
if [[ "$GATEWAY" == "true" ]]; then
    info "      Gateway:     $GATEWAY_HOST:$GATEWAY_PORT"
    if [[ -n "$GATEWAY_TOKEN" ]]; then info "      Gateway token: configured"; else info "      Gateway token: none (loopback only)"; fi
else
    info "      Gateway:     disabled"
fi
info "      Log:         $LOG"

launch_args=(backend:="$BACKEND" mode:="$MODE" maps_dir:="$MAPS_DIR" rviz:="$RVIZ"
             gateway:="$GATEWAY" gateway_host:="$GATEWAY_HOST" gateway_port:="$GATEWAY_PORT")
if ((HEADLESS)) && [[ "$BACKEND" == "sim" ]]; then launch_args+=(headless:=-s); fi

: >"$LOG"
chmod 600 "$LOG"
if [[ -n "$GATEWAY_TOKEN" ]]; then export ROBOT_GATEWAY_TOKEN="$GATEWAY_TOKEN"; else unset ROBOT_GATEWAY_TOKEN; fi
export ROBOT_NAVIGATION_RUN_ID="$RUN_ID"

# Own session + process group: $! is both the PID and the PGID of the whole stack.
setsid ros2 launch robot_bringup robot.launch.py "${launch_args[@]}" \
    </dev/null >>"$LOG" 2>&1 9>&- &
LAUNCH_PID=$!
sleep 0.2
LAUNCH_PGID="$(ps -o pgid= -p "$LAUNCH_PID" 2>/dev/null | tr -d ' ')"
if [[ "$LAUNCH_PGID" != "$LAUNCH_PID" ]]; then
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    LAUNCH_PGID=""
    die "could not start the launch in its own process group"
fi
unset ROBOT_NAVIGATION_RUN_ID

{
    echo "pid=$LAUNCH_PID"
    echo "pgid=$LAUNCH_PGID"
    echo "starttime=$(proc_starttime "$LAUNCH_PID")"
    echo "supervisor_pid=$$"
    echo "run_id=$RUN_ID"
    echo "backend=$BACKEND"
    echo "mode=$MODE"
    echo "maps_dir=$MAPS_DIR"
    echo "gateway_host=$([[ "$GATEWAY" == "true" ]] && echo "$GATEWAY_HOST")"
    echo "gateway_port=$([[ "$GATEWAY" == "true" ]] && echo "$GATEWAY_PORT")"
    echo "distro=$ROS_DISTRO"
    echo "log=$LOG"
} >"$PID_FILE.tmp"
chmod 600 "$PID_FILE.tmp"
mv "$PID_FILE.tmp" "$PID_FILE"
info "[2/7] Robot process started (pid $LAUNCH_PID)"

# Mirror important log lines to the terminal; the full output stays in the log.
( tail -n 0 -F --pid=$$ "$LOG" 2>/dev/null \
    | grep --line-buffered -E '\[(ERROR|FATAL)\]|process has died|Traceback| (ERROR|CRITICAL) |Robot Gateway listening' ) &
TAIL_PID=$!

probe_gateway_port=0
[[ "$GATEWAY" == "true" ]] && probe_gateway_port="$GATEWAY_PORT"
/usr/bin/env python3 -c "$PROBE_PY" startup \
    --backend "$BACKEND" --mode "$MODE" --timeout "$STARTUP_TIMEOUT" \
    --leader "$LAUNCH_PID" --pgid "$LAUNCH_PGID" \
    --gateway-host "$GATEWAY_HOST" --gateway-port "$probe_gateway_port" &
PROBE_PID=$!
if ! wait "$PROBE_PID"; then
    echo "" >&2
    echo "Startup failed. Last log lines ($LOG):" >&2
    tail -n 25 "$LOG" >&2 || true
    EXIT_CODE=1
    exit 1
fi

info "[7/7] RobotNavigation READY"
info "      Stop with Ctrl+C"

# Supervise until the stack exits or we are signalled.
set +e
wait "$LAUNCH_PID"
rc=$?
set -e
if ((rc != 0)); then
    echo "" >&2
    echo "ros2 launch exited with status $rc (see $LOG)" >&2
    EXIT_CODE=$rc
fi
exit "$rc"
