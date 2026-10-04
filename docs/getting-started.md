# Getting Started

## 1. Supported Host Model

The repository's <code>setup.sh</code> currently targets **Ubuntu**.

It expects ROS 2 to already exist under:

~~~text
/opt/ros/<distribution>
~~~

The script intentionally does **not** silently install an arbitrary ROS distribution because the correct distribution depends on the target Ubuntu release and deployment environment.

## 2. Clone

~~~bash
git clone https://github.com/alokrathava/RobotNavigation.git
cd RobotNavigation
~~~

If the repository is private, the machine needs valid GitHub credentials.

## 3. Run Setup

~~~bash
./setup.sh
~~~

If multiple ROS distributions are installed:

~~~bash
ROS_DISTRO=<your-distro> ./setup.sh
~~~

### What setup.sh does

The setup script is designed to be repeatable.

It:

1. validates Ubuntu;
2. prints OS, architecture, Python, memory, and disk information;
3. resolves the ROS distribution from <code>ROS_DISTRO</code> or <code>/opt/ros</code>;
4. installs missing build tools;
5. initializes/updates rosdep;
6. installs declared ROS dependencies;
7. creates <code>.venv</code> with <code>--system-site-packages</code>;
8. installs Python requirements;
9. verifies <code>rclpy</code> and <code>websockets</code>;
10. builds with <code>colcon build --symlink-install</code>;
11. verifies the five expected ROS packages;
12. verifies launch files and installed Python executables;
13. parses the top-level launch file;
14. creates runtime directories.

Runtime directories:

~~~text
~/.robot/maps
~/.robot/logs
~/.robot/run
~~~

Existing maps are not deleted.

## 4. First Simulation Run

~~~bash
./run.sh --backend sim --mode navigation --rviz
~~~

The launcher will:

- source ROS;
- source the Python environment;
- source the built workspace;
- verify packages;
- create runtime directories;
- start the stack in a dedicated process group;
- monitor real startup signals;
- report readiness.

Expected final message:

~~~text
[7/7] RobotNavigation READY
~~~

## 5. Status

From another terminal:

~~~bash
./run.sh --status
~~~

Typical information includes:

- RUNNING / STOPPED / DEGRADED;
- launch PID;
- process group;
- backend;
- mode;
- ROS distribution;
- map directory;
- gateway endpoint;
- log location;
- live ROS readiness/state information where available.

## 6. Stop

The normal stop mechanism is:

~~~text
Ctrl+C
~~~

<code>run.sh</code> sends SIGINT to the RobotNavigation process group first so ROS nodes can shut down cleanly.

It escalates to SIGTERM and finally SIGKILL only when processes refuse to exit.

## 7. Mapping Mode

~~~bash
./run.sh --backend sim --mode mapping --rviz
~~~

Mapping mode uses SLAM Toolbox rather than AMCL as the map→odom source.

## 8. Headless Mode

If no display is available, <code>run.sh</code> automatically uses headless Gazebo for simulation.

It can also be selected explicitly:

~~~bash
./run.sh --backend sim --headless
~~~

Do not request RViz without a graphical display.

## 9. Gateway Networking

### Local-only default

~~~bash
./run.sh --backend sim
~~~

Gateway:

~~~text
127.0.0.1:8080
~~~

### LAN access

~~~bash
export ROBOT_GATEWAY_TOKEN="strong-token"

./run.sh \
  --backend sim \
  --gateway-host 0.0.0.0
~~~

A non-loopback bind without a token is rejected.

## 10. Common Commands

~~~bash
./run.sh --backend sim --mode navigation --rviz
./run.sh --backend sim --mode navigation --headless
./run.sh --backend hardware --mode navigation
./run.sh --backend sim --no-gateway
./run.sh --maps-dir /data/robot/maps
./run.sh --status
./run.sh --clean
~~~

For deployment internals, see [Runtime & Deployment](runtime-and-deployment.md).
