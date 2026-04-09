# UR10 & Basler Camera Integrated ROS Workspace: Local Installation

This guide explains how to install the `ur_modern_driver` and `pylon_trigger` packages directly onto an Ubuntu 20.04 LTS host running ROS 1 Noetic without using Docker.

## Differences from Docker setup

The Docker environment relies on Ubuntu 18.04 and ROS 1 Melodic. This local setup runs on the host OS natively, which leverages **Ubuntu 20.04 LTS**, **ROS 1 Noetic**, and **Python 3**.

A shell script `install_ros1_local.sh` and virtual environment setup automate the differences and build operations natively without affecting the repository structure. This allows both the native testing on the host machine and deployment via the standard `Dockerfile`.

## Automated Installation

You can automatically perform the entire installation, virtual environment setup, and dependency building by running:

```bash
cd /path/to/UR10-control
./install_ros1_local.sh
```

### What does the script do?
1. Configures the Ubuntu `apt` repositories for ROS 1 Noetic and installs base system dependencies.
2. Initializes `rosdep` to manage ROS package requirements.
3. Builds an isolated Catkin Workspace at `local_ws`.
4. Symlinks the current repository and clones the necessary ROS-industrial packages to the newly created Catkin Workspace.
5. Emulates the Dockerfile patching logic out-of-the-box (fixes C++ incompatibilities regarding strict pointer ownership and ROS 1 backward features using built-in Python scripts).
6. Installs Basler Pylon SDK natively.
7. Installs required python libraries in an isolated **Virtual Environment** (`venv`).
8. Compiles the ROS 1 workspace with `catkin_make`.

---

## Manual Execution & Workspace Initialization

Every time you open a new terminal to interact with the robot, camera or visualization (RViz):

```bash
# Source the isolated system and built ROS Noetic environment
source /opt/ros/noetic/setup.bash &&
source ~/ur10_local_ws/devel/setup.bash &&
source ~/ur10_local_ws/venv/bin/activate
```

## Running the Examples Natively

Once you source the proper layers, the commands represent exactly the ones used in the Docker environment.

### 1. Unified Launch (Arm + RViz + Camera Support)
Connect the robot via Ethernet (expected IP `10.1.1.2` by default):
```bash
roslaunch ur_modern_driver ur10_camera_bringup.launch robot_ip:=10.1.1.2 
```
*(Check RViz visually to confirm tracking telemetry alongside camera availability).*

### 2. Manual URScript Execution
Use the CLI to publish `movej` trajectory strings directly:
```bash
rostopic pub -1 /ur_driver/URScript std_msgs/String "data: 'movej([0.0, -1.5708, 0.0, -1.5708, 0.0, 3.1415], a=0.5, v=0.5)'"
```

### 3. Executing a Trajectory Target Script (Python)
Since you are inside the `venv` virtual environment, Python dependencies perfectly match the runtime:
```bash
rosrun ur_modern_driver move_to_xyz.py
```

### 4. Triggering Basler Frame Capture
Initiate an action sequence for the trigger listener without Docker isolation:
```bash
rosservice call /trigger_camera
```

> **Note:** Natively, images get safely serialized into your actual home directory without requiring privileged mount layers (`-v $HOME:/host_home` in Docker).