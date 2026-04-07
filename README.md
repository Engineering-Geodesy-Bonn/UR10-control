# UR10 & Basler Camera Integrated ROS Workspace

This repository provides a deeply integrated ROS 1 workspace for controlling a Universal Robots arm (UR3, UR5, or UR10) alongside image capture with a Basler Pylon camera. It extends the legacy `ur_modern_driver` by incorporating custom movement logic, boundary safety constraints, and automated camera triggering capabilities, containerized efficiently with Docker.

---

## 📦 Repository Structure

- `ur_modern_driver` : The core driver handling C++ real-time communication with Universal Robots hardware.
- `pylon_trigger` : Custom package containing the action client interface and camera triggering service for the Basler Pylon driver.
- `launch/` : Contains bringup files bridging the robotic arm, control GUI (RViz), and vision system.
- `src/` : Custom Python movement nodes, coordinate generators, and publisher utilities.
- `urdf/` : Specialized layout descriptions like visual markers for safety boundaries.
- `config/` : Controller parameters and saved RViz visualizations.

---

## 🐳 Docker Setup & Quick Start

A complete environment including both the arm driver and camera dependencies can be built using the provided Dockerfile.

### 1. Build the Integration Image
Before running the workspace, build the Docker container:
```bash
docker build -t ur10_basler .
```

### 2. Allow GUI Access (Host Machine)
To visualize the arm and camera stream in RViz on your local display, allow X11 connections:
```bash
xhost +local:docker
```

### 3. Run the Container
Start the container. This grants host networking (required for the UR10 ethernet connection), USB access (for the Basler Camera), and maps your Home folder to save the captured images:
```bash
docker run -it --rm \
  --network host \
  --privileged \
  -v /dev/bus/usb:/dev/bus/usb \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $HOME:/host_home \
  -e HOST_HOME=/host_home \
  ur10_basler bash
```

---

## 🚀 Launch Files Overview

Inside the workspace, you can trigger specific workflows using the pre-configured generic and specialized launch scripts located in the `launch/` and `pylon_trigger/launch/` directories.

### Integrated Launch (Arm + Camera)
- **`ur10_camera_bringup.launch`**: Brings up the entire integrated system. It acts as the main entry point:
  - Starts the UR10 hardware communication over ethernet (`robot_ip`).
  - Launches the Basler `pylon_camera_node`.
  - Generates a static TF transform mapping `tool0` (the robot end-effector) to `camera`.
  - Starts the custom `camera_trigger_node`.
  - Optionally configures and opens an RViz instance pre-loaded with point cloud and arm telemetry.
  *(Usage: `roslaunch ur_modern_driver ur10_camera_bringup.launch robot_ip:=10.1.1.2`)*

### Standalone UR Launches
- **`ur10_bringup.launch` / `ur5_bringup.launch` / `ur3_bringup.launch`**: Hardware communication initializers for varying standard arm lengths.
- **`ur_bringup_rviz.launch`**: Dedicated file to load only the UR configuration within RViz without the camera topics.
- **`ur10_ros_control.launch`**: Initializes hardware controllers specifically for ros_control compliance directly via position / velocity controllers rather than standard URScript.

### Camera Only
- **`camera_with_trigger_rviz.launch`** (in `pylon_trigger`): Starts the Basler camera, its service triggers, and its respective visualizer without querying the robot.

---

## 💻 Nodes & Scripts

The following custom Python executables define the logic and routines for capturing synchronized data from the robot endpoints.

### Movement Scripts (`src/`)
- **`move_to_xyz.py`**: The primary data-gathering script. Parses a set list of XYZ cartesian targets (typically from `.csv` like `robotpos_xyz.csv` or `dome_poses.csv`) and commands the robot to move successively. Internally guards movements against preset dynamic table bounding boxes.
- **`move_to_pose.py`**: A trajectory-action client focusing on preset joint configurations. Can move the arm safely back into states like `elbow_up` by parsing states on `/joint_states`.
- **`table_marker_publisher.py`**: Continuously publishes spatial bounding box markers for RViz visualization to guarantee the operator can easily evaluate physical clearance constraints before running trajectory tasks.
- **`generate_dome_poses.py`**: A utility specifically built to formulate `.csv` paths consisting of spherical/dome-like orientations around a focal point on the workspace table.

### Trigger Scripts (`pylon_trigger/scripts/`)
- **`trigger_camera.py`**: Serves as the listener for `/trigger_camera`. Initiates an image extraction via `GrabImagesAction`, decodes the format matching `cv_bridge`, and writes out the resulting raw image strictly to `$HOST_HOME` matching chronological time stamps.

---

## 🕹️ Typical Operations Workflow

### 1. Unified Launch
Connect the 10.1.1.2 Ethernet socket and launch:
```bash
roslaunch ur_modern_driver ur10_camera_bringup.launch robot_ip:=10.1.1.2 
```
Check RViz visually to confirm proper tracking of the arm telemetry.

### 2. Manual Arm Command (URScript)
Using the terminal, command `movej` coordinates to return to a standard ready position:
```bash
rostopic pub -1 /ur_driver/URScript std_msgs/String "data: 'movej([0.0, -1.5708, 0.0, -1.5708, 0.0, 3.1415], a=0.5, v=0.5)'"
```

### 3. Automated Scanning Drive
Execute large-scale `.csv` defined trajectory points:
```bash
rosrun ur_modern_driver move_to_xyz.py
```

### 4. Capture Custom Image Point
In a separate terminal host connection:
```bash
rosservice call /trigger_camera
```
The saved frame will appear in your home directory with full resolution.