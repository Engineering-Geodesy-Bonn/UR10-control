# UR10 & Basler Camera Docker Setup

This container runs the UR10 modern driver AND the Basler pylon camera driver, allowing full control of the robotic arm and image capturing via ROS 1.

---

## 🚀 Quick Start (Integrated Environment)

### 0. Build the Docker Image
Before running the container for the first time or after making changes to the repository, you need to build the Docker image containing both the UR10 driver and the Basler camera ROS nodes:

```bash
docker build -t ur10_basler .
```

### 1. Allow GUI Access (Host Machine)
Before starting the container, allow Docker to show graphical windows (like RViz) on your local X11 display:

```bash
xhost +local:docker
```

### 2. Run the Docker Container
Start the deeply integrated environment. This maps your network (for the UR10), your USB ports (for the camera), your display, and your Home folder (to save captured images):

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

### 3. Launch the ROS Nodes (Inside the Container)
Inside the container, your ROS workspace is already sourced. Launch the Basler camera driver and RViz visualization (this will open the RViz GUI):

```bash
roslaunch pylon_trigger camera_with_trigger_rviz.launch &
```

Then, launch the UR10 driver (replace `10.1.1.2` with your actual robot IP):

```bash
roslaunch ur_modern_driver ur10_bringup.launch robot_ip:=10.1.1.2 
```

### 4. Trigger an Image Capture
To trigger an image capture that updates in RViz and saves to your Host's home folder (`~/basler_image.png`), open a **NEW terminal on your host**:

```bash
# Find your container ID:
docker ps

# Execute a bash shell inside the running container:
docker exec -it <container_id> bash

# Call the trigger service:
rosservice call /trigger_camera
```

Once triggered, RViz will display the colored, full-resolution image. The image will also automatically be saved to your host machine's home directory.

---

## 🤖 UR10 Standalone Commands 

If you just want to run the standalone UR10 commands inside the container:

**Bringup & Visualization:**
```bash
# Bringup UR10
roslaunch ur_modern_driver ur10_bringup.launch robot_ip:=10.1.1.2

# RViz Visualization
roslaunch ur_modern_driver ur_bringup_rviz.launch robot_model:=ur10 robot_ip:=10.1.1.2
```

**Move the Arm (via URScript):**
```bash
# Connect to the running container
docker exec -it <container_id> bash

# Move to a specific joint configuration
rostopic pub -1 /ur_driver/URScript std_msgs/String "data: 'movej([0, -1.57, 0, 0, 0, 0], a=1.2, v=1.2)'"

# Bring robot to the initial position
rostopic pub -1 /ur_driver/URScript std_msgs/String "data: 'movej([0.0, -1.5708, 0.0, -1.5708, 0.0, 3.1415], a=0.5, v=0.5)'"
```

**Drive to List of Points (from CSV):**
```bash
rosrun ur_modern_driver move_to_xyz.py
```