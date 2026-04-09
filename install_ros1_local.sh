#!/bin/bash
set -e

# ==============================================================================
# UR10 & Basler Camera Local Installation Script (Ubuntu 20.04 LTS / ROS 1 Noetic)
# ==============================================================================

# Get repository directory right at the start
REPO_DIR=$(cd "$(dirname "$0")" && pwd)
export REPO_DIR=$REPO_DIR

echo "Starting Local Installation for UR10-control & Basler..."

# 1. Setup ROS Noetic Sources
if [ ! -f /etc/apt/sources.list.d/ros-latest.list ]; then
    echo "Setting up ROS Noetic repository..."
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt update || true
    sudo apt install -y curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
fi

# 2. System Dependencies
echo "Installing standard system & ROS dependencies..."
sudo apt update
sudo apt install -y --no-install-recommends \
    git \
    build-essential \
    ca-certificates \
    python3-rosdep \
    python3-catkin-tools \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-venv \
    python3-pip \
    python-is-python3 \
    iputils-ping \
    net-tools \
    ros-noetic-desktop-full \
    ros-noetic-controller-manager \
    ros-noetic-hardware-interface \
    ros-noetic-joint-trajectory-controller \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-industrial-msgs \
    ros-noetic-industrial-robot-client \
    ros-noetic-kdl-parser \
    ros-noetic-ur-msgs \
    ros-noetic-ur-description

# 3. Initialize rosdep
echo "Initializing rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init || true
fi
rosdep update

# 4. Setup Local Workspace
export CATKIN_WS=$HOME/ur10_local_ws
echo "Setting up catkin workspace at $CATKIN_WS..."
mkdir -p $CATKIN_WS/src
cd $CATKIN_WS/src

# Symlink the UR10-control repository into the workspace.
if [ ! -d $CATKIN_WS/src/ur10_control ]; then
    # We rename it to ur10_control to ensure catkin discovers it normally
    ln -s $REPO_DIR $CATKIN_WS/src/ur10_control
fi

if [ ! -d $CATKIN_WS/src/pylon_trigger ]; then
    ln -s $REPO_DIR/pylon_trigger $CATKIN_WS/src/pylon_trigger
fi

# Clone external dependencies
if [ ! -d $CATKIN_WS/src/universal_robot ]; then
    git clone --depth 1 --branch kinetic-devel https://github.com/ros-industrial/universal_robot.git
fi
if [ ! -d $CATKIN_WS/src/pylon-ros-camera ]; then
    git clone -b pylon5-legacy https://github.com/basler/pylon-ros-camera.git
fi
if [ ! -d $CATKIN_WS/src/dragandbot_common ]; then
    git clone https://github.com/dragandbot/dragandbot_common.git
fi

# 5. Patch & Prune specific packages (mimic Dockerfile behavior)
find $CATKIN_WS/src/universal_robot -mindepth 1 -maxdepth 1 -type d \
    ! -name ur_description ! -name ur_msgs \
    ! -name universal_robot \
    -exec touch {}/CATKIN_IGNORE \;

if [ ! -f $CATKIN_WS/src/universal_robot/ur_description/launch/ur10_upload.launch ]; then
    printf '%s\n' \
    '<launch>' \
    '  <arg name="limited" default="false"/>' \
    '  <arg name="prefix" default=""/>' \
    '  <param name="robot_description"' \
    '         command="$(find xacro)/xacro --inorder '\''$(find ur_description)/urdf/ur10_robot.urdf.xacro'\'' prefix:=$(arg prefix) limited:=$(arg limited)" />' \
    '</launch>' \
    > $CATKIN_WS/src/universal_robot/ur_description/launch/ur10_upload.launch
fi

# Apply ur_cpp patch as inherited from Dockerfile
export REPO_DIR=$REPO_DIR
python3 - <<'PY'
import io, os, re

# For local patching, we assume the repo directory holds these files in `src/`!
repo_dir = os.environ.get('REPO_DIR')
p1 = os.path.join(repo_dir, "src/ur_hardware_interface.cpp")
with io.open(p1, "r", encoding="utf-8") as f:
    s = f.read()

if "#include <string>" not in s:
    inc_anchor = '#include "ur_modern_driver/ur_hardware_interface.h"\n'
    s = s.replace(inc_anchor, inc_anchor + "#include <string>\n", 1) if inc_anchor in s else "#include <string>\n" + s

helper = """
namespace {
const std::string& ctrlHwIface(const hardware_interface::ControllerInfo& c)
{
  static const std::string empty;
  if (!c.claimed_resources.empty())
    return c.claimed_resources.front().hardware_interface;
  return empty;
}
}  // namespace

"""
if "ctrlHwIface(const hardware_interface::ControllerInfo& c)" not in s:
    i = s.find("namespace ros_control_ur")
    s = s[:i] + helper + s[i:] if i != -1 else s + "\n" + helper

s = s.replace("stop_controller_it->hardware_interface", "ctrlHwIface(*stop_controller_it)")
s = s.replace("controller_it->hardware_interface", "ctrlHwIface(*controller_it)")
s = s.replace("stop_ctrlHwIface(*controller_it)", "ctrlHwIface(*stop_controller_it)")

with io.open(p1, "w", encoding="utf-8") as f:
    f.write(s)

p2 = os.path.join(repo_dir, "src/ur_ros_wrapper.cpp")

# SetPayload.srv might be resolved remotely relative to workspace or system
srv1 = os.path.join(os.environ.get('CATKIN_WS', '/catkin_ws'), "src/universal_robot/ur_msgs/srv/SetPayload.srv")
srv2 = "/opt/ros/noetic/share/ur_msgs/srv/SetPayload.srv"

field = "payload"
for srv in [srv1, srv2]:
    if os.path.exists(srv):
        print("Parsing srv file at:", srv)
        with io.open(srv, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                if line == "---":
                    break
                m = re.match(r"^[A-Za-z0-9_/]+\s+([A-Za-z_][A-Za-z0-9_]*)$", line)
                if m:
                    field = m.group(1)
                    break
        break

with io.open(p2, "r", encoding="utf-8") as f:
    s2 = f.read()

s2 = re.sub(
    r"robot_\.setPayload\(req\.(payload|mass)\)",
    "robot_.setPayload(req." + field + ")",
    s2
)

with io.open(p2, "w", encoding="utf-8") as f:
    f.write(s2)
print("Locally Patched:", p1)
PY


# 6. Install Pylon Software
PYLON_TAR="$REPO_DIR/pylon-26.03.1_linux-x86_64_debs.tar.gz"
if dpkg -l | grep -q pylon; then
    echo "Pylon SDK is already installed. Skipping..."
else
    echo "Installing Pylon SDK from $PYLON_TAR"
    mkdir -p /tmp/pylon_install
    cd /tmp/pylon_install
    tar -xzf $PYLON_TAR
    sudo apt-get install -y ./pylon_*.deb ./codemeter*.deb
    echo "/opt/pylon/lib" | sudo tee /etc/ld.so.conf.d/pylon.conf
    sudo ldconfig
    
    # Configure USB rules natively
    if [ -f /opt/pylon/share/pylon/setup-usb.sh ]; then
        echo "y\ny\n" | sudo /opt/pylon/share/pylon/setup-usb.sh
        sudo udevadm control --reload-rules
        sudo udevadm trigger
    fi
     
    rm -rf /tmp/pylon_install
fi

# 7. Apply Pylon patch for finding via CMake (ensure the script handles CATKIN_WS correctly)
export CATKIN_WS=$CATKIN_WS
python3 $REPO_DIR/patch_findpylon.py $(find $CATKIN_WS/src -name FindPylon.cmake -o -name CMakeLists.txt)

# 8. Install missing ros dependencies
echo "Installing remaining ROS package dependencies..."
cd $CATKIN_WS
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y

# 9. Setup Python Virtual Environment using access to system site packages
VENV_DIR=$CATKIN_WS/venv
echo "Setting up Python Virtual Environment in $VENV_DIR..."
python3 -m venv --system-site-packages $VENV_DIR
source $VENV_DIR/bin/activate
pip install --upgrade pip

# Install python requirements
if [ -f $REPO_DIR/requirements.txt ]; then
    pip install -r $REPO_DIR/requirements.txt
fi

# 10. Build the workspace natively
echo "Building the workspace..."
cd $CATKIN_WS
catkin_make -DCMAKE_BUILD_TYPE=Release

echo "========================================================================="
echo " Installation Complete! "
echo "========================================================================="
echo "To use your environment, source the following in every new terminal:"
echo ""
echo "  source /opt/ros/noetic/setup.bash"
echo "  source $CATKIN_WS/devel/setup.bash"
echo "  source $CATKIN_WS/venv/bin/activate"
echo ""
echo "Or add them to your ~/.bashrc."
echo "========================================================================="
