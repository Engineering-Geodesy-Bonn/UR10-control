FROM osrf/ros:melodic-desktop-full

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=melodic
ENV CATKIN_WS=/catkin_ws
ENV PYLON_ROOT=/opt/pylon
ENV CPATH=/opt/pylon/include

# ==============================================================================
# Stage 1: System Dependencies & Base Setup (rarely changes)
# ==============================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    build-essential \
    ca-certificates \
    python-rosdep \
    python-catkin-tools \
    iputils-ping \
    net-tools \
    ros-melodic-controller-manager \
    ros-melodic-hardware-interface \
    ros-melodic-joint-trajectory-controller \
    ros-melodic-ros-control \
    ros-melodic-ros-controllers \
    ros-melodic-industrial-msgs \
    ros-melodic-industrial-robot-client \
    ros-melodic-kdl-parser \
    ros-melodic-ur-msgs \
    ros-melodic-ur-description \
    && rm -rf /var/lib/apt/lists/*

RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi && \
    rosdep update

# ==============================================================================
# Stage 2: Clone Git Repositories (rarely changes)
# ==============================================================================
RUN mkdir -p ${CATKIN_WS}/src
WORKDIR ${CATKIN_WS}/src
RUN git clone --depth 1 --branch master https://github.com/ros-industrial-attic/ur_modern_driver.git && \
    git clone --depth 1 --branch kinetic-devel https://github.com/ros-industrial/universal_robot.git && \
    git clone -b pylon5-legacy https://github.com/basler/pylon-ros-camera.git && \
    git clone https://github.com/dragandbot/dragandbot_common.git

# ==============================================================================
# Stage 3: Setup & Patches (rarely changes)
# ==============================================================================
# Keep only ur_description + ur_msgs from universal_robot (ignore moveit/kinematics/etc)
RUN find ${CATKIN_WS}/src/universal_robot -mindepth 1 -maxdepth 1 -type d \
    ! -name ur_description ! -name ur_msgs \
    -exec touch {}/CATKIN_IGNORE \;

# Ensure legacy launch file expected by ur_modern_driver exists
RUN if [ ! -f ${CATKIN_WS}/src/universal_robot/ur_description/launch/ur10_upload.launch ]; then \
      printf '%s\n' \
      '<launch>' \
      '  <arg name="limited" default="false"/>' \
      '  <arg name="prefix" default=""/>' \
      '  <param name="robot_description"' \
      '         command="$(find xacro)/xacro --inorder '\''$(find ur_description)/urdf/ur10_robot.urdf.xacro'\'' prefix:=$(arg prefix) limited:=$(arg limited)" />' \
      '</launch>' \
      > ${CATKIN_WS}/src/universal_robot/ur_description/launch/ur10_upload.launch; \
    fi

# ==============================================================================
# Stage 4: Apply Compatibility Patches (rarely changes)
# ==============================================================================
RUN python - <<'PY'
import io, os, re

p1 = "/catkin_ws/src/ur_modern_driver/src/ur_hardware_interface.cpp"
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

# Auto-detect SetPayload request field: payload OR mass
p2 = "/catkin_ws/src/ur_modern_driver/src/ur_ros_wrapper.cpp"
srv = "/catkin_ws/src/universal_robot/ur_msgs/srv/SetPayload.srv"

field = "payload"
if os.path.exists(srv):
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

with io.open(p2, "r", encoding="utf-8") as f:
    s2 = f.read()

s2 = re.sub(
    r"robot_\.setPayload\(req\.(payload|mass)\)",
    "robot_.setPayload(req.{})".format(field),
    s2
)

with io.open(p2, "w", encoding="utf-8") as f:
    f.write(s2)

print("Patched:", p1)
print("Patched:", p2, "using field:", field)
PY

# ==============================================================================
# Stage 5: Install UR Dependencies & Build UR Packages (rarely changes)
# ==============================================================================
WORKDIR ${CATKIN_WS}
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --rosdistro=${ROS_DISTRO} --from-paths src/ur_modern_driver src/universal_robot/ur_description src/universal_robot/ur_msgs --ignore-src -r -y

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin_make -DCMAKE_BUILD_TYPE=Release --pkg ur_modern_driver ur_description ur_msgs

# ==============================================================================
# Stage 6: Install Pylon Software (rarely changes)
# ==============================================================================
COPY pylon-26.03.1_linux-x86_64_debs.tar.gz /root/
RUN cd /root && tar -xzf pylon-26.03.1_linux-x86_64_debs.tar.gz && \
    apt-get update && \
    apt-get install -y ./pylon_*.deb ./codemeter*.deb && \
    rm -rf /root/pylon* /root/codemeter*

RUN echo "/opt/pylon/lib" > /etc/ld.so.conf.d/pylon.conf && ldconfig

# ==============================================================================
# Stage 7: Install Pylon Dependencies & Patch (rarely changes)
# ==============================================================================
COPY patch_findpylon.py /patch_findpylon.py
RUN python /patch_findpylon.py $(find /catkin_ws/src -name FindPylon.cmake -o -name CMakeLists.txt)

WORKDIR ${CATKIN_WS}
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --from-paths src/pylon-ros-camera src/dragandbot_common --ignore-src -r -y

# ==============================================================================
# Stage 8: Copy Pylon Trigger Package (semi-frequently changes)
# ==============================================================================
COPY pylon_trigger /catkin_ws/src/pylon_trigger

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --from-paths src/pylon_trigger --ignore-src -r -y

# ==============================================================================
# Stage 9: Copy Configuration & Data Files (semi-frequently changes)
# ==============================================================================
COPY config /catkin_ws/src/ur_modern_driver/config/
COPY launch /catkin_ws/src/ur_modern_driver/launch/
COPY *.csv /catkin_ws/src/ur_modern_driver/

# ==============================================================================
# Stage 10: Copy Python Scripts (frequently changes)
# ==============================================================================
COPY src/*.py /catkin_ws/src/ur_modern_driver/src/

# ==============================================================================
# Stage 11: Final Build & Environment Setup
# ==============================================================================
WORKDIR ${CATKIN_WS}
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin_make -DCMAKE_BUILD_TYPE=Release

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source ${CATKIN_WS}/devel/setup.bash && \
    rospack find ur_modern_driver

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source ${CATKIN_WS}/devel/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]