FROM osrf/ros:melodic-desktop-full

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=melodic
ENV CATKIN_WS=/catkin_ws

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

RUN mkdir -p ${CATKIN_WS}/src
WORKDIR ${CATKIN_WS}/src
RUN git clone --depth 1 --branch master https://github.com/ros-industrial-attic/ur_modern_driver.git && \
    git clone --depth 1 --branch kinetic-devel https://github.com/ros-industrial/universal_robot.git

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

# Compatibility patches
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

WORKDIR ${CATKIN_WS}
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --rosdistro=${ROS_DISTRO} --from-paths src/ur_modern_driver src/universal_robot/ur_description src/universal_robot/ur_msgs --ignore-src -r -y

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin_make -DCMAKE_BUILD_TYPE=Release --pkg ur_modern_driver ur_description ur_msgs

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source ${CATKIN_WS}/devel/setup.bash && \
    rospack find ur_modern_driver

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source ${CATKIN_WS}/devel/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]