FROM ros:humble-ros-base

# Install basic requirements
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    python3-pip \
    portaudio19-dev \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    ros-humble-rviz2 \
    software-properties-common \
    ros-humble-rmw-cyclonedds-cpp \
    ffmpeg

# Create workspace
WORKDIR /ros2_ws

# ---------------------------------------------------------------------------
# DEPENDENCY LAYERS FIRST (keyed only on the dependency manifests, NOT the
# source tree). Docker reuses these cached layers on every rebuild unless one
# of the copied manifests actually changes. That means editing launch/config/
# driver code no longer re-runs the heavy (~5 min) torch/open3d pip install or
# rosdep - only the source COPY + colcon build below re-run.
# ---------------------------------------------------------------------------

# Python deps (heavy: torch/open3d/...). Copy ONLY requirements.txt so this
# layer stays cached unless requirements.txt itself changes.
RUN pip install 'numpy<2.0.0'
COPY requirements.txt src/requirements.txt
RUN cd src && pip install -r requirements.txt

# ROS system deps via rosdep. Copy ONLY the package.xml manifests (directory
# structure preserved) so rosdep stays cached unless a manifest changes.
# NOTE: if you add a NEW ROS package to the repo, add its package.xml here too,
# otherwise its system deps won't be installed and `colcon build` will fail.
COPY go2_interfaces/package.xml      src/go2_interfaces/package.xml
COPY go2_robot_sdk/package.xml       src/go2_robot_sdk/package.xml
COPY go2_nav_sim/package.xml         src/go2_nav_sim/package.xml
COPY lidar_processor/package.xml     src/lidar_processor/package.xml
COPY lidar_processor_cpp/package.xml src/lidar_processor_cpp/package.xml
COPY coco_detector/package.xml       src/coco_detector/package.xml
COPY speech_processor/package.xml    src/speech_processor/package.xml
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y

# Pin setuptools to a version that is compatible with ament_python. The heavy
# pip deps above (torch/open3d/...) otherwise drag in a setuptools so new that
# `colcon build` fails on the ament_python packages with either
# "canonicalize_version() got an unexpected keyword argument 'strip_trailing_zero'"
# or a removed `setup.py install`. 70.0.0 vendors a matching packaging (24.x)
# and still supports the setup.py path colcon uses. Kept as the LAST pip step so
# nothing above can bump it back up.
RUN pip install "setuptools==70.0.0"

# ---------------------------------------------------------------------------
# SOURCE + BUILD. Only these re-run when you edit code (which is unavoidable -
# a code change must be recompiled). Everything above is a cache hit.
# ---------------------------------------------------------------------------
COPY . /ros2_ws/src
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build

# Source ROS2 and workspace in bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# Set environment variables
ENV ROBOT_IP=""
ENV CONN_TYPE="webrtc"
ENV WEBRTC_SERVER_HOST="0.0.0.0"
ENV WEBRTC_SERVER_PORT="9991"

# Remove apt cache
RUN rm -rf /var/lib/apt/lists/*

# add /ros2_ws to the entrypoint
RUN sed --in-place --expression \
      '$isource "/ros2_ws/install/setup.bash"' \
      /ros_entrypoint.sh

CMD ["ros2", "launch", "go2_robot_sdk", "robot.launch.py"]
