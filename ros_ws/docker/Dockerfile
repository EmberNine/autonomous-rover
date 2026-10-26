# The rover's image (the bridge and the EKF): ROS 2 Jazzy base + pyserial, and
# the C++ build tools for rover_ekf. The code is not in the image: compose.yaml
# mounts the repo read-only, so a `git pull` and a restart update it (no rebuild;
# the EKF service compiles its package into the /ws volume when it starts).
FROM ros:jazzy-ros-base

RUN apt-get update \
 && apt-get install -y --no-install-recommends python3-serial \
      build-essential cmake libeigen3-dev python3-colcon-common-extensions \
      ros-jazzy-tf2-ros ros-jazzy-nav-msgs ros-jazzy-std-srvs \
 && rm -rf /var/lib/apt/lists/*

# The EKF's colcon workspace (a named volume takes this owner the first time): uid 1000, as compose's user.
RUN mkdir -p /ws && chown 1000:1000 /ws
