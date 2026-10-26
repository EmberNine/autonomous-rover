#!/bin/bash
# The EKF service's command (compose.yaml): build rover_ekf from the read-only repo into the
# /ws volume (colcon only rebuilds what changed), then run it. A failed build stops here, and the
# restart policy tries again: the error is in `docker compose logs ekf`.
set -e
source /opt/ros/jazzy/setup.bash
cd /ws
colcon --log-base /ws/log build --base-paths /repo/ros_ws/src/rover_ekf \
  --build-base /ws/build --install-base /ws/install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source /ws/install/setup.bash
exec ros2 run rover_ekf ekf_node --ros-args -p vehicle_cfg:=/repo/config/vehicle.cfg
