cmake_minimum_required(VERSION 3.8)
project(rover_description)

find_package(ament_cmake REQUIRED)

# No compiled code: this package only ships data files (model, launch, world).
# ROS tools look them up under share/<package_name>, so install them there.
install(
  DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# vehicle.cfg lives in the repo root, not inside this package, because it is
# the single source of truth shared with the ROS-free C++ core (see
# CMakeLists.txt / tests at the repo root). Installing it by reference
# (via --symlink-install) rather than copying it means colcon build never
# gets a chance to go stale against the file the C++ tests also read.
install(
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../../config/vehicle.cfg
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()