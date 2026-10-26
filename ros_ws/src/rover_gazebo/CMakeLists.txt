cmake_minimum_required(VERSION 3.8)
project(rover_gazebo)

find_package(ament_cmake REQUIRED)

# No compiled code: this package only ships data files (world, xacro,
# launch, bridge config), the same pattern as rover_description.
install(
  DIRECTORY worlds urdf launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
