cmake_minimum_required(VERSION 3.20)
project(rover_ekf LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

# The ROS-free core (bicycle model, EKF, OdometryFusion, vehicle.cfg parser)
# from the repo root, built here as a static library; its GoogleTest suite is
# the root project's business (ctest), not this package's.
set(ROVER_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../.. rover_core)

add_executable(ekf_node src/ekf_node.cpp)
target_link_libraries(ekf_node rover_core rover_warnings)
ament_target_dependencies(ekf_node rclcpp sensor_msgs nav_msgs geometry_msgs std_srvs tf2_ros)

install(TARGETS ekf_node DESTINATION lib/${PROJECT_NAME})

# The node wired up (test/test_ekf_node.py), against the build tree's executable so the test
# doesn't depend on `ros2 pkg prefix` finding an install.
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_ekf_node test/test_ekf_node.py
    ENV ROVER_EKF_NODE=$<TARGET_FILE:ekf_node>
    TIMEOUT 60)
endif()

ament_package()
