# GoogleTest is downloaded and built at configure time (FetchContent), so no
# system-wide install is needed - the first configure needs internet access.
include(FetchContent)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG v1.15.2
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(googletest)

add_executable(rover_tests
  test_bicycle_model.cpp
  test_extended_kalman_filter.cpp
  test_extended_kalman_filter_system.cpp
  test_odometry_fusion.cpp
  test_vehicle_config.cpp
)
target_link_libraries(rover_tests PRIVATE rover_core rover_warnings GTest::gtest_main)

# Lets tests locate the shipped config/vehicle.cfg regardless of build dir.
target_compile_definitions(rover_tests PRIVATE
  ROVER_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

include(GoogleTest)
gtest_discover_tests(rover_tests)  # registers each TEST() with ctest individually
