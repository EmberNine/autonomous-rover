cmake_minimum_required(VERSION 3.20)
project(autonomous_rover VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)  # plain ISO C++17, no GNU-specific extensions
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)  # lets editors/clangd understand the build

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Debug builds use -Og (GCC/Clang's "optimize for debugging": keeps variables
# and line info usable in gdb) instead of the default -O0. Eigen relies on
# inlining; at -O0 every 6x6 product is hundreds of tiny function calls, and
# the Kalman filter Monte Carlo tests took ~39 s instead of ~2.5 s. This
# normal variable overrides the cached CMAKE_CXX_FLAGS_DEBUG for this project,
# so it also applies to build directories configured before this line existed.
set(CMAKE_CXX_FLAGS_DEBUG "-Og -g")

option(ROVER_BUILD_TESTS "Build the GoogleTest unit tests" ON)
option(ROVER_WARNINGS_AS_ERRORS "Fail the build on any compiler warning" OFF)

# Warning flags are attached to our own targets only (via this interface
# library), so third-party code such as GoogleTest is not held to them.
add_library(rover_warnings INTERFACE)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(rover_warnings INTERFACE -Wall -Wextra -Wpedantic)
  if(ROVER_WARNINGS_AS_ERRORS)
    target_compile_options(rover_warnings INTERFACE -Werror)
  endif()
endif()

# Eigen (header-only linear algebra) is used by the Kalman filter. Install it
# with: sudo apt install libeigen3-dev. Include directories from an imported
# target like Eigen3::Eigen are treated as SYSTEM (-isystem), so warnings inside
# Eigen's headers are not reported and cannot break a -Werror build.
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

# Core library: pure C++17, no ROS dependency, so it can be used standalone
# and later linked into a ROS 2 node.
add_library(rover_core STATIC
  src/vehicle_config.cpp
  src/bicycle_model.cpp
  src/extended_kalman_filter.cpp
  src/odometry_fusion.cpp
)
target_include_directories(rover_core PUBLIC include)
# Eigen is PUBLIC because Eigen types appear in our public headers.
target_link_libraries(rover_core PUBLIC Eigen3::Eigen PRIVATE rover_warnings)

if(ROVER_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
