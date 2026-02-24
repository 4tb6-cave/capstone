cmake_minimum_required(VERSION 3.0)
project(sensor_fusion CXX)

# Find GTSAM, either from a local build, or from a Debian/Ubuntu package.
find_package(GTSAM REQUIRED)

add_executable(sensor_fusion
  src/main.cpp
)

# By using CMake exported targets, a simple "link" dependency introduces the
# include directories (-I) flags, links against Boost, and add any other
# required build flags (e.g. C++11, etc.)
target_link_libraries(sensor_fusion PRIVATE gtsam)

target_include_directories(sensor_fusion PRIVATE src ../includes/CLI)
