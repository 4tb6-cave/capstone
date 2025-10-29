cmake_minimum_required(VERSION 3.8)
project(pointCloud2)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(PCL 1.12 REQUIRED COMPONENTS io filters)
find_package(pcl_conversions REQUIRED)

add_executable(pointCloud2 src/point_cloud2_extract.cpp)
ament_target_dependencies(pointCloud2 rclcpp sensor_msgs rosbag2_cpp pcl_conversions)
target_link_libraries(pointCloud2 ${PCL_LIBRARIES})
install(TARGETS pointCloud2 DESTINATION lib/${PROJECT_NAME})

ament_package()
