cmake_minimum_required(VERSION 3.8)
project(point_cloud2)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(PCL 1.12 REQUIRED COMPONENTS io filters registration console surface visualization)
find_package(pcl_conversions REQUIRED)

add_executable(point_cloud_extract src/point_cloud2_extract.cpp)
add_executable(icp src/icp_point_cloud.cpp)
add_executable(imu_extract src/imu_extract.cpp)

ament_target_dependencies(point_cloud_extract rclcpp sensor_msgs rosbag2_cpp pcl_conversions)
ament_target_dependencies(imu_extract rclcpp sensor_msgs rosbag2_cpp)

target_link_libraries(point_cloud_extract ${PCL_LIBRARIES})
target_link_libraries(icp ${PCL_LIBRARIES})

install(TARGETS 
            point_cloud_extract 
            icp
            imu_extract

        DESTINATION 
            lib/${PROJECT_NAME}
)

ament_package()
