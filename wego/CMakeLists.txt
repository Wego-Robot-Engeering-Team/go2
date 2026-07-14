cmake_minimum_required(VERSION 3.8)
project(wego)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

add_executable(cloud_merger_node src/pointcloud_merge.cpp)
ament_target_dependencies(cloud_merger_node
  rclcpp sensor_msgs tf2 tf2_ros tf2_sensor_msgs message_filters pcl_conversions
)

target_include_directories(cloud_merger_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(cloud_merger_node ${PCL_LIBRARIES})

install(TARGETS cloud_merger_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch rviz config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
