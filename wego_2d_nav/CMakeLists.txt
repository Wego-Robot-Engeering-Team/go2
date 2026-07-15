cmake_minimum_required(VERSION 3.8)
project(wego_2d_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(./include/)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/WaypointTask.srv"
  DEPENDENCIES
)

add_executable(waypoint_follow ./src/waypoint_follow.cpp ./include/wego_2d_nav/waypoint_follow.hpp)
ament_target_dependencies(waypoint_follow 
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  std_srvs
)

add_executable(waypoint_save ./src/waypoint_save.cpp)
ament_target_dependencies(waypoint_save
                          rclcpp
                          std_srvs
                          tf2_ros
                          tf2_geometry_msgs
                          geometry_msgs
                          )

add_executable(waypoint_runner ./src/waypoint_runner.cpp)
ament_target_dependencies(waypoint_runner
                          rclcpp
                          rclcpp_action
                          nav2_msgs
                          geometry_msgs
                          std_msgs
                          std_srvs
                          )
# rosidl_get_typesupport_target() is not available in ROS 2 Foxy.
rosidl_target_interfaces(waypoint_runner ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  waypoint_save
  waypoint_follow
  waypoint_runner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
)

# Install only the Foxy-compatible Nav2 launch and configuration files.
# This excludes archived Humble-era copies and the unsupported speed-filter
# configuration from the installed package.
install(FILES
  launch/localization_launch.py
  launch/navigation_launch.py
  launch/waypoint_follower_launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

install(FILES
  config/nav2_params.yaml
  config/waypoint.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
