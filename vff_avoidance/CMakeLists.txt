cmake_minimum_required(VERSION 3.8)
project(vff_avoidance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(avoidance_node src/vff_avoidance/avoidance_vff_main.cpp src/vff_avoidance/AvoidanceNode.cpp)

ament_target_dependencies(avoidance_node
  rclcpp
  sensor_msgs
  visualization_msgs
  nav_msgs
  geometry_msgs
  rcl_interfaces
  tf2
  tf2_ros
)

install(TARGETS avoidance_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)


install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
ament_package()
