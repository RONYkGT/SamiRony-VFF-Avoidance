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

add_executable(avoidance_node src/vff_avoidance/AvoidanceNode.cpp)

ament_target_dependencies(avoidance_node
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
)

install(TARGETS avoidance_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
