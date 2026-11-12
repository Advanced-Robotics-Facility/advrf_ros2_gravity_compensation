cmake_minimum_required(VERSION 3.16)
project(advrf_ros2_gravity_compensation)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(advrf_ros2_interfaces REQUIRED)

find_package(xbot2_interface REQUIRED)

add_executable(advrf_ros2_gravity_compensation
  src/gravity_compensation.cpp
)

ament_target_dependencies(advrf_ros2_gravity_compensation
  rclcpp
  sensor_msgs
  std_srvs
  advrf_ros2_interfaces
)

target_link_libraries(advrf_ros2_gravity_compensation
  xbot2_interface::xbot2_interface
)

install(TARGETS advrf_ros2_gravity_compensation
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
