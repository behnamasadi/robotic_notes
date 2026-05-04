cmake_minimum_required(VERSION 3.10)
project(ros2_qos_examples_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(EXECUTABLES
  camera_publisher
  latest_frame_subscriber
  custom_qos_publisher
  latest_frame_buffer
)

foreach(exe ${EXECUTABLES})
  add_executable(${exe} src/${exe}.cpp)
  ament_target_dependencies(${exe} rclcpp sensor_msgs)
endforeach()

install(TARGETS ${EXECUTABLES}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
