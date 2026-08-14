cmake_minimum_required(VERSION 3.10)
project(livox_ros_driver2)

# This is a messages-only shim — see package.xml. The real driver lives
# at github.com/Livox-SDK/livox_ros_driver2 and additionally requires the
# Livox C++ SDK at build time. We don't need any of that; just the
# CustomMsg + CustomPoint types so FAST_LIO compiles.

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/CustomPoint.msg"
  "msg/CustomMsg.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime std_msgs)
ament_package()
