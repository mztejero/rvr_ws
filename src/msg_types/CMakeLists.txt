cmake_minimum_required(VERSION 3.5)
project(msg_types)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files
  "msg/Joystick.msg"
  "msg/IMUdata.msg"
  "msg/Vector3.msg"
  "msg/LiDAR.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
