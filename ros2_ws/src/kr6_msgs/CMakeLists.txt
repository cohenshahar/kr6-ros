cmake_minimum_required(VERSION 3.8)
project(kr6_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ActionChunk.msg"
  "srv/Reset.srv"
  "srv/ApplyCmd.srv"
  "srv/GetChunk.srv"
  DEPENDENCIES sensor_msgs geometry_msgs
)
ament_package()
