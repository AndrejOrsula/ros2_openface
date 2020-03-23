cmake_minimum_required(VERSION 3.5)
project(openface_msgs)

# ament_cmake
find_package(ament_cmake REQUIRED)
# rosidl
find_package(rosidl_default_generators REQUIRED)
# ROS2 interfaces
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Interface dependencies
set(INTERFACE_DEPENDENCIES std_msgs geometry_msgs)

# New interfaces
set(MSG_FILES
    "msg/ActionUnit.msg"
    "msg/ActionUnitsStamped.msg"
    "msg/Face.msg"
    "msg/FaceStamped.msg"
    "msg/Pixel.msg"
    "msg/PixelsStamped.msg"
    "msg/PointsStamped.msg")
set(SRV_FILES)
set(ACTION_FILES)

# Generate interfaces
rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  ${ACTION_FILES}
  DEPENDENCIES
  ${INTERFACE_DEPENDENCIES}
  ADD_LINTER_TESTS)

# Export interfaces
ament_export_dependencies(rosidl_default_runtime)

# Setup the project
ament_package()
