cmake_minimum_required(VERSION 3.5)
project(openface_msgs)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


# Interfaces
set(MSG_FILES
  "msg/ActionUnit.msg"
  "msg/ActionUnitsStamped.msg"
  "msg/Face.msg"
  "msg/FaceStamped.msg"
  "msg/Pixel.msg"
  "msg/PixelsStamped.msg"
  "msg/PointsStamped.msg"
)
set(SRV_FILES

)
set(ACTION_FILES

)
set(MSG_SRV_ACTION_DEPENDENCIES
  std_msgs
  geometry_msgs
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}  
  ${SRV_FILES}
  ${ACTION_FILES}
  DEPENDENCIES ${MSG_SRV_ACTION_DEPENDENCIES}
  ADD_LINTER_TESTS
)
ament_export_dependencies(rosidl_default_runtime)


ament_package()
