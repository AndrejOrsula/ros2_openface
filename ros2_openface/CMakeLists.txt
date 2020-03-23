cmake_minimum_required(VERSION 3.5)
project(openface)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Testing and linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ament_cmake
find_package(ament_cmake REQUIRED)
# ROS2 packages
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# ROS2 interfaces
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(openface_msgs REQUIRED)
# OpenCV 4
find_package(OpenCV 4 REQUIRED)
# Boost
find_package(Boost 1.5.9 REQUIRED COMPONENTS filesystem system)
# OpenFace
find_library(OPENFACE_LD_LIB LandmarkDetector PATH_SUFFIXES openface)
find_library(OPENFACE_FA_LIB FaceAnalyser PATH_SUFFIXES openface)
find_library(OPENFACE_GA_LIB GazeAnalyser PATH_SUFFIXES openface)
find_library(OPENFACE_UT_LIB Utilities PATH_SUFFIXES openface)
set(OPENFACE_LIBS ${OPENFACE_LD_LIB} ${OPENFACE_FA_LIB} ${OPENFACE_GA_LIB}
                  ${OPENFACE_UT_LIB})
find_path(OPENFACE_INCLUDE_DIRS LandmarkCoreIncludes.h PATH_SUFFIXES OpenFace)

# Include directories
include_directories(
  include ${OPENFACE_INCLUDE_DIRS} ${OpenBLAS_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}/boost ${OpenCV_INCLUDE_DIRS})

# Executables
set(EXECUTABLE_0 openface)
set(EXECUTABLE_1 openface_separate)

# Executable 0
add_executable(${EXECUTABLE_0} src/${EXECUTABLE_0}.cpp)
target_link_libraries(${EXECUTABLE_0} ${OPENFACE_LIBS} openblas dlib
                      ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})
ament_target_dependencies(
  ${EXECUTABLE_0}
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  openface_msgs
  image_transport
  cv_bridge
  tf2
  tf2_ros)

# Executable 1
add_executable(${EXECUTABLE_1} src/${EXECUTABLE_1}.cpp)
target_link_libraries(${EXECUTABLE_1} ${OPENFACE_LIBS} openblas dlib
                      ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})
ament_target_dependencies(
  ${EXECUTABLE_1}
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  openface_msgs
  image_transport
  cv_bridge
  tf2
  tf2_ros)

# Install targets
install(TARGETS ${EXECUTABLE_0} ${EXECUTABLE_1} DESTINATION lib/${PROJECT_NAME})
# Install directories
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# Setup the project
ament_package()
