cmake_minimum_required(VERSION 3.8)
project(gpd_ros)

# -----------------------------
# Compiler settings
# -----------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)

# -----------------------------
# Find dependencies
# -----------------------------
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)

find_package(rosidl_default_generators REQUIRED)

# -----------------------------
# Custom messages/services
# -----------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CloudIndexed.msg"
  "msg/CloudSamples.msg"
  "msg/CloudSources.msg"
  "msg/GraspConfig.msg"
  "msg/GraspConfigList.msg"
  "msg/SamplesMsg.msg"
  "srv/DetectGrasps.srv"
  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
)

# -----------------------------
# External GPD library
# -----------------------------
find_library(GPD_LIB
  NAMES gpd
  PATHS /usr/local/lib
  PATH_SUFFIXES lib
  NO_DEFAULT_PATH
)

if(NOT GPD_LIB)
  message(FATAL_ERROR "Library GPD not found")
endif()

# -----------------------------
# Libraries
# -----------------------------
add_library(${PROJECT_NAME}_grasp_messages
  src/grasp_messages.cpp
)

add_library(${PROJECT_NAME}_grasp_plotter
  src/grasp_plotter.cpp
)

# Include dirs for libraries
target_include_directories(${PROJECT_NAME}_grasp_messages PRIVATE
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_include_directories(${PROJECT_NAME}_grasp_plotter PRIVATE
  include
  ${PCL_INCLUDE_DIRS}
)

# Dependencies (ROS)
ament_target_dependencies(${PROJECT_NAME}_grasp_messages
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2_eigen
)

ament_target_dependencies(${PROJECT_NAME}_grasp_plotter
  rclcpp
  visualization_msgs
)

# Link external libs
target_link_libraries(${PROJECT_NAME}_grasp_messages
  ${GPD_LIB}
  ${PCL_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_grasp_plotter
  ${GPD_LIB}
  ${PCL_LIBRARIES}
)

# Link generated messages
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(${PROJECT_NAME}_grasp_messages
  ${cpp_typesupport_target}
)

target_link_libraries(${PROJECT_NAME}_grasp_plotter
  ${cpp_typesupport_target}
)

# -----------------------------
# Executables
# -----------------------------
add_executable(detect_grasps
  src/grasp_detection_node.cpp
)

add_executable(detect_grasps_server
  src/grasp_detection_server.cpp
)

# Include dirs
target_include_directories(detect_grasps PRIVATE
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_include_directories(detect_grasps_server PRIVATE
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# ROS dependencies
ament_target_dependencies(detect_grasps
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  pcl_conversions
)

ament_target_dependencies(detect_grasps_server
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  pcl_conversions
)

# Link libraries
target_link_libraries(detect_grasps
  ${PROJECT_NAME}_grasp_messages
  ${PROJECT_NAME}_grasp_plotter
  ${GPD_LIB}
  ${PCL_LIBRARIES}
)

target_link_libraries(detect_grasps_server
  ${PROJECT_NAME}_grasp_messages
  ${PROJECT_NAME}_grasp_plotter
  ${GPD_LIB}
  ${PCL_LIBRARIES}
)

# -----------------------------
# Install
# -----------------------------
install(TARGETS
  detect_grasps
  detect_grasps_server
  ${PROJECT_NAME}_grasp_messages
  ${PROJECT_NAME}_grasp_plotter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# -----------------------------
# Testing
# -----------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()