cmake_minimum_required(VERSION 2.8.3)
project(cpp_avoidance)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  laser_geometry
  sensor_msgs
  tf2_ros
  tf
  pcl_conversions
  pcl_ros
  visualization_msgs
  tf2_sensor_msgs
)
find_package(octomap REQUIRED)
## Declare a catkin package
catkin_package()

## Build
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  include/avoidance/
)

## Declare a C++ executable
add_executable(avoidance_node src/avoidance.cpp)
add_executable(octomap_node src/octomap.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(avoidance_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(octomap_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(avoidance_node
  ${catkin_LIBRARIES}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_LIBRARIES}
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  # include/avoidance
)
target_link_libraries(octomap_node
  ${catkin_LIBRARIES}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_LIBRARIES}
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  # include/avoidance
)


catkin_install_python(PROGRAMS src/mothernode.py src/path.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
