cmake_minimum_required(VERSION 3.0.2)
project(my_avoidance)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  pcl_conversions
  pcl_ros
  # ompl
)
# find_package(octomap REQUIRED)
# find_package(fcl REQUIRED)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES avoidance
 CATKIN_DEPENDS rospy std_msgs geometry_msgs 
#  DEPENDS system_lib
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  # ${OCTOMAP_INCLUDE_DIRS}
	# ${OMPL_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS  src/takeoff_node.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# add_executable(path_planning_node src/path_planning_node src/path_planning)
# target_link_libraries(path_planning_node
# 	${catkin_LIBRARIES}
# 	${OMPL_LIBRARIES}
# 	${FCL_LIBRARIES}
# )

