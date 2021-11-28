cmake_minimum_required(VERSION 3.0.2)
project(walker_turtlebot)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  move_base_msgs
  roscpp
  sensor_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs move_base_msgs
)
###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
${catkin_INCLUDE_DIRS}
)
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/walker_turtlebot_node.cpp)
## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
add_executable(walker_bot src/main.cpp src/turtlebot_walker.cpp)
#############
## Install ##
#############
target_link_libraries(walker_bot
  ${catkin_LIBRARIES}
)

