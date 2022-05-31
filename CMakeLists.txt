cmake_minimum_required(VERSION 3.0.2)
project(global_planner)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
)

find_package(PCL REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES global_planner
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/global_planner.cpp
# )

############ UNCOMMENT THE REQIURED ALGORITHM #################

#add_library(global_planner_lib src/bfs_planner.cpp)
add_library(global_planner_lib src/dijkstra_planner.cpp)
#add_library(global_planner_lib src/astar_planner.cpp)
#add_library(global_planner_lib src/rrt_planner.cpp)

