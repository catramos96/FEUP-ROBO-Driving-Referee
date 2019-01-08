cmake_minimum_required(VERSION 2.8.3)
project(conde_world_sensors)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp  
  std_msgs
)

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Utils header file
include_directories(
  ${RESOURCES_DIRECTORY}
)

set(RESOURCES 
  ${RESOURCES_DIRECTORY}/utils.cpp ${RESOURCES_DIRECTORY}/utils.h
)

# Build our plugin
add_library(contact_borders SHARED ContactPlugin.cc)
add_library(contact_waypoints SHARED ContactWaypointsPlugin.cc)
add_library(contact_parking SHARED ContactParkingPlugin.cc)
target_link_libraries(contact_borders ${GAZEBO_LIBRARIES})
target_link_libraries(contact_waypoints ${GAZEBO_LIBRARIES})
target_link_libraries(contact_parking ${GAZEBO_LIBRARIES})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES conde_world
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)