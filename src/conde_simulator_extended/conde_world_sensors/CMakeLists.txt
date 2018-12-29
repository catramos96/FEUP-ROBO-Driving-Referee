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

# Build our plugin
add_library(contact_borders SHARED ContactPlugin.cc)
add_library(contact_semaphore SHARED ContactSemaphorePlugin.cc)
add_library(contact_waypoints SHARED ContactWaypointsPlugin.cc)
target_link_libraries(contact_borders ${GAZEBO_LIBRARIES})
target_link_libraries(contact_semaphore ${GAZEBO_LIBRARIES})
target_link_libraries(contact_waypoints ${GAZEBO_LIBRARIES})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES conde_world
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)