cmake_minimum_required(VERSION 2.8.3)
project(conde_world)

find_package(catkin REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES conde_world
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin
add_library(contact_1 SHARED ContactPlugin.cc)
add_library(contact_2 SHARED ContactSemaphorePlugin.cc)
target_link_libraries(contact_1 ${GAZEBO_LIBRARIES})
target_link_libraries(contact_2 ${GAZEBO_LIBRARIES})