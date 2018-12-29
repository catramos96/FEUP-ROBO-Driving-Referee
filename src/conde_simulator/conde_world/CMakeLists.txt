cmake_minimum_required(VERSION 2.8.3)
project(conde_world)

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
add_library(contact_1 SHARED ContactPlugin.cc)
add_library(contact_2 SHARED ContactSemaphorePlugin.cc)
target_link_libraries(contact_1 ${GAZEBO_LIBRARIES})
target_link_libraries(contact_2 ${GAZEBO_LIBRARIES})
add_library(contact SHARED ContactPlugin.cc)
target_link_libraries(contact ${GAZEBO_LIBRARIES})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES conde_world
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)