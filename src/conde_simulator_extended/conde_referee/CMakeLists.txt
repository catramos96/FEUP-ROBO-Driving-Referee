cmake_minimum_required(VERSION 2.8.3)
project(conde_referee)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
 add_executable(conde_referee_node src/conde_referee_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
 add_dependencies(conde_referee_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(conde_referee_node
   ${catkin_LIBRARIES}
 )