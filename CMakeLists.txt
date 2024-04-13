cmake_minimum_required(VERSION 3.0.2)
project(sonar_reader)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CATKIN_DEPENDENCIES
  roscpp
  mrs_lib
  std_msgs
  )

set(LIBRARIES
)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
)

## System dependencies are found with CMake's conventions
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${LIBRARIES}
 CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
#  DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_executable(sonar_reader_node src/sonar_reader.cpp)

target_link_libraries(sonar_reader_node
  ${catkin_LIBRARIES}
)

