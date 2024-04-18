cmake_minimum_required(VERSION 3.5)
project(sonar_reader)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CATKIN_DEPENDENCIES
  mrs_lib
  nodelet
  roscpp
  std_msgs
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nodelet_basics
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_library(Sonar_sonar_reader src/sonar_reader.cpp)
 
## Link libraries
target_link_libraries(Sonar_sonar_reader ${catkin_LIBRARIES}) 
